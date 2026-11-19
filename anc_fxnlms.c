#include "anc_fxnlms.h"
#include "anc_ref_band.h"

// 卷積函式固定 2048 次迴圈，路徑長度改變時必須同步修改
#if (ANC_S_TAPS != 2048) || (ANC_F_TAPS != 2048) || (ANC_W_TAPS != 2048)
#error "fir_conv_2048 / lms_update_2048 / lms_leak_2048 assume S = F = W = 2048 taps"
#endif
// Ŝ / F̂ 標頭必須是 48 kHz 核心取樣 (tools/export_paths.py)。_FS 含 (uint32_t) 轉型無法放進 #if，改用陣列大小檢查:
// 不符時陣列大小為 -1，編譯失敗
typedef char anc_path_headers_must_be_48k[((SECOND_PATH_TAPS_FS == ANC_CORE_FS) && (FEEDBACK_PATH_TAPS_FS == ANC_CORE_FS)) ? 1 : -1];

// SHARC+ L1 雙匯流排記憶體配置: 係數放 PM、歷史延遲線放 DM，卷積時每 cycle 可同時讀取兩者
// 使用標準 #pragma section 語法，兼具 CCES 硬體記憶體映射與 Eclipse CDT / IDE 語法解析相容性
//
// 所有係數一律「反轉存放」: h_rev[k] = h[N-1-k]
// 卷積 sum_j h[j] * x(n-j) 改寫為 sum_k h_rev[k] * x(n-N+1+k)，係數與歷史指標同向遞增，
// SIMD 的 PEy 才能取到相鄰位址 (+1)，編譯器得以向量化 (原本歷史指標遞減，無法 SIMD)
//
// ANC_PM: SHARC 的 pm 型別限定字，告訴編譯器係數走 PM 匯流排，卷積迴圈才能同一 cycle 同時讀係數與歷史
// (實測 cc21k -annotate: 未加 pm 為 1 cycle/tap，加 pm 為 0.5 cycle/tap)
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
#define ANC_PM pm
#else
#define ANC_PM
#endif

#pragma section("seg_pmda")
static float ANC_PM w_coeffs[ANC_W_TAPS];   // W(z) 自適應權重 (反轉存放)

#pragma section("seg_pmda")
static float ANC_PM s_hat_rev[ANC_S_TAPS];  // Ŝ(z) 次級路徑模型 (SECOND_PATH 反轉副本)

#pragma section("seg_pmda")
static float ANC_PM f_hat_rev[ANC_F_TAPS];  // F̂(z) 反饋路徑模型 (FEEDBACK_PATH 反轉副本)

// 採用雙倍鏡像緩衝區 (Mirror Buffer: 2 * HIST_SIZE)
// 徹底消除環狀折返與分支判斷，保證迴圈固定 1024/2048 次，永不下溢越界
#pragma section("seg_dmda")
static float x_history[2 * ANC_X_HIST_SIZE];

#pragma section("seg_dmda")
static float x_filt_history[2 * ANC_X_HIST_SIZE];

#pragma section("seg_dmda")
static float y_history[2 * ANC_Y_HIST_SIZE];

static float px_power    = 0.01f;
static int   anc_buf_idx = 0;
static int   y_buf_idx   = 0;
static float leak_acc    = 1.0f;   // 尚未套用的累積漏洩 (1 - ANC_LEAK)^n
static int   leak_count  = 0;

// 192 kHz <-> 48 kHz 邊界 (anc_rate.h): 誤差麥與參考麥各一個降頻器，抗噪波一個升頻器
static ANC_Rate_State dec_err, dec_ref, int_y;

// 迴路內參考帶通 (anc_ref_band.h)
static ANC_Ref_Band_State ref_band;

// 延遲啟動: wait_samples 數到 0 之前 y = 0、W 不更新 (中斷裡遞減，main loop 經 ANC_Get_Status 讀)
static volatile uint32_t wait_samples = 0u;
static volatile uint32_t run_samples  = 0u;   // 啟動後的核心取樣數

// 狀態統計: 中斷裡累加，ANC_Get_Status 讀出後清除。和 cycle meter 一樣不關中斷，
// 讀與清之間進來的一兩個取樣可能算到下一段或遺漏，對 ~2 s 的統計可忽略
static volatile uint32_t st_samples = 0u, st_clipped = 0u;
static volatile float    st_ref_sum = 0.0f, st_err_sum = 0.0f, st_y_peak = 0.0f;

// 發散保護 (anc_fxnlms.h 的 ANC_TRIP_*): 中斷裡更新；判定當時的數值由 main loop 經 ANC_Get_Trip 讀
#define ANC_TRIP_ALPHA      (1.0f / (ANC_TRIP_TAU_S * (float)ANC_CORE_FS))               // 指數平均的新值權重
#define ANC_TRIP_FADE_STEP  (1000.0f / ((float)ANC_TRIP_FADE_MS * (float)ANC_CORE_FS))  // 每個核心取樣的淡出量
#define ANC_TRIP_QUIET_SAMPLES ((ANC_TRIP_QUIET_MS * ANC_CORE_FS) / 1000u)              // 淡出完再等這麼多取樣才回報
static float clip_avg = 0.0f;   // 截斷比例
static float zc_avg   = 0.0f;   // 過零率 (每個核心取樣)
static float zc_wt    = 0.0f;   // 同樣平均的 1。zc_avg / zc_wt 補正從 0 起算的偏低 (啟動後不久就判定時)
static float y_hold   = 0.0f;   // 限幅前 |y| 的峰值，以 ANC_TRIP_TAU_S 衰減
static bool  y_neg    = false;  // 上一個 y 為負 (過零偵測)
static float out_gain = 1.0f;   // 判定後從 1 線性降到 0
static volatile uint32_t quiet_samples = 0u;   // 淡出完後的取樣數 (數到 ANC_TRIP_QUIET_SAMPLES 為止)
static volatile ANC_Trip_Reason trip_reason = ANC_TRIP_NONE;
static volatile float trip_seconds = 0.0f, trip_clip = 0.0f, trip_peak = 0.0f, trip_hz = 0.0f;

// 喇叭輸出開關 (anc_fxnlms.h 的 ANC_SWITCH_FADE_MS / ANC_AUTO_OFF_S): out_req 由 ANC_Set_Output 或定時關閉寫入，
// 其餘只在中斷裡改
#define ANC_SWITCH_STEP (1000.0f / ((float)ANC_SWITCH_FADE_MS * (float)ANC_CORE_FS))   // 每個核心取樣的增益變化
static volatile bool  out_req  = true;          // 要求的狀態: true = 開
static bool           out_prev = true;          // 中斷上一個取樣看到的 out_req (偵測切換)
static volatile float sw_gain  = 1.0f;          // 開關的淡入淡出增益 0..1
static volatile uint32_t off_quiet = 0u;        // 完全關上後的取樣數 (數到 ANC_TRIP_QUIET_SAMPLES 為止)，切換時歸零
static volatile float off_seconds  = 0.0f;      // 最近一次關閉時，ANC 啟動後的秒數
static volatile bool  auto_off_done = false;    // ANC_AUTO_OFF_S 已經關過

#if ANC_DIAG
// 診斷紀錄 (anc_fxnlms.h 的 ANC_DIAG): ANC 運轉中在中斷裡記錄，判定發散時凍結。放 L2，L1 留給延遲線與係數
#define ANC_DIAG_TICK ((ANC_DIAG_TICK_MS * ANC_CORE_FS) / 1000u)   // 每筆的核心取樣數
#pragma section("seg_l2_bsz_data", ZERO_INIT)
static float diag_snap[ANC_DIAG_SIGNALS * ANC_DIAG_SNAP];   // 訊號 s 的第 j 個在 [s * ANC_DIAG_SNAP + j]
#pragma section("seg_l2_bsz_data", ZERO_INIT)
static ANC_Diag_Rec diag_hist[ANC_DIAG_HIST];
static volatile uint32_t diag_snap_idx = 0u, diag_snap_n = 0u;   // 下一個寫入位置、有效取樣數 (最多 ANC_DIAG_SNAP)
static volatile uint32_t diag_hist_n = 0u;                       // 已記錄的筆數
static uint32_t dg_n = 0u, dg_clip = 0u;                         // 這一筆的取樣數、被限幅的取樣數
static float dg_ypk = 0.0f, dg_x = 0.0f, dg_fy = 0.0f, dg_u = 0.0f, dg_y = 0.0f, dg_e = 0.0f;

/** @brief 結束目前這一筆 (每 ANC_DIAG_TICK 個取樣，或判定發散時)。W2 的 2048 次乘加只在這裡做，每 0.25 s 一次 */
static void diag_store(void)
{
    ANC_Diag_Rec *r = &diag_hist[diag_hist_n % ANC_DIAG_HIST];
    float n = (dg_n > 0u) ? (float)dg_n : 1.0f;
    float w2 = 0.0f;
    int j;
    for (j = 0; j < ANC_W_TAPS; j++)
    {
        w2 += w_coeffs[j] * w_coeffs[j];
    }
    r->t     = (float)run_samples / (float)ANC_CORE_FS;
    r->y_pk  = dg_ypk;
    r->clip  = (float)dg_clip / n;
    r->x_ms  = dg_x / n;
    r->fy_ms = dg_fy / n;
    r->u_ms  = dg_u / n;
    r->y_ms  = dg_y / n;
    r->e_ms  = dg_e / n;
    r->w2    = w2;
    diag_hist_n++;
    dg_n    = 0u;
    dg_clip = 0u;
    dg_ypk  = 0.0f;
    dg_x    = 0.0f;
    dg_fy   = 0.0f;
    dg_u    = 0.0f;
    dg_y    = 0.0f;
    dg_e    = 0.0f;
}
#endif

/** @brief 判定發散，記下當時的數值。trip_reason 最後寫：main loop 看到它時，其他欄位已經寫好 */
static void anc_trip(ANC_Trip_Reason reason)
{
#if ANC_DIAG
    if (dg_n > 0u) diag_store();   // 判定前不滿一筆的最後一段
#endif
    trip_seconds = (float)run_samples / (float)ANC_CORE_FS;
    trip_clip    = clip_avg;
    trip_peak    = y_hold;
    trip_hz      = (zc_wt > 0.0f) ? (zc_avg / zc_wt) * (float)ANC_CORE_FS * 0.5f : 0.0f;
    trip_reason  = reason;
}

// 初始化函式
void ANC_Init(void)
{
    int i;
    for (i = 0; i < ANC_W_TAPS; i++)
    {
        w_coeffs[i] = 0.0f;
    }
    for (i = 0; i < ANC_S_TAPS; i++)
    {
        s_hat_rev[i] = SECOND_PATH[ANC_S_TAPS - 1 - i];
    }
    for (i = 0; i < ANC_F_TAPS; i++)
    {
        f_hat_rev[i] = FEEDBACK_PATH[ANC_F_TAPS - 1 - i];
    }
    for (i = 0; i < 2 * ANC_X_HIST_SIZE; i++)
    {
        x_history[i] = 0.0f;
        x_filt_history[i] = 0.0f;
    }
    for (i = 0; i < 2 * ANC_Y_HIST_SIZE; i++)
    {
        y_history[i] = 0.0f;
    }
    px_power    = 0.01f;
    anc_buf_idx = 0;
    y_buf_idx   = 0;
    leak_acc    = 1.0f;
    leak_count  = 0;
    anc_rate_reset(&dec_err);
    anc_rate_reset(&dec_ref);
    anc_rate_reset(&int_y);
    anc_ref_band_reset(&ref_band);
    wait_samples = ANC_START_DELAY_S * ANC_CORE_FS;
    run_samples  = 0u;
    st_samples   = 0u;
    st_clipped   = 0u;
    st_ref_sum   = 0.0f;
    st_err_sum   = 0.0f;
    st_y_peak    = 0.0f;
    clip_avg     = 0.0f;
    zc_avg       = 0.0f;
    zc_wt        = 0.0f;
    y_hold       = 0.0f;
    y_neg        = false;
    out_gain     = 1.0f;
    quiet_samples = 0u;
    trip_reason  = ANC_TRIP_NONE;
    out_req      = true;
    out_prev     = true;
    sw_gain      = 1.0f;
    off_quiet    = 0u;
    off_seconds  = 0.0f;
    auto_off_done = false;
#if ANC_DIAG
    diag_snap_idx = 0u;
    diag_snap_n   = 0u;
    diag_hist_n   = 0u;
    dg_n    = 0u;
    dg_clip = 0u;
    dg_ypk  = 0.0f;
    dg_x    = 0.0f;
    dg_fy   = 0.0f;
    dg_u    = 0.0f;
    dg_y    = 0.0f;
    dg_e    = 0.0f;
#endif
}

/**
 * @brief 2048 階 FIR 卷積 (固定 2048 次，兩指標同向遞增，供編譯器 SIMD 向量化)
 * @param h_rev       反轉係數 (h_rev[k] = h[2047-k])
 * @param hist_oldest 指向 x(n-2047)，hist_oldest[2047] = x(n)
 */
static inline float fir_conv_2048(const float ANC_PM * restrict h_rev, const float * restrict hist_oldest)
{
    float sum = 0.0f;
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(2048, 2048, 2)
#endif
    for (j = 0; j < 2048; j++)
    {
        sum += h_rev[j] * hist_oldest[j];
    }
    return sum;
}

/**
 * @brief 2048 階 LMS 權重更新 w += grad_step * x' (固定 2048 次，兩指標同向遞增，供編譯器 SIMD 向量化)
 *        梯度下降的負號由呼叫端放進 grad_step
 *        漏洩不在此處逐點相乘 (會讓迴圈從 1.0 變 1.5 cycle/tap)，改由 lms_leak_2048 每 ANC_LEAK_INTERVAL 個取樣套用一次
 * @param w_rev       反轉權重 (w_rev[k] = w[2047-k])
 * @param hist_oldest 指向 x'(n-2047)，hist_oldest[2047] = x'(n)
 */
static inline void lms_update_2048(float ANC_PM * restrict w_rev, const float * restrict hist_oldest, float grad_step)
{
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(2048, 2048, 2)
#endif
    for (j = 0; j < 2048; j++)
    {
        w_rev[j] += grad_step * hist_oldest[j];
    }
}

/**
 * @brief 累積漏洩: w *= (1 - ANC_LEAK)^n，等效於逐取樣漏洩 (差異為 ANC_LEAK * grad_step 的二階項)
 */
static inline void lms_leak_2048(float ANC_PM * restrict w_rev, float leak)
{
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(2048, 2048, 2)
#endif
    for (j = 0; j < 2048; j++)
    {
        w_rev[j] *= leak;
    }
}

/** @brief float 轉 int32 並飽和 (升頻濾波器的漣波會讓輸出略超過 ANC_SAFE_OUTPUT_LIMIT；限幅 0.9 時，一直被截斷的波形
 *         可能超過滿刻度，在這裡飽和) */
static inline int float_to_q31(float v)
{
    if (v >  0.9999999f) v =  0.9999999f;
    if (v < -1.0f)       v = -1.0f;
    return (int)(v * 2147483647.0f);
}

// 區塊級即時處理函式 (直接處理 SPORT DMA TDM4 四聲道數據)
// 每 ANC_DECIM 個 192 kHz frame 為一個 48 kHz 核心取樣: 降頻 -> FxNLMS -> 升頻
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count)
{
    int i, k;
    uint32_t n_core = 0u, n_clip = 0u;
    float ref_sum = 0.0f, err_sum = 0.0f, y_peak = 0.0f;

    for (i = 0; i < count; i += 4 * ANC_DECIM)
    {
        // 1. 降頻 (TDM 輸入映射: Slot 0 為誤差麥 d; Slot 1 為參考麥 x)
        //    每個 192 kHz frame 都經過抗混疊濾波，取這組最後一個 frame 的輸出當核心取樣 (延遲最小)
        float d_n = 0.0f, x_n = 0.0f;
        for (k = 0; k < ANC_DECIM; k++)
        {
            const int *pf = &pRxBuf[i + 4 * k];
            d_n = anc_rate_filter(&dec_err, (float)pf[0] * (1.0f / 2147483648.0f)); // 誤差麥克風 (Error Mic，Slot 0)
            x_n = anc_rate_filter(&dec_ref, (float)pf[1] * (1.0f / 2147483648.0f)); // 參考麥克風 (Reference Mic，Slot 1)
        }

        // 2. 【反饋中和 (Feedback Neutralization)】 f_leak = F̂ * y，從 y(n-1) 往回 2048 點
        int k_y = (y_buf_idx > 0) ? (y_buf_idx - 1) : (ANC_Y_HIST_SIZE - 1);
        const float *py_oldest = &y_history[k_y + ANC_Y_HIST_SIZE - (ANC_F_TAPS - 1)];
        float f_leak  = fir_conv_2048(f_hat_rev, py_oldest);
        float u_hat_n = x_n - f_leak;
#if (ANC_REF_BAND != ANC_REF_BAND_NONE)
        // 2b. 迴路內參考帶通 (anc_ref_band.h): 只讓要消的音調進 W 與 Ŝ，誤差麥不濾波
        u_hat_n = anc_ref_band_filter(&ref_band, u_hat_n);
#endif

        // 3. 雙鏡像更新參考訊號歷史緩衝區
        x_history[anc_buf_idx] = u_hat_n;
        x_history[anc_buf_idx + ANC_X_HIST_SIZE] = u_hat_n;

        // 3b. 喇叭輸出開關的切換 (ANC_Set_Output / ANC_AUTO_OFF_S)。這個取樣一律用 on_req，main loop 中途改寫也一致
        bool on_req = out_req;
        if (on_req != out_prev)
        {
            out_prev  = on_req;
            off_quiet = 0u;
            if (!on_req)
            {
                off_seconds = (float)run_samples / (float)ANC_CORE_FS;
#if ANC_DIAG
                if (dg_n > 0u) diag_store();   // 關閉前不滿一筆的最後一段
#endif
            }
        }

        // 4. 【抗噪波卷積】y(n) = W * X。發散保護淡出完，或開關關上且淡出完，y = 0 (不必算)
        const float *px = &x_history[anc_buf_idx + ANC_X_HIST_SIZE]; // 指向 x(n)
        bool live = (out_gain > 0.0f) && (on_req || (sw_gain > 0.0f));
        float y_n = live ? fir_conv_2048(w_coeffs, px - (ANC_W_TAPS - 1)) : 0.0f;

        // [P0 修正]: 必須先限幅，才能寫入反饋歷史。否則估計的反饋波形與實際喇叭播出的波形會產生致命偏差！
        if (y_n != y_n)   // NaN 熔斷保護: W 已經壞了，交給發散保護
        {
            y_n = 0.0f;
            if (trip_reason == ANC_TRIP_NONE) anc_trip(ANC_TRIP_NAN);
        }
#if ANC_DIAG
        if ((wait_samples == 0u) && (trip_reason == ANC_TRIP_NONE) && on_req)   // 快照: 輸出開著時一直覆寫，判定或關閉時凍結
        {
            uint32_t j = diag_snap_idx;
            diag_snap[0u * ANC_DIAG_SNAP + j] = x_n;       // 參考麥 (回授中和前)
            diag_snap[1u * ANC_DIAG_SNAP + j] = f_leak;    // 回授估計 F̂y
            diag_snap[2u * ANC_DIAG_SNAP + j] = u_hat_n;   // 帶通後、送進 W 的參考
            diag_snap[3u * ANC_DIAG_SNAP + j] = y_n;       // 限幅前的 y
            diag_snap[4u * ANC_DIAG_SNAP + j] = d_n;       // 誤差麥
            diag_snap_idx = (j + 1u < ANC_DIAG_SNAP) ? (j + 1u) : 0u;
            if (diag_snap_n < ANC_DIAG_SNAP) diag_snap_n++;
        }
#endif
        float y_abs = (y_n < 0.0f) ? -y_n : y_n;
        if (y_abs > y_peak) y_peak = y_abs;
        bool clipped = (y_abs > ANC_SAFE_OUTPUT_LIMIT);
        if (clipped)
        {
            y_n = (y_n > 0.0f) ? ANC_SAFE_OUTPUT_LIMIT : -ANC_SAFE_OUTPUT_LIMIT;
            n_clip++;
        }
#if ANC_DIAG
        if ((wait_samples == 0u) && (trip_reason == ANC_TRIP_NONE) && on_req)   // 每 ANC_DIAG_TICK_MS 一筆的統計
        {
            dg_n++;
            if (clipped) dg_clip++;
            if (y_abs > dg_ypk) dg_ypk = y_abs;
            dg_x  += x_n * x_n;
            dg_fy += f_leak * f_leak;
            dg_u  += u_hat_n * u_hat_n;
            dg_y  += y_n * y_n;   // 限幅後
            dg_e  += d_n * d_n;
            if (dg_n >= ANC_DIAG_TICK) diag_store();
        }
#endif

        // 4b. 發散保護 (anc_fxnlms.h 的 ANC_TRIP_*): ANC 啟動後追蹤截斷比例、過零率與 |y| 峰值；判定後 y 淡出到 0。
        //     只在輸出完全打開時累計 (關閉與淡入淡出期間 W 停住，平均值保留)
        if ((trip_reason == ANC_TRIP_NONE) && (wait_samples == 0u) && on_req && (sw_gain >= 1.0f))
        {
            bool neg = (y_n < 0.0f);
            clip_avg += ANC_TRIP_ALPHA * ((clipped ? 1.0f : 0.0f) - clip_avg);
            zc_avg   += ANC_TRIP_ALPHA * (((neg != y_neg) ? 1.0f : 0.0f) - zc_avg);
            zc_wt    += ANC_TRIP_ALPHA * (1.0f - zc_wt);
            y_hold    = (y_abs > y_hold) ? y_abs : y_hold * (1.0f - ANC_TRIP_ALPHA);
            y_neg     = neg;
            if (clip_avg > ANC_TRIP_CLIP_FRACTION) anc_trip(ANC_TRIP_CLIP);
        }
        if (trip_reason != ANC_TRIP_NONE)
        {
            y_n *= out_gain;
            if (out_gain > 0.0f)
            {
                out_gain = (out_gain > ANC_TRIP_FADE_STEP) ? (out_gain - ANC_TRIP_FADE_STEP) : 0.0f;
            }
            else if (quiet_samples < ANC_TRIP_QUIET_SAMPLES)
            {
                quiet_samples++;   // 升頻濾波器的拖尾和 DMA 緩衝都歸零後，main loop 才印 log
            }
        }

        // 4c. 喇叭輸出開關: sw_gain 在 ANC_SWITCH_FADE_MS 內線性淡入 / 淡出
        y_n *= sw_gain;
        if (on_req)
        {
            sw_gain = (sw_gain < 1.0f - ANC_SWITCH_STEP) ? (sw_gain + ANC_SWITCH_STEP) : 1.0f;
        }
        else if (sw_gain > 0.0f)
        {
            sw_gain = (sw_gain > ANC_SWITCH_STEP) ? (sw_gain - ANC_SWITCH_STEP) : 0.0f;
        }
        else if (off_quiet < ANC_TRIP_QUIET_SAMPLES)
        {
            off_quiet++;   // 升頻濾波器的拖尾和 DMA 緩衝都歸零後，main loop 才印
        }

        // 5. 雙鏡像更新抗噪輸出歷史 (寫入限幅、乘過保護與開關增益後實際送出的 y_n)
        y_history[y_buf_idx] = y_n;
        y_history[y_buf_idx + ANC_Y_HIST_SIZE] = y_n;
        if (++y_buf_idx >= ANC_Y_HIST_SIZE) y_buf_idx = 0;

        // 6. 【次級路徑濾波】計算濾波參考訊號 x'(n) = Ŝ * X
        float x_prime_n = fir_conv_2048(s_hat_rev, px - (ANC_S_TAPS - 1));
        x_filt_history[anc_buf_idx] = x_prime_n;
        x_filt_history[anc_buf_idx + ANC_X_HIST_SIZE] = x_prime_n;

        // 7. 誤差訊號
        float e_n = d_n;

        // 8. 功率追蹤: IIR 平滑的 x' 短期平均功率 (等待期間照常更新，ANC 啟動時已收斂)
        px_power = ANC_PWR_SMOOTH_ALPHA * px_power + ANC_PWR_SMOOTH_BETA * (x_prime_n * x_prime_n);

        // 9. NLMS 權重更新，梯度下降: e = d + S*y，e 對 w 的梯度是 x'，所以
        //    w -= mu * e * x' / (ANC_W_TAPS * Px' + eps)
        //    (舊版是 w += ...，y 送出時又不反相，等於往誤差變大的方向更新；Ŝ 與 S 同極性，負號必須在這裡)
        //    漏洩累積後每 ANC_LEAK_INTERVAL 個取樣統一套用，防止阻帶發散
        if (wait_samples == 0u)
        {
            // 判定發散後 W 停住，保留當時的值；輸出關閉與淡入淡出期間也停住 (誤差麥聽不到完整的輸出)
            if ((trip_reason == ANC_TRIP_NONE) && on_req && (sw_gain >= 1.0f))
            {
                float p_eff = px_power * (float)ANC_W_TAPS + ANC_EPSILON;
                float grad_step = -(ANC_MU / p_eff) * e_n;
                const float *pxf = &x_filt_history[anc_buf_idx + ANC_X_HIST_SIZE]; // 指向 x'(n)
                lms_update_2048(w_coeffs, pxf - (ANC_W_TAPS - 1), grad_step);
                leak_acc *= (1.0f - ANC_LEAK);
                if (++leak_count >= ANC_LEAK_INTERVAL)
                {
                    lms_leak_2048(w_coeffs, leak_acc);
                    leak_acc   = 1.0f;
                    leak_count = 0;
                }
            }
            if (run_samples != 0xFFFFFFFFu) run_samples++;
#if (ANC_AUTO_OFF_S > 0u)
            if (!auto_off_done && (run_samples >= ANC_AUTO_OFF_S * ANC_CORE_FS))
            {
                auto_off_done = true;
                out_req = false;   // 下一個取樣開始淡出 (3b)
            }
#endif
        }
        else
        {
            wait_samples--;   // 延遲啟動中: W 維持 0，y = 0
        }

        // 10. 循環指標自增
        if (++anc_buf_idx >= ANC_X_HIST_SIZE) anc_buf_idx = 0;

        // 11. 升頻: y(n) 補零 (乘 ANC_DECIM 補回通帶增益) 後經同一濾波器去鏡像，寫入 ANC_DECIM 個 TDM4 frame
        //   - 喇叭端 (J18): 左聲道(Tip)靜音=0，右聲道(Ring)輸出抗噪波
        //   - 監聽端 (J4/J17): 左聲道(Tip)=Error Mic, 右聲道(Ring)=Ref Mic (192 kHz 原始訊號)
        for (k = 0; k < ANC_DECIM; k++)
        {
            float y_hi = anc_rate_filter(&int_y, (k == 0) ? (float)ANC_DECIM * y_n : 0.0f);
            int  *pt   = &pTxBuf[i + 4 * k];
            const int *pf = &pRxBuf[i + 4 * k];
            pt[0] = 0;                  // J18 左聲道 (Tip): 自動靜音 = 0
            pt[1] = float_to_q31(y_hi); // J18 右聲道 (Ring): 抗噪波 y (不另外反相，見 anc_fxnlms.h)
            pt[2] = pf[0];              // J17 左聲道 (Tip): 原始 Error Mic
            pt[3] = pf[1];              // J17 右聲道 (Ring): 原始 Reference Mic
        }

        n_core++;
        ref_sum += x_n * x_n;
        err_sum += d_n * d_n;
    }

    // 狀態統計，main loop 由 ANC_Get_Status 讀出
    st_samples += n_core;
    st_clipped += n_clip;
    st_ref_sum += ref_sum;
    st_err_sum += err_sum;
    if (y_peak > st_y_peak) st_y_peak = y_peak;
}

bool ANC_Is_Running(void)
{
    return wait_samples == 0u;
}

void ANC_Get_Status(ANC_Status *st)
{
    uint32_t n = st_samples;
    float w2 = 0.0f;
    int j;

    st->samples = n;
    st->clipped = st_clipped;
    st->ref_ms  = (n > 0u) ? st_ref_sum / (float)n : 0.0f;
    st->err_ms  = (n > 0u) ? st_err_sum / (float)n : 0.0f;
    st->y_peak  = st_y_peak;
    st_samples  = 0u;
    st_clipped  = 0u;
    st_ref_sum  = 0.0f;
    st_err_sum  = 0.0f;
    st_y_peak   = 0.0f;

    st->running = (wait_samples == 0u);
    st->tripped = (trip_reason != ANC_TRIP_NONE);
    st->output_on = out_req;
    st->seconds = st->running ? (float)run_samples / (float)ANC_CORE_FS
                              : -(float)wait_samples / (float)ANC_CORE_FS;
    st->p_x     = px_power * (float)ANC_W_TAPS;
    for (j = 0; j < ANC_W_TAPS; j++)
    {
        w2 += w_coeffs[j] * w_coeffs[j];
    }
    st->w_energy = w2;
}

bool ANC_Get_Trip(ANC_Trip *t)
{
    ANC_Trip_Reason r = trip_reason;
    if ((r == ANC_TRIP_NONE) || (quiet_samples < ANC_TRIP_QUIET_SAMPLES))
    {
        return false;   // 淡出中也回 false: printf 會讓核心暫停，DMA 重播最後的緩衝，輸出卡在當時的值
    }
    t->reason    = r;
    t->seconds   = trip_seconds;
    t->clip_frac = trip_clip;
    t->y_peak    = trip_peak;
    t->freq_hz   = trip_hz;
    return true;
}

/** @brief 已判定發散、輸出還沒完全靜音 (淡出中，或等升頻濾波器與 DMA 緩衝歸零)。這時 printf 會讓輸出卡住 */
bool ANC_Trip_Pending(void)
{
    return (trip_reason != ANC_TRIP_NONE) && (quiet_samples < ANC_TRIP_QUIET_SAMPLES);
}

/** @brief 打開 / 關閉喇叭輸出。中斷在下一個取樣看到後淡入或淡出 (ANC_SWITCH_FADE_MS)，關閉期間 W 停住 */
void ANC_Set_Output(bool on)
{
    out_req = on;
}

bool ANC_Output_Requested(void)
{
    return out_req;
}

ANC_State ANC_Get_State(void)
{
    if (trip_reason != ANC_TRIP_NONE) return ANC_STATE_TRIPPED;
    if (wait_samples != 0u)           return ANC_STATE_WAIT;
    return out_req ? ANC_STATE_ON : ANC_STATE_OFF;
}

/** @brief 輸出已關、淡出完並維持 0 ANC_TRIP_QUIET_MS。off_quiet 在每次切換時歸零，剛要求關閉時不會誤報 */
bool ANC_Output_Quiet(void)
{
    return !out_req && (off_quiet >= ANC_TRIP_QUIET_SAMPLES);
}

bool ANC_Get_Auto_Off(float *seconds)
{
    if (!auto_off_done || !ANC_Output_Quiet()) return false;
    *seconds = off_seconds;
    return true;
}

#if ANC_DIAG
/** @brief 診斷紀錄的狀態歷史 (main loop 在判定、輸出歸零後讀；那時中斷已經不再寫入) */
uint32_t ANC_Diag_History(const ANC_Diag_Rec **ring)
{
    *ring = diag_hist;
    return diag_hist_n;
}

/** @brief 判定前最後 ANC_DIAG_SNAP 個核心取樣的快照 */
uint32_t ANC_Diag_Snapshot(const float **sig, uint32_t *oldest)
{
    *sig    = diag_snap;
    *oldest = (diag_snap_n < ANC_DIAG_SNAP) ? 0u : diag_snap_idx;
    return diag_snap_n;
}
#endif
