#include "anc_fxnlms.h"

// 卷積函式固定 2048 / 1024 次迴圈，路徑長度改變時必須同步修改
#if (ANC_S_TAPS != 2048) || (ANC_F_TAPS != 2048) || (ANC_W_TAPS != 1024)
#error "fir_conv_2048 / fir_conv_1024 / lms_update_1024 assume S = F = 2048 taps, W = 1024 taps"
#endif

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
 * @brief 1024 階 FIR 卷積 (固定 1024 次，兩指標同向遞增，供編譯器 SIMD 向量化)
 * @param h_rev       反轉係數 (h_rev[k] = h[1023-k])
 * @param hist_oldest 指向 x(n-1023)，hist_oldest[1023] = x(n)
 */
static inline float fir_conv_1024(const float ANC_PM * restrict h_rev, const float * restrict hist_oldest)
{
    float sum = 0.0f;
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(1024, 1024, 2)
#endif
    for (j = 0; j < 1024; j++)
    {
        sum += h_rev[j] * hist_oldest[j];
    }
    return sum;
}

/**
 * @brief 1024 階 LMS 權重更新 (固定 1024 次，兩指標同向遞增，供編譯器 SIMD 向量化)
 *        漏洩不在此處逐點相乘 (會讓迴圈從 1.0 變 1.5 cycle/tap)，改由 lms_leak_1024 每區塊套用一次
 * @param w_rev       反轉權重 (w_rev[k] = w[1023-k])
 * @param hist_oldest 指向 x'(n-1023)，hist_oldest[1023] = x'(n)
 */
static inline void lms_update_1024(float ANC_PM * restrict w_rev, const float * restrict hist_oldest, float grad_step)
{
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(1024, 1024, 2)
#endif
    for (j = 0; j < 1024; j++)
    {
        w_rev[j] += grad_step * hist_oldest[j];
    }
}

/**
 * @brief 區塊級漏洩: w *= (1 - ANC_LEAK)^frames，等效於逐取樣漏洩 (差異為 ANC_LEAK * grad_step 的二階項)
 */
static inline void lms_leak_1024(float ANC_PM * restrict w_rev, float leak_block)
{
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(1024, 1024, 2)
#endif
    for (j = 0; j < 1024; j++)
    {
        w_rev[j] *= leak_block;
    }
}

// 區塊級即時處理函式 (直接處理 SPORT DMA TDM4 四聲道數據: 每次步進 i += 4)
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count)
{
    int i;
    float leak_block = 1.0f; // 累積本區塊的漏洩係數 (1 - ANC_LEAK)^frames

    for (i = 0; i < count; i += 4)
    {
        // 1. 定點轉浮點 (TDM 輸入映射: Slot 0 = pRxBuf[i] 為誤差麥 d; Slot 1 = pRxBuf[i+1] 為參考麥 x)
        float d_n = (float)pRxBuf[i]   * (1.0f / 2147483648.0f); // 誤差麥克風 (Error Mic，Slot 0)
        float x_n = (float)pRxBuf[i+1] * (1.0f / 2147483648.0f); // 參考麥克風 (Reference Mic，Slot 1)

        // 2. 【反饋中和 (Feedback Neutralization)】 f_leak = F̂ * y，從 y(n-1) 往回 2048 點
        int k_y = (y_buf_idx > 0) ? (y_buf_idx - 1) : (ANC_Y_HIST_SIZE - 1);
        const float *py_oldest = &y_history[k_y + ANC_Y_HIST_SIZE - (ANC_F_TAPS - 1)];
        float f_leak  = fir_conv_2048(f_hat_rev, py_oldest);
        float u_hat_n = x_n - f_leak;

        // 3. 雙鏡像更新參考訊號歷史緩衝區
        x_history[anc_buf_idx] = u_hat_n;
        x_history[anc_buf_idx + ANC_X_HIST_SIZE] = u_hat_n;

        // 4. 【抗噪波卷積】y(n) = W * X
        const float *px = &x_history[anc_buf_idx + ANC_X_HIST_SIZE]; // 指向 x(n)
        float y_n = fir_conv_1024(w_coeffs, px - (ANC_W_TAPS - 1));

        // [P0 修正]: 必須先限幅，才能寫入反饋歷史。否則估計的反饋波形與實際喇叭播出的波形會產生致命偏差！
        if (y_n != y_n) y_n = 0.0f; // NaN 熔斷保護
        if (y_n > ANC_SAFE_OUTPUT_LIMIT)  y_n = ANC_SAFE_OUTPUT_LIMIT;
        if (y_n < -ANC_SAFE_OUTPUT_LIMIT) y_n = -ANC_SAFE_OUTPUT_LIMIT;

        // 5. 雙鏡像更新抗噪輸出歷史 (寫入限幅後的 y_n)
        y_history[y_buf_idx] = y_n;
        y_history[y_buf_idx + ANC_Y_HIST_SIZE] = y_n;
        if (++y_buf_idx >= ANC_Y_HIST_SIZE) y_buf_idx = 0;

        // 6. 【次級路徑濾波】計算濾波參考訊號 x'(n) = Ŝ * X
        float x_prime_n = fir_conv_2048(s_hat_rev, px - (ANC_S_TAPS - 1));
        x_filt_history[anc_buf_idx] = x_prime_n;
        x_filt_history[anc_buf_idx + ANC_X_HIST_SIZE] = x_prime_n;

        // 7. 誤差訊號
        float e_n = d_n;

        // 8. 功率追蹤與歸一化步長 (NLMS)
        // 使用 IIR 濾波器計算參考訊號的短期平均功率
        px_power = ANC_PWR_SMOOTH_ALPHA * px_power + ANC_PWR_SMOOTH_BETA * (x_prime_n * x_prime_n);
        float p_eff = px_power * (float)ANC_W_TAPS + ANC_EPSILON;
        float norm_mu = ANC_MU / p_eff;
        if (norm_mu > ANC_MAX_MU) norm_mu = ANC_MAX_MU;

        // 9. 權重梯度更新 (漏洩於區塊結尾統一套用)
        float grad_step = norm_mu * e_n;
        const float *pxf = &x_filt_history[anc_buf_idx + ANC_X_HIST_SIZE]; // 指向 x'(n)
        lms_update_1024(w_coeffs, pxf - (ANC_W_TAPS - 1), grad_step);
        leak_block *= (1.0f - ANC_LEAK);

        // 10. 循環指標自增
        if (++anc_buf_idx >= ANC_X_HIST_SIZE) anc_buf_idx = 0;

        // 12. 寫入 TDM4 輸出緩衝區 (4 channels per audio frame):
        //   - 喇叭端 (J18): 左聲道(Tip)靜音=0，右聲道(Ring)輸出抗噪波 y(n)
        //   - 監聽端 (J4/J17): 左聲道(Tip)=Error Mic, 右聲道(Ring)=Ref Mic
        pTxBuf[i + 0] = 0;                          // J18 左聲道 (Tip): 自動靜音 = 0
        pTxBuf[i + 1] = (int)(y_n * 2147483647.0f); // J18 右聲道 (Ring): 反相抗噪波 y(n)
        pTxBuf[i + 2] = pRxBuf[i];                  // J17 左聲道 (Tip): 原始 Error Mic
        pTxBuf[i + 3] = pRxBuf[i+1];                // J17 右聲道 (Ring): 原始 Reference Mic
    }

    // 11. 區塊級漏洩 (防止阻帶發散)
    lms_leak_1024(w_coeffs, leak_block);
}
