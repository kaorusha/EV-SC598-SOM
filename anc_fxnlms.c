#include "anc_fxnlms.h"

// SHARC+ L1 雙匯流排記憶體變數 (PM 放權重，DM 放歷史延遲線)
// 使用標準 #pragma section 語法，兼具 CCES 硬體記憶體映射與 Eclipse CDT / IDE 語法解析相容性
#pragma section("seg_pmda")
static float w_coeffs[ANC_W_TAPS];

// 採用雙倍鏡像緩衝區 (Mirror Buffer: 2 * HIST_SIZE)
// 徹底消除環狀折返與分支判斷，保證迴圈固定 1024/2048 次，由硬體雙 MAC SIMD 向量化，永不下溢越界
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
 * @brief 2048 階硬體向量化 FIR 卷積 (固定 2048 次，編譯器 SIMD Dual-MAC 1 Cycle/Tap，零分支、永不越界)
 */
static inline float fir_conv_2048(const float *h, const float *hist_ptr)
{
    float sum = 0.0f;
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(2048, 2048, 2)
#endif
    for (j = 0; j < 2048; j++)
    {
        sum += h[j] * (*hist_ptr--);
    }
    return sum;
}

/**
 * @brief 1024 階硬體向量化 FIR 卷積 (固定 1024 次，編譯器 SIMD Dual-MAC 1 Cycle/Tap，零分支、永不越界)
 */
static inline float fir_conv_1024(const float *h, const float *hist_ptr)
{
    float sum = 0.0f;
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(1024, 1024, 2)
#endif
    for (j = 0; j < 1024; j++)
    {
        sum += h[j] * (*hist_ptr--);
    }
    return sum;
}

/**
 * @brief 1024 階硬體向量化 LMS 權重更新 (固定 1024 次，編譯器 SIMD 1.5 Cycle/Tap，零分支、永不越界)
 */
static inline void lms_update_1024(float *w, const float *hist_ptr, float grad_step, float leak)
{
    int j;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(1024, 1024, 2)
#endif
    for (j = 0; j < 1024; j++)
    {
        w[j] = leak * w[j] + grad_step * (*hist_ptr--);
    }
}

// 區塊級即時處理函式 (直接處理 SPORT DMA TDM4 四聲道數據: 每次步進 i += 4)
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count)
{
    int i;

    for (i = 0; i < count; i += 4)
    {
        // 1. 定點轉浮點 (TDM 輸入映射: Slot 0 = pRxBuf[i] 為誤差麥 d; Slot 1 = pRxBuf[i+1] 為參考麥 x)
        float d_n = (float)pRxBuf[i]   * (1.0f / 2147483648.0f); // 誤差麥克風 (Error Mic，Slot 0)
        float x_n = (float)pRxBuf[i+1] * (1.0f / 2147483648.0f); // 參考麥克風 (Reference Mic，Slot 1)

        // 2. 【反饋中和 (Feedback Neutralization)】
        int k_y = (y_buf_idx > 0) ? (y_buf_idx - 1) : (ANC_Y_HIST_SIZE - 1);
        const float *py = &y_history[k_y + ANC_Y_HIST_SIZE];
        float f_leak  = fir_conv_2048(FEEDBACK_PATH, py);
        float u_hat_n = x_n - f_leak;

        // 3. 雙鏡像更新參考訊號歷史緩衝區
        x_history[anc_buf_idx] = u_hat_n;
        x_history[anc_buf_idx + ANC_X_HIST_SIZE] = u_hat_n;

        // 4. 【抗噪波卷積】y(n) = W * X
        const float *px = &x_history[anc_buf_idx + ANC_X_HIST_SIZE];
        float y_n = fir_conv_1024(w_coeffs, px);

        // 5. 雙鏡像更新抗噪輸出歷史
        y_history[y_buf_idx] = y_n;
        y_history[y_buf_idx + ANC_Y_HIST_SIZE] = y_n;
        if (++y_buf_idx >= ANC_Y_HIST_SIZE) y_buf_idx = 0;

        // 6. 【次級路徑濾波】計算濾波參考訊號 x'(n) = Ŝ * X
        float x_prime_n = fir_conv_2048(SECOND_PATH, px);
        x_filt_history[anc_buf_idx] = x_prime_n;
        x_filt_history[anc_buf_idx + ANC_X_HIST_SIZE] = x_prime_n;

        // 7. 誤差訊號
        float e_n = d_n;

        // 8. 功率追蹤與歸一化步長 (NLMS)
        px_power = 0.999f * px_power + 0.001f * (x_prime_n * x_prime_n);
        float p_eff = px_power * (float)ANC_W_TAPS + ANC_EPSILON;
        float norm_mu = ANC_MU / p_eff;
        if (norm_mu > ANC_MAX_MU) norm_mu = ANC_MAX_MU; 

        // 9. 權重梯度更新
        float grad_step = norm_mu * e_n;
        float leak = 1.0f - ANC_LEAK;
        const float *pxf = &x_filt_history[anc_buf_idx + ANC_X_HIST_SIZE];
        lms_update_1024(w_coeffs, pxf, grad_step, leak);

        // 10. 循環指標自增
        if (++anc_buf_idx >= ANC_X_HIST_SIZE) anc_buf_idx = 0;

        // 11. 數值保護與安全限幅
        if (y_n != y_n) y_n = 0.0f; // NaN 熔斷保護
        if (y_n > ANC_SAFE_OUTPUT_LIMIT)  y_n = ANC_SAFE_OUTPUT_LIMIT;
        if (y_n < -ANC_SAFE_OUTPUT_LIMIT) y_n = -ANC_SAFE_OUTPUT_LIMIT;

        // 12. 寫入 TDM4 輸出緩衝區 (4 channels per audio frame):
        //   - 喇叭端 (J17): 左聲道(Tip)靜音=0，右聲道(Ring)輸出抗噪波 y(n)
        //   - 監聽端 (J18): 左聲道(Tip)=Error Mic, 右聲道(Ring)=Ref Mic
#if (ANC_OUTPUT_MODE == 0)
        // [模式 0: 直通硬體驗證模式]
        pTxBuf[i + 0] = 0;           // J17 左聲道 (Tip): 靜音 = 0
        pTxBuf[i + 1] = pRxBuf[i+1]; // J17 右聲道 (Ring): 播放 Reference Mic 音樂
        pTxBuf[i + 2] = pRxBuf[i];   // J18 左聲道 (Tip): 原始 Error Mic
        pTxBuf[i + 3] = pRxBuf[i+1]; // J18 右聲道 (Ring): 原始 Reference Mic
#else
        // [模式 1: 正式 ANC 降噪 + 獨立監聽模式]
        pTxBuf[i + 0] = 0;                          // J17 左聲道 (Tip): 自動靜音 = 0
        pTxBuf[i + 1] = (int)(y_n * 2147483647.0f); // J17 右聲道 (Ring): 反相抗噪波 y(n)
        pTxBuf[i + 2] = pRxBuf[i];                  // J18 左聲道 (Tip): 原始 Error Mic
        pTxBuf[i + 3] = pRxBuf[i+1];                // J18 右聲道 (Ring): 原始 Reference Mic
#endif
    }
}
