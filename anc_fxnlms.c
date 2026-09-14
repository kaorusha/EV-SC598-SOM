#include "anc_fxnlms.h"

#if defined(__GNUC__) || defined(__clang__) || defined(__INTELLISENSE__)
#define section(x) __attribute__((section(x)))
#elif !defined(__ADSP21000__) && !defined(__ADSPSHARC__)
#define section(x)
#endif

// SHARC+ L1 雙匯流排記憶體變數 (PM 放權重，DM 放歷史延遲線)
section("seg_pmda") static float w_coeffs[ANC_W_TAPS];
section("seg_dmda") static float x_history[ANC_X_HIST_SIZE];
section("seg_dmda") static float x_filt_history[ANC_X_HIST_SIZE];
section("seg_dmda") static float y_history[ANC_Y_HIST_SIZE];

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
    for (i = 0; i < ANC_X_HIST_SIZE; i++)
    {
        x_history[i] = 0.0f;
        x_filt_history[i] = 0.0f;
    }
    for (i = 0; i < ANC_Y_HIST_SIZE; i++)
    {
        y_history[i] = 0.0f;
    }
    px_power    = 0.01f;
    anc_buf_idx = 0;
    y_buf_idx   = 0;
}

/**
 * @brief 環狀緩衝區 FIR 卷積計算 (Circular Buffer Convolution: y = h * x)
 *
 * @param h           濾波器係數陣列 (長度 taps)
 * @param taps        濾波器階數 (Taps 數量)
 * @param history     歷史訊號環狀緩衝區
 * @param hist_size   環狀緩衝區總長度 (歷史容量)
 * @param start_idx   最新取樣點在緩衝區中的起始索引
 * @return float      卷積輸出值
 */
static inline float fir_circular_conv(const float *h,
                                      int taps,
                                      const float *history,
                                      int hist_size,
                                      int start_idx)
{
    float sum = 0.0f;
    int k = start_idx;
    int j;

#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
    #pragma loop_count(min=64, max=2048, modulo=2)
#endif
    for (j = 0; j < taps; j++)
    {
        sum += h[j] * history[k];
        if (--k < 0) k = hist_size - 1;
    }
    return sum;
}

// 區塊級即時處理函式 (直接處理 SPORT DMA 雙聲道數據)
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count)
{
    int i, j, k;

    for (i = 0; i < count; i += 2)
    {
        // 1. 定點轉浮點 (左聲道: 參考麥 x; 右聲道: 誤差麥 d)
        float x_n = (float)pRxBuf[i]   * (1.0f / 2147483648.0f);
        float d_n = (float)pRxBuf[i+1] * (1.0f / 2147483648.0f);

        // 2. 【反饋中和 (Feedback Neutralization)】
        // 卷積估計揚聲器洩漏聲波 f_leak(n)，並從參考麥還原純淨噪聲源 u_hat(n)
        int k_y = (y_buf_idx > 0) ? (y_buf_idx - 1) : (ANC_Y_HIST_SIZE - 1);
        float f_leak  = fir_circular_conv(FEEDBACK_PATH, ANC_F_TAPS, y_history, ANC_Y_HIST_SIZE, k_y);
        float u_hat_n = x_n - f_leak;

        // 3. 更新參考訊號歷史緩衝區
        x_history[anc_buf_idx] = u_hat_n;

        // 4. 【抗噪波卷積】y(n) = W * X (使用中和後的純淨噪聲)
        float y_n = fir_circular_conv(w_coeffs, ANC_W_TAPS, x_history, ANC_X_HIST_SIZE, anc_buf_idx);

        // 5. 更新抗噪輸出歷史 (供下一個取樣點的反饋路徑卷積使用)
        y_history[y_buf_idx] = y_n;
        if (++y_buf_idx >= ANC_Y_HIST_SIZE) y_buf_idx = 0;

        // 6. 【次級路徑濾波】計算濾波參考訊號 x'(n) = Ŝ * X
        float x_prime_n = fir_circular_conv(SECOND_PATH, ANC_S_TAPS, x_history, ANC_X_HIST_SIZE, anc_buf_idx);
        x_filt_history[anc_buf_idx] = x_prime_n;

        // 7. 誤差訊號
        float e_n = d_n;

        // 8. 功率追蹤與歸一化步長 (NLMS)
        px_power = 0.999f * px_power + 0.001f * (x_prime_n * x_prime_n);
        float norm_mu = ANC_MU / (px_power * (float)ANC_W_TAPS + ANC_EPSILON);

        // 9. 權重梯度更新 (帶微弱漏洩)
        float grad_step = norm_mu * e_n;
        k = anc_buf_idx;
#if defined(__ADSPSHARC__) || defined(__ADSP21000__)
        #pragma loop_count(min=64, max=1024, modulo=2)
#endif
        for (j = 0; j < ANC_W_TAPS; j++)
        {
            w_coeffs[j] = (1.0f - ANC_LEAK) * w_coeffs[j] + grad_step * x_filt_history[k];
            if (--k < 0) k = ANC_X_HIST_SIZE - 1;
        }

        // 10. 循環指標自增
        if (++anc_buf_idx >= ANC_X_HIST_SIZE) anc_buf_idx = 0;

        // 11. 輸出保護與限幅 (Clamping to [-0.95, +0.95])
        if (y_n > 0.95f)  y_n = 0.95f;
        if (y_n < -0.95f) y_n = -0.95f;

        // 12. 寫入 DAC 輸出緩衝區
        pTxBuf[i]   = (int)(y_n * 2147483647.0f); // 左聲道: 抗噪抵消波輸出至揚聲器
        pTxBuf[i+1] = (int)(e_n * 2147483647.0f); // 右聲道: 殘差訊號供耳機/示波器監聽
    }
}