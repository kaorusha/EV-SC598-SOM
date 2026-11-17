#ifndef _ANC_FXNLMS_H_
#define _ANC_FXNLMS_H_

#include <stdint.h>
#include <stdbool.h>
#include "second_path.h"
#include "feedback_path.h"
#include "anc_rate.h"

// 取樣率: codec 192 kHz，ANC 核心 48 kHz (192 kHz 每 ANC_DECIM 個 frame 算一次，降頻 / 升頻濾波見 anc_rate.h)
#define ANC_CORE_FS         48000u

// 演算法配置常數 (以下取樣數、步長、時間常數皆以 48 kHz 核心取樣為單位)
#define ANC_W_TAPS          2048    // 主自適應濾波器階數 W(z)，42.7 ms (192 kHz 核心時 1024 階僅 5.3 ms)

// 次級路徑與反饋路徑長度直接對齊標頭檔定義 (單一真實來源，不重複定義)
#define ANC_S_TAPS          (SECOND_PATH_TAPS)
#define ANC_F_TAPS          (FEEDBACK_PATH_TAPS)

// 歷史緩衝區容量自動推導 (編譯期自動取 max，對齊 Python 邏輯)
#define ANC_X_HIST_SIZE     ((ANC_W_TAPS > SECOND_PATH_TAPS) ? (ANC_W_TAPS) : (SECOND_PATH_TAPS))
#define ANC_Y_HIST_SIZE     (FEEDBACK_PATH_TAPS)

// 迴路內參考帶通 (anc_ref_band.h): 回授中和後、送進 W 與 Ŝ 之前；誤差麥不濾波
#define ANC_REF_BAND_NONE       0   // 不濾波
#define ANC_REF_BAND_400_1500   1   // 100% duty 只消 600 Hz。第一次 ANC 實測先用這個
#define ANC_REF_BAND_400_5000   2   // 30-100% duty 的轉速音與 BPF (100% duty 的 BPF 要 y 約 0.7，限幅 0.9 夠用)
#define ANC_REF_BAND            ANC_REF_BAND_400_1500

// 正規化步長 mu: w -= mu * e * x' / (ANC_W_TAPS * x' 功率 + ANC_EPSILON)，不另設上限
// (舊的步長上限 0.005 加上 ANC_EPSILON 0.005，讓步長固定在 0.005、W 幾乎不動，已移除)
#if (ANC_REF_BAND == ANC_REF_BAND_NONE)
#define ANC_REF_BAND_NAME   "none"
#define ANC_MU              0.002f   // 不濾波: x' 幾乎都是 Ŝ 共振附近的低頻，300 Hz 單音穩定上限 0.0064；FxNLMS.py 模擬 0.01 會在低頻發散
#elif (ANC_REF_BAND == ANC_REF_BAND_400_1500)
#define ANC_REF_BAND_NAME   "400-1500 Hz"
#define ANC_MU              0.01f    // 2026-11-09 模擬 100% duty、限幅 0.2: 穩定，602 Hz 降 20.8 dB
#elif (ANC_REF_BAND == ANC_REF_BAND_400_5000)
#define ANC_REF_BAND_NAME   "400-5000 Hz"
#define ANC_MU              0.01f    // 各音調的穩定上限最小約 0.033 (30% duty 的 1132 Hz)；FxNLMS.py 模擬 30-100% duty 都穩定
#else
#error "unknown ANC_REF_BAND"
#endif
#define ANC_LEAK            0.00001f // 每個核心取樣的漏洩係數 (防止阻帶發散，時間常數 2.1 s)
#define ANC_LEAK_INTERVAL   16       // 每 16 個核心取樣套用一次累積漏洩 (逐取樣套用每 tap 多 1 cycle)
#define ANC_EPSILON         1e-11f   // 正規化分母底限，板子單位 (ADC 滿刻度 = 1)。暫定值，由 8/4 錄音換算:
                                     // 帶通後的 ANC_W_TAPS * x' 功率在 20% duty 約 8e-12、100% 約 1.4e-8。
                                     // 該設為風扇關閉時的量級: 看 ANC 狀態列印的 Px' (Audio_Talkthrough_I2S.c)
#define ANC_PWR_SMOOTH_ALPHA 0.999f  // 功率估計 IIR 平滑常數 (歷史權重，時間常數 1000 取樣 = 20.8 ms)
#define ANC_PWR_SMOOTH_BETA  0.001f  // 功率估計 IIR 平滑常數 (新值權重，與 ALPHA 相加為 1)
#define ANC_SAFE_OUTPUT_LIMIT 0.90f  // 輸出限幅 (約 -0.9 dBFS)。2026-11-16 喇叭測試 (APP_MODE_SPEAKER_TEST): 3614 Hz 0.72 加
                                     // 602 Hz 0.20 (峰值 0.92) 連放 5 分鐘、單音到 1.0 都沒有失真或壓縮。100% duty 約需
                                     // 0.24 (600 Hz) + 0.65-0.92 (3.6 kHz，隨出風口前方的環境變)；30%、50% duty 都在 0.08 以下。
                                     // 測過的只是兩個純音 (RMS 0.53)；發散時持續截斷會接近 RMS 0.9，要看狀態列的 clip
#define ANC_START_DELAY_S   15u      // 開機後 ANC 先關閉這麼多秒 (y = 0、W 不更新，功率估計照常)，
                                     // 同一段錄音前段是關、後段是開；也留時間離開出風口前方

// TDM4 雙端子獨立輸出 (Slot 0,1 -> J18 喇叭端子; Slot 2,3 -> J4/J17 電腦/NI-9234 監聽端子, J4 = J17 經耳機放大):
//     - J18    (絲印 DAC 3/4, SPORT Slot 0/1): 左喇叭靜音(0), 右喇叭輸出抗噪波 y(n)
//       (y 送出時不另外反相: Ŝ 量測時 y 的極性與此相同，W 以梯度下降自己學成與噪音反相)
//     - J4/J17 (絲印 DAC 1/2, SPORT Slot 2/3): Tip(左) = Error Mic 殘留誤差, Ring(右) = Ref Mic
//     (DAC_CTRL1 預設 LRCLK 50% duty + 正常極性，DAC frame 起點與 SPORT 差半個 TDM4 frame，
//      故 DAC1/2 收到 Slot 2/3；以 APP_MODE_DIAG_DAC_ID 實測，刻意維持不改 register)
// (麥克風直通驗證請改用 Audio_Talkthrough_I2S.c 的 APP_MODE_DIAG_MIC)

// ANC 狀態，main loop 每 ~2 s 讀一次並印出 (Audio_Talkthrough_I2S.c 的 ENABLE_ANC_STATUS)
typedef struct
{
    bool     running;   // false: 還在 ANC_START_DELAY_S 的等待期間 (y = 0)
    float    seconds;   // 啟動後的秒數 (等待中為負值: 距離啟動還有幾秒)
    uint32_t samples;   // 這段期間的核心取樣數
    uint32_t clipped;   // 其中 y 被限幅的取樣數
    float    ref_ms;    // 參考麥原始訊號的均方值 (回授中和、帶通之前)
    float    err_ms;    // 誤差麥的均方值
    float    y_peak;    // 限幅前的最大 |y|
    float    p_x;       // ANC_W_TAPS * x' 功率 = 正規化分母扣掉 ANC_EPSILON
    float    w_energy;  // sum w^2 (持續變大 = 發散)
} ANC_Status;

// 對外 API 函式宣告
void ANC_Init(void);
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count);  // count 須為 4 * ANC_DECIM 的倍數
bool ANC_Is_Running(void);              // ANC_START_DELAY_S 過了沒
void ANC_Get_Status(ANC_Status *st);    // 讀出並清除這段期間的統計 (main loop 呼叫，非中斷)

#endif /* _ANC_FXNLMS_H_ */
