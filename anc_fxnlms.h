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

#define ANC_MU              0.007f   // 48 kHz 核心的收斂步長 (192 kHz 核心時為 0.00175)
#define ANC_LEAK            0.00001f // 每個核心取樣的漏洩係數 (防止阻帶發散，時間常數 2.1 s)
#define ANC_LEAK_INTERVAL   16       // 每 16 個核心取樣套用一次累積漏洩 (逐取樣套用每 tap 多 1 cycle)
#define ANC_EPSILON         0.005f   // 功率門檻底限 (防止靜音時除以接近0導致步長暴增)
#define ANC_MAX_MU          0.005f   // 48 kHz 核心的歸一化步長上限保護 (192 kHz 核心時為 0.00125)
#define ANC_PWR_SMOOTH_ALPHA 0.999f  // 功率估計 IIR 平滑常數 (歷史權重，時間常數 1000 取樣 = 20.8 ms)
#define ANC_PWR_SMOOTH_BETA  0.001f  // 功率估計 IIR 平滑常數 (新值權重，與 ALPHA 相加為 1)
#define ANC_SAFE_OUTPUT_LIMIT 0.20f  // 初測安全限幅 (約 -14 dBFS，防破音與大音量爆音)

// TDM4 雙端子獨立輸出 (Slot 0,1 -> J18 喇叭端子; Slot 2,3 -> J4/J17 電腦/NI-9234 監聽端子, J4 = J17 經耳機放大):
//     - J18    (絲印 DAC 3/4, SPORT Slot 0/1): 左喇叭靜音(0), 右喇叭輸出反相抗噪波 y(n)
//     - J4/J17 (絲印 DAC 1/2, SPORT Slot 2/3): Tip(左) = Error Mic 殘留誤差, Ring(右) = Ref Mic
//     (DAC_CTRL1 預設 LRCLK 50% duty + 正常極性，DAC frame 起點與 SPORT 差半個 TDM4 frame，
//      故 DAC1/2 收到 Slot 2/3；以 APP_MODE_DIAG_DAC_ID 實測，刻意維持不改 register)
// (麥克風直通驗證請改用 Audio_Talkthrough_I2S.c 的 APP_MODE_DIAG_MIC)

// 對外 API 函式宣告
void ANC_Init(void);
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count);  // count 須為 4 * ANC_DECIM 的倍數

#endif /* _ANC_FXNLMS_H_ */
