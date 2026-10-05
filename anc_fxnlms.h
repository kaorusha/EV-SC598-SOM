#ifndef _ANC_FXNLMS_H_
#define _ANC_FXNLMS_H_

#include <stdint.h>
#include <stdbool.h>
#include "second_path.h"
#include "feedback_path.h"

// 演算法配置常數
#define ANC_W_TAPS          1024    // 主自適應濾波器階數 W(z)

// 次級路徑與反饋路徑長度直接對齊標頭檔定義 (單一真實來源，不重複定義)
#define ANC_S_TAPS          (SECOND_PATH_TAPS)
#define ANC_F_TAPS          (FEEDBACK_PATH_TAPS)

// 歷史緩衝區容量自動推導 (編譯期自動取 max，對齊 Python 邏輯)
#define ANC_X_HIST_SIZE     ((ANC_W_TAPS > SECOND_PATH_TAPS) ? (ANC_W_TAPS) : (SECOND_PATH_TAPS))
#define ANC_Y_HIST_SIZE     (FEEDBACK_PATH_TAPS)

#define ANC_MU              0.007f  // 收斂步長
#define ANC_LEAK            0.00001f// 漏洩係數 (防止阻帶發散)
#define ANC_EPSILON         0.005f  // 功率門檻底限 (防止靜音時除以接近0導致步長暴增)
#define ANC_MAX_MU          0.005f  // 歸一化步長上限保護 (防發散震盪)
#define ANC_SAFE_OUTPUT_LIMIT 0.20f // 初測安全限幅 (約 -14 dBFS，防破音與大音量爆音)

// TDM4 雙端子獨立輸出模式設定 (Slot 0,1 -> J4/J17 喇叭端子; Slot 2,3 -> J18 電腦/NI-9234 監聽端子):
// 0 = 雙孔直通驗證模式 (當前設定，用於先確認雙孔獨立輸出與監聽正常):
//     - J4/J17 (DAC 1/2): 左喇叭靜音(0), 右喇叭直通播放 Ref Mic (觸碰 Ref Mic 右喇叭發聲)
//     - J18    (DAC 3/4): Tip(左) = Error Mic, Ring(右) = Ref Mic (給電腦音效卡 / NI-9234 監聽)
// 1 = 正式 ANC 降噪 + 雙孔獨立監聽模式:
//     - J4/J17 (DAC 1/2): 左喇叭自動靜音(0), 右喇叭單獨輸出反相抗噪波 y(n)
//     - J18    (DAC 3/4): Tip(左) = Error Mic 殘留誤差, Ring(右) = Ref Mic
#define ANC_OUTPUT_MODE     0

// 對外 API 函式宣告
void ANC_Init(void);
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count);

#endif /* _ANC_FXNLMS_H_ */
