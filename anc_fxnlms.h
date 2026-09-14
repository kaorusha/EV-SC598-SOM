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
#define ANC_EPSILON         1e-6f   // 避免除以零

// 對外 API 函式宣告
void ANC_Init(void);
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count);

#endif /* _ANC_FXNLMS_H_ */
