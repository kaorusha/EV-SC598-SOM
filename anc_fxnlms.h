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

// 發散保護: ANC 啟動後，限幅前 |y| 超過 ANC_SAFE_OUTPUT_LIMIT 的取樣比例 (指數平均，時間常數 ANC_TRIP_TAU_S)
// 超過 ANC_TRIP_CLIP_FRACTION，或 y 變成 NaN，就判定發散:
//   - 喇叭輸出在 ANC_TRIP_FADE_MS 內淡出到 0 (直接切 0 會爆音)，之後一直是 0，重開機才會再跑 ANC
//   - W 停止更新，保留判定當時的值 (可用除錯器讀 w_coeffs)
//   - main loop 印 [ANC TRIP]: 啟動後幾秒、截斷比例、限幅前 |y| 的峰值、y 的頻率 (過零率估計)。過零率的平均
//     在 ANC 啟動時從 0 起算，除以同樣平均的 1 補正；否則啟動後不久就判定會偏低 (模擬: 0.4 s 判定的
//     1130 Hz 會印成 904 Hz；最快的 0.17 s 判定只剩一半)
//   - log 等淡出完、輸出再維持 0 ANC_TRIP_QUIET_MS 之後才印，這段期間狀態列也不印: JTAG printf 會讓核心暫停，
//     DMA 重播最後幾個緩衝，輸出卡在當時的值。2026-11-17 自我測試 (J18 R 錄音) 在判定當下就印，輸出卡在
//     限幅值 0.5 s，淡出延到暫停之後
// 起因: 2026-11-16 開機測試後 CCES_runner 結束，核心繼續跑，ANC 在沒人看著、風扇關的情況下發散，
// 喇叭持續發出中音並破音，直到板子斷電
// 門檻 (2026-11-16 模擬): 400-1500 Hz 正常約需 y 0.24，不會截斷；400-5000 Hz 在 100% duty 的兩音 0.24 + 0.92
// 截斷比例 0.20，3.6 kHz 加到 1.10 也只有 0.34。發散的單音撞到限幅後 0.3-0.8 s 判定
#define ANC_TRIP_CLIP_FRACTION  0.5f
#define ANC_TRIP_TAU_S          0.25f
#define ANC_TRIP_FADE_MS        5u
#define ANC_TRIP_QUIET_MS       10u      // 升頻濾波器拖尾約 1 ms、DMA 緩衝 4 個取樣，這個值留足餘裕

// 喇叭輸出開關 (ANC_Set_Output；ANC_AUTO_OFF_S 也經過它):
//   - 關: y 在 ANC_SWITCH_FADE_MS 內線性淡出到 0，之後不再算 W 的卷積
//   - 開: 用停住的 W 淡入，已收斂的音調馬上消掉，不必重新收斂
//   - 只有完全打開時 W 才更新、發散保護才累計。關閉與淡入淡出期間 W 停住: 這時誤差麥聽不到 (或聽不完整) 輸出，
//     繼續更新 W 只會一直變大。拔喇叭功放的電源當開關就是這種情形，接回時會以限幅大小輸出
//   - 回授歷史記錄實際送出的 y (乘過開關增益)，F̂ 的回授中和在開關前後都正確
#define ANC_SWITCH_FADE_MS  50u

// 定時關閉 (診斷用): ANC 啟動後這麼多秒把輸出關掉 (淡出、W 停住)。main loop 等輸出歸零後印 [ANC OFF]；ANC_DIAG 開著
// 時接著印狀態歷史，並寫出關閉前最後 ANC_DIAG_SNAP 個取樣，不必等發散判定就看得到 y。之後輸出一直是關的，
// CCES_runner 結束後核心照跑也不會無人看管地繼續 ANC。0 = 不關
#define ANC_AUTO_OFF_S      0u

typedef enum
{
    ANC_STATE_WAIT = 0, // ANC_START_DELAY_S 的等待期間
    ANC_STATE_ON,       // 輸出打開 (或正在淡入)
    ANC_STATE_OFF,      // 輸出關閉 (或正在淡出)，W 停住
    ANC_STATE_TRIPPED   // 發散保護已判定
} ANC_State;

typedef enum
{
    ANC_TRIP_NONE = 0,
    ANC_TRIP_CLIP,      // 持續截斷
    ANC_TRIP_NAN        // y 變成 NaN (W 已經壞了)
} ANC_Trip_Reason;

// 判定發散當時的數值 (ANC_Get_Trip)
typedef struct
{
    ANC_Trip_Reason reason;
    float seconds;      // ANC 啟動後幾秒判定
    float clip_frac;    // 平滑後的截斷比例
    float y_peak;       // 限幅前 |y| 的峰值 (以 ANC_TRIP_TAU_S 衰減)
    float freq_hz;      // 限幅前 y 的過零率換算的頻率 (發散時 y 幾乎是單音)
} ANC_Trip;

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
    bool     tripped;   // 發散保護已判定，喇叭靜音 (ANC_TRIP_*)
    bool     running;   // false: 還在 ANC_START_DELAY_S 的等待期間 (y = 0)
    bool     output_on; // false: 輸出被開關關掉 (ANC_Set_Output / ANC_AUTO_OFF_S)，W 停住
    float    seconds;   // 啟動後的秒數 (等待中為負值: 距離啟動還有幾秒)
    uint32_t samples;   // 這段期間的核心取樣數
    uint32_t clipped;   // 其中 y 被限幅的取樣數
    float    ref_ms;    // 參考麥原始訊號的均方值 (回授中和、帶通之前)
    float    err_ms;    // 誤差麥的均方值
    float    y_peak;    // 限幅前的最大 |y|
    float    p_x;       // ANC_W_TAPS * x' 功率 = 正規化分母扣掉 ANC_EPSILON
    float    w_energy;  // sum w^2 (持續變大 = 發散)
} ANC_Status;

// 診斷紀錄 (ANC_DIAG): ANC 運轉時不能 printf (JTAG 暫停會讓輸出卡住)，所以先存在 L2，判定發散或定時關閉
// (ANC_AUTO_OFF_S)、輸出歸零之後才由 main loop 印出並經 JTAG 寫檔 (Audio_Talkthrough_I2S.c 的 Diag_Report)。
// 兩者都沒發生就不輸出
//   - 每 ANC_DIAG_TICK_MS 一筆: 限幅前 |y| 的峰值、被限幅的比例、x / F̂y / u / y / e 的均方值、W2，保留最近 ANC_DIAG_HIST 筆
//   - 最近 ANC_DIAG_SNAP 個核心取樣的 x (參考麥)、F̂y (回授估計)、u (帶通後、送進 W 的參考)、限幅前的 y、e (誤差麥)，
//     判定或關閉輸出時凍結 (輸出關著時不記錄)
// 起因: 2026-11-17 實驗室 400-5000 Hz 實測 17.5 s 後判定 (一半取樣被限幅，約 3573 Hz)，誤差麥上卻看不到任何頻率
// 變大。要看到 y 本身，才分得出是輸出不夠 (W 越加越大) 還是 y 裡有誤差麥聽不到的成分
#define ANC_DIAG             1
#define ANC_DIAG_TICK_MS     250u
#define ANC_DIAG_HIST        128u     // 32 s
#define ANC_DIAG_SNAP        8192u    // 171 ms
#define ANC_DIAG_SIGNALS     5u       // x, F̂y, u, 限幅前的 y, e
#define ANC_DIAG_FILE        "measurements/anc_diag/anc_diag_snapshot.txt"   // repo 相對路徑 (Host_Path)，資料夾要先建好

typedef struct
{
    float t;        // 這一筆結束時，ANC 啟動後的秒數
    float y_pk;     // 限幅前 |y| 的最大值
    float clip;     // 被限幅的取樣比例
    float x_ms;     // 參考麥 (回授中和前) 的均方值
    float fy_ms;    // 回授估計 F̂y 的均方值
    float u_ms;     // 帶通後、送進 W 的參考的均方值
    float y_ms;     // 限幅後 y 的均方值
    float e_ms;     // 誤差麥的均方值
    float w2;       // 這一筆結束時的 sum w^2
} ANC_Diag_Rec;

// 對外 API 函式宣告
void ANC_Init(void);
void ANC_Process_Block(const int *pRxBuf, int *pTxBuf, int count);  // count 須為 4 * ANC_DECIM 的倍數
bool ANC_Is_Running(void);              // ANC_START_DELAY_S 過了沒
void ANC_Get_Status(ANC_Status *st);    // 讀出並清除這段期間的統計 (main loop 呼叫，非中斷)
bool ANC_Get_Trip(ANC_Trip *t);         // 已判定發散且輸出已靜音時回傳 true，並填入判定當時的數值 (main loop 呼叫)
bool ANC_Trip_Pending(void);            // 已判定、輸出還沒靜音 (淡出中): 這時不要 printf
void ANC_Set_Output(bool on);           // 打開 / 關閉喇叭輸出 (main loop 呼叫；淡入淡出在中斷裡做)
bool ANC_Output_Requested(void);        // 目前要求的開關狀態 (ANC_AUTO_OFF_S 關掉後是 false)
ANC_State ANC_Get_State(void);
bool ANC_Output_Quiet(void);            // 輸出已關、淡出完並維持 0 ANC_TRIP_QUIET_MS: 這時 printf 不會讓輸出卡住
bool ANC_Get_Auto_Off(float *seconds);  // ANC_AUTO_OFF_S 已關掉輸出且已歸零時回傳 true，並填入關閉時 ANC 啟動後的秒數
#if ANC_DIAG
uint32_t ANC_Diag_History(const ANC_Diag_Rec **ring);             // 已記錄的筆數；第 i 筆在 ring[i % ANC_DIAG_HIST]
uint32_t ANC_Diag_Snapshot(const float **sig, uint32_t *oldest);  // 快照的有效取樣數；訊號 s 的第 j 個在
                                                                  // sig[s * ANC_DIAG_SNAP + j]，最舊的在 j = oldest
#endif

#endif /* _ANC_FXNLMS_H_ */
