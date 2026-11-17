#ifndef _ANC_REF_BAND_H_
#define _ANC_REF_BAND_H_

// 迴路內參考帶通 (48 kHz 核心)
//
// 回授中和後的參考訊號先過這個濾波器，再送進 W 與 Ŝ (x')；誤差麥的 e 不濾波。
// 用途: 只讓要消的音調進 x'。不濾波時 100% duty 的 x' 有 93% 是 88-354 Hz 的寬頻 (Ŝ 在 181 Hz 最大)，
// 總功率正規化下 602 Hz 只分到 2.4%、3615 Hz 0.26%，高頻音調幾乎不收斂；低頻又落在 Ŝ 的共振附近，
// 步長必須降到 0.002 才不發散。
//
// 濾波器: 4 階 Butterworth 帶通 (8 階)，與 FxNLMS.py 的模擬相同:
//   scipy.signal.butter(4, (f1, f2), 'bandpass', fs=48000, output='sos')
// 頻帶由 anc_fxnlms.h 的 ANC_REF_BAND 選擇:
//   - 400-1500 Hz: 只消 100% duty 的 600 Hz。限幅不夠 3.6 kHz (y 約 0.7) 時用這個，否則 3.6 kHz 留在 x' 只會讓輸出
//     一直被截斷 (2026-11-09 模擬，限幅 0.2: 602 Hz 降 20.8 dB；400-5000 Hz 只降 11.1 dB，98% 的取樣被截斷)。
//     第一次 ANC 實測先用這個
//   - 400-5000 Hz: 30-100% duty 的轉速音與 BPF。30%、50% duty 需要的 y 都在 0.08 以下；100% duty 的 3.6 kHz 要
//     y 約 0.7，限幅 0.9 (2026-11-16) 夠用 (模擬，限幅 1.0: 602 Hz 降 20.5 dB、3615 Hz 降 6.4 dB)
// float32 Transposed DF-II: 極點半徑最大 0.988，與 float64 的差 < -98 dB (相對輸出)。
//
// 群延遲 (400-1500): 602 Hz 1.09 ms、1131 Hz 0.65 ms；(400-5000): 602 Hz 0.72 ms、3615 Hz 0.14 ms。
// 這段延遲在 W 前面，週期性的音調 W 會補回來；寬頻噪音則因此失去因果性 (因果裕度只有約 0.19 ms)。

#include "anc_rate.h"   // ANC_Biquad

#if !defined(ANC_REF_BAND)
#error "include anc_fxnlms.h (ANC_REF_BAND) before anc_ref_band.h"
#endif

#define ANC_REF_BAND_SECTIONS   4

typedef struct { float s1[ANC_REF_BAND_SECTIONS], s2[ANC_REF_BAND_SECTIONS]; } ANC_Ref_Band_State;

#if (ANC_REF_BAND == ANC_REF_BAND_400_1500)
static const ANC_Biquad ANC_REF_BAND_SOS[ANC_REF_BAND_SECTIONS] = {   // butter(4, (400, 1500), 'bandpass', fs=48000)
    {+2.240902531e-05f, +4.481805062e-05f, +2.240902531e-05f, -1.819097609e+00f, +8.377510825e-01f},
    {+1.000000000e+00f, +2.000000000e+00f, +1.000000000e+00f, -1.909242087e+00f, +9.142279158e-01f},
    {+1.000000000e+00f, -2.000000000e+00f, +1.000000000e+00f, -1.883269869e+00f, +9.179826723e-01f},
    {+1.000000000e+00f, -2.000000000e+00f, +1.000000000e+00f, -1.973059504e+00f, +9.759359488e-01f},
};
#elif (ANC_REF_BAND == ANC_REF_BAND_400_5000)
static const ANC_Biquad ANC_REF_BAND_SOS[ANC_REF_BAND_SECTIONS] = {   // butter(4, (400, 5000), 'bandpass', fs=48000)
    {+4.172117804e-03f, +8.344235608e-03f, +4.172117804e-03f, -1.133185774e+00f, +3.484957801e-01f},
    {+1.000000000e+00f, +2.000000000e+00f, +1.000000000e+00f, -1.333139185e+00f, +6.669486471e-01f},
    {+1.000000000e+00f, -2.000000000e+00f, +1.000000000e+00f, -1.894538784e+00f, +8.979926802e-01f},
    {+1.000000000e+00f, -2.000000000e+00f, +1.000000000e+00f, -1.962378592e+00f, +9.651676329e-01f},
};
#endif

#if (ANC_REF_BAND != ANC_REF_BAND_NONE)
// 一個 48 kHz 核心取樣通過四節 biquad (Transposed Direct Form II，與 anc_rate_filter 相同寫法)
static inline float anc_ref_band_filter(ANC_Ref_Band_State *st, float x)
{
    int k;
    for (k = 0; k < ANC_REF_BAND_SECTIONS; k++)
    {
        const ANC_Biquad *c = &ANC_REF_BAND_SOS[k];
        float y = c->b0 * x + st->s1[k];
        st->s1[k] = c->b1 * x - c->a1 * y + st->s2[k];
        st->s2[k] = c->b2 * x - c->a2 * y;
        x = y;
    }
    return x;
}
#endif

static inline void anc_ref_band_reset(ANC_Ref_Band_State *st)
{
    int k;
    for (k = 0; k < ANC_REF_BAND_SECTIONS; k++)
    {
        st->s1[k] = 0.0f;
        st->s2[k] = 0.0f;
    }
}

#endif /* _ANC_REF_BAND_H_ */
