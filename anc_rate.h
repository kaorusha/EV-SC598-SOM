#ifndef _ANC_RATE_H_
#define _ANC_RATE_H_

// 192 kHz codec <-> 48 kHz ANC 核心的降頻 / 升頻濾波器
//
// codec 維持 192 kHz: ADAU1979 ADC 群延遲 35 us、ADAU1962A Low Prop. DAC 10 us (48 kHz 模式分別為 479 / 521 us)。
// ANC 核心改在 48 kHz 運算: 同樣 42.7 ms 的 W / S-hat / F-hat 只需 2048 階，運算量為 192 kHz 核心的 1/16，
// 而 S 約 20 ms、F 的箱體共振拖尾超過 33 ms，192 kHz 下 2048 階 (10.7 ms) 涵蓋不到。
//
// 濾波器: 6 階 Chebyshev II 低通，scipy.signal.cheby2(6, 60, 24000, fs=192000, output='sos')
//   - 通帶: 6 kHz 以下 < 0.01 dB，10 kHz -0.11 dB
//   - 阻帶: 24 kHz 起 >= 60 dB (42-54 kHz 會折疊到 0-6 kHz，同樣 >= 60 dB)
//   - 群延遲: 100 Hz-6 kHz 約 40-45 us (降頻、升頻各一次)
// 降頻與升頻共用這組係數。兩者的響應都成為 S-hat / F-hat 的一部分，APP_MODE_PATH_MEASURE 經同一個邊界量測。
// tools/export_paths.py 由本檔讀取係數，修改時兩邊自動一致。

#define ANC_DECIM           4           // 192 kHz codec 取樣 / 48 kHz 核心取樣
#define ANC_RATE_SECTIONS   3

typedef struct { float b0, b1, b2, a1, a2; } ANC_Biquad;                       // a0 = 1
typedef struct { float s1[ANC_RATE_SECTIONS], s2[ANC_RATE_SECTIONS]; } ANC_Rate_State;

static const ANC_Biquad ANC_RATE_SOS[ANC_RATE_SECTIONS] = {
    {+2.367812636e-03f, +2.076113943e-03f, +2.367812636e-03f, -1.216921088e+00f, +3.797170512e-01f},
    {+1.000000000e+00f, -9.780833528e-01f, +1.000000000e+00f, -1.394991592e+00f, +5.546560150e-01f},
    {+1.000000000e+00f, -1.378688700e+00f, +1.000000000e+00f, -1.663733515e+00f, +8.301249437e-01f},
};

// 一個 192 kHz 取樣通過三節 biquad (Transposed Direct Form II，極點半徑最大 0.91，float 精度足夠)
static inline float anc_rate_filter(ANC_Rate_State *st, float x)
{
    int k;
    for (k = 0; k < ANC_RATE_SECTIONS; k++)
    {
        const ANC_Biquad *c = &ANC_RATE_SOS[k];
        float y = c->b0 * x + st->s1[k];
        st->s1[k] = c->b1 * x - c->a1 * y + st->s2[k];
        st->s2[k] = c->b2 * x - c->a2 * y;
        x = y;
    }
    return x;
}

static inline void anc_rate_reset(ANC_Rate_State *st)
{
    int k;
    for (k = 0; k < ANC_RATE_SECTIONS; k++)
    {
        st->s1[k] = 0.0f;
        st->s2[k] = 0.0f;
    }
}

#endif /* _ANC_RATE_H_ */
