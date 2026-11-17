#ifndef _SPEAKER_TEST_H_
#define _SPEAKER_TEST_H_

/*
 * Speaker linearity test (APP_MODE_SPEAKER_TEST). Header-only: Audio_Talkthrough_I2S.c includes it in that mode only,
 * after its slot map.
 *
 * Purpose: find the largest y the speaker reproduces cleanly, to set ANC_SAFE_OUTPUT_LIMIT (anc_fxnlms.h). It was 0.2,
 * a first-run guard against divergence, until the 2026-11-16 run of SPK_PLAN_HOLD set it to 0.9. Cancelling the 3.6 kHz
 * BPF at 100% duty needs y of about 0.72 and 602 Hz about 0.2 (8/4 recordings), about 0.92 peak together.
 *
 * The core plays sines as y in ANC units: the same 48 kHz core rate, the same x4 interpolation (anc_rate.h) and J18 Right
 * output as ANC_Process_Block, so y = 1.0 is DAC full scale. The Err mic is decimated as in ANC and measured at the core
 * rate. SPK_PLAN picks the steps; each numbered item is a ladder:
 *   SPK_PLAN_LADDERS: the levels
 *     1. SPK_BPF_HZ alone, y = SPK_LEVELS (0.1 ... 1.0)
 *     2. SPK_F0_HZ alone, the same levels
 *     3. both at the 100% duty need (SPK_BPF_NEED + SPK_F0_NEED), then the same held for SPK_HOLD_SEGMENTS x SPK_HOLD_S
 *        (heating: the gain should not drift)
 *   SPK_PLAN_HOLD: the need level only, with single-tone references before and after the hold
 *     1. SPK_BPF_HZ alone at SPK_BPF_NEED      2. SPK_F0_HZ alone at SPK_F0_NEED
 *     3. both together, then held as above
 *     4. SPK_BPF_HZ alone again                5. SPK_F0_HZ alone again
 * Each step fades in over SPK_RAMP_S, settles for SPK_SETTLE_S, is measured for SPK_MEASURE_S (a hold segment for
 * SPK_HOLD_S), fades out and stays silent at least SPK_GAP_S while main() prints the result. A JTAG printf halts the
 * core; with the output silent it costs no click, so nothing prints while a tone plays.
 *
 * Printed per tone (synchronous detection with the generator's own phase, so the reading is exact at that frequency):
 *   err    Err amplitude at f, dBFS (peak)
 *   gain   err / y in dB, i.e. |S| at this level; d = change from the first step at this frequency (compression < 0)
 *   ph     change of the Err phase from the first step at this frequency, degrees. The latency is fixed within a boot,
 *          so ph moves only with the acoustic path. A reflection that changes the gain (people in front of the outlet)
 *          usually moves ph too; speaker compression mostly changes the gain alone
 *   H2 H3  Err at 2f / 3f relative to err, dBc
 * Printed per step: rest = Err power left after removing the tones (noise, harmonics, rattle) in dB re the tones, and
 * the raw Err ADC peak. While the mic noise dominates, rest falls 6 dB per doubling of y; a rise at high y means
 * distortion or a rattle.
 * Two tones together also show intermodulation: a gain drop of one tone that the single tones did not show.
 * A step fails when H2 or H3 > SPK_MAX_HARMONIC_DBC, the Err ADC peak exceeds SPK_MAX_ERR_PEAK_DBFS (then the ADC, not
 * the speaker, limits the reading), or the gain drops: d < -SPK_MAX_COMPRESSION_DB for a single tone, or
 * d < -SPK_MAX_TWO_TONE_DROP_DB for two tones (a smaller two-tone drop is only noted, so the hold goes on). A failure
 * ends its ladder and the next ladder starts from its own first step. Listen too: a rattle or buzz the numbers miss
 * also counts as a failure.
 *
 * Run with the fan off, nobody in front of the outlet and the speaker knob untouched. J4/J17 carry the raw Err (Left)
 * and Ref (Right) mics as in ANC mode, for listening or recording.
 * To abort, unplug J18 or switch the speaker off. Do not pause the core while a tone plays: the TX DMA keeps repeating
 * its last buffers, which plays as a steady buzz.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "anc_rate.h"

#ifndef J12_ERR_MIC_SLOT
#error "include speaker_test.h after the slot map (J12_ERR_MIC_SLOT, J12_REF_MIC_SLOT)"
#endif

/* Tones: the 100% duty targets */
#define SPK_BPF_HZ              3614.0f   /* blade-pass frequency, 6 x f0 */
#define SPK_F0_HZ               602.0f    /* rotation frequency */
#define SPK_BPF_NEED            0.72f     /* y that would cancel each tone at the Err mic at 100% duty: 8/4 Err tone */
#define SPK_F0_NEED             0.20f     /* in board units / |S|. Recheck with the ANC-off part of the first ANC run */

/* Timing */
#define SPK_CORE_FS             48000.0f
#define SPK_START_S             3.0f      /* silence after boot */
#define SPK_RAMP_S              0.05f     /* raised-cosine fade in / out */
#define SPK_SETTLE_S            0.5f      /* not measured: speaker, room reverb and rate filters settle */
#define SPK_MEASURE_S           5.0f      /* measured part of a ladder step */
#define SPK_HOLD_S              30.0f     /* measured part of a hold segment */
#define SPK_HOLD_SEGMENTS       10        /* 10 x 30 s = 5 min at the need level */
#define SPK_GAP_S               1.0f      /* minimum silence between steps; main() prints in it */

/* Pass limits */
#define SPK_MAX_COMPRESSION_DB  1.0f      /* single tone: gain drop from the first step at that frequency */
#define SPK_MAX_TWO_TONE_DROP_DB 3.0f     /* two tones: a smaller drop is only noted */
#define SPK_MAX_HARMONIC_DBC    (-30.0f)  /* H2 and H3 */
#define SPK_MAX_ERR_PEAK_DBFS   (-1.0f)   /* raw Err ADC peak */

/* Plan */
#define SPK_PLAN_LADDERS        1         /* each tone at SPK_LEVELS, then the two tones and the hold */
#define SPK_PLAN_HOLD           2         /* the need level only: each tone alone, both + hold, each tone alone again */
#define SPK_PLAN                SPK_PLAN_HOLD

static const float SPK_LEVELS[] = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f };
#define SPK_NUM_LEVELS          ((int)(sizeof(SPK_LEVELS) / sizeof(SPK_LEVELS[0])))
#define SPK_MAX_STEPS           (2 * SPK_NUM_LEVELS + 5 + SPK_HOLD_SEGMENTS)   /* enough for either plan */

#define SPK_N(sec)              ((uint32_t)((sec) * SPK_CORE_FS + 0.5f))   /* seconds -> core samples */
#define SPK_PART                4800u     /* core samples per float partial sum, then added in long double */

#define SPK_NEXT                1         /* main() -> callback: play the next step */
#define SPK_SKIP_LADDER         2         /* main() -> callback: this step failed, go to the next ladder */

typedef struct
{
	float    f[2];        /* Hz */
	float    y[2];        /* amplitude in ANC y units; 0 = tone unused */
	uint32_t n_meas;      /* measured core samples */
	int      ladder;      /* 1, 2, 3 */
	int      hold;        /* hold segment 1 .. SPK_HOLD_SEGMENTS, 0 otherwise */
} Spk_Step;

typedef struct
{
	float amp[2][3];      /* Err amplitude (FS, peak) at f, 2f, 3f of each tone */
	float ph[2];          /* Err phase at f re the generator, radians */
	float ms;             /* Err mean square */
	float err_peak;       /* max |raw Err ADC sample|, FS */
	int   step;
} Spk_Result;

static Spk_Step g_SpkStep[SPK_MAX_STEPS];
static int      g_SpkNumSteps = 0;

/* SPORT callback state */
static ANC_Rate_State g_SpkDec, g_SpkInt;
static int         g_SpkIdx = -1;                  /* step playing; -1 = start-up silence */
static uint32_t    g_SpkN = 0u;                    /* core samples since the step (or start-up) began */
static float       g_SpkPhase[2], g_SpkInc[2];     /* generator phase and increment, in cycles */
static float       g_SpkPart[2][3][2], g_SpkPartMs;
static uint32_t    g_SpkPartN;
static long double g_SpkSum[2][3][2], g_SpkSumMs;
static uint32_t    g_SpkSumN;
static float       g_SpkPeak;

/* Handshake: the callback posts each finished step, main() prints it and answers with a command */
static volatile Spk_Result g_SpkRes;
static volatile int g_SpkPosted = 0;               /* 1: g_SpkRes holds a finished step */
static volatile int g_SpkCmd = 0;                  /* SPK_NEXT or SPK_SKIP_LADDER, 0 = not answered yet */
static volatile int g_SpkDone = 0;                 /* every step played; the output stays muted */

static void Spk_Add(float f0, float y0, float f1, float y1, float seconds, int ladder, int hold)
{
	Spk_Step *s;
	if ((g_SpkNumSteps >= SPK_MAX_STEPS) || (y0 + y1 > 1.0f))
	{
		printf("[SPK] step left out: y %.2f + %.2f is above full scale\n", y0, y1);
		return;
	}
	s = &g_SpkStep[g_SpkNumSteps++];
	s->f[0] = f0;
	s->y[0] = y0;
	s->f[1] = f1;
	s->y[1] = y1;
	s->n_meas = SPK_N(seconds);
	s->ladder = ladder;
	s->hold = hold;
}

static void Spk_Init(void)
{
	int i;
#if (SPK_PLAN == SPK_PLAN_LADDERS)
	for (i = 0; i < SPK_NUM_LEVELS; i++)
	{
		Spk_Add(SPK_BPF_HZ, SPK_LEVELS[i], 0.0f, 0.0f, SPK_MEASURE_S, 1, 0);
	}
	for (i = 0; i < SPK_NUM_LEVELS; i++)
	{
		Spk_Add(SPK_F0_HZ, SPK_LEVELS[i], 0.0f, 0.0f, SPK_MEASURE_S, 2, 0);
	}
#elif (SPK_PLAN == SPK_PLAN_HOLD)
	Spk_Add(SPK_BPF_HZ, SPK_BPF_NEED, 0.0f, 0.0f, SPK_MEASURE_S, 1, 0);
	Spk_Add(SPK_F0_HZ, SPK_F0_NEED, 0.0f, 0.0f, SPK_MEASURE_S, 2, 0);
#else
#error "Unknown SPK_PLAN"
#endif
	Spk_Add(SPK_BPF_HZ, SPK_BPF_NEED, SPK_F0_HZ, SPK_F0_NEED, SPK_MEASURE_S, 3, 0);
	for (i = 1; i <= SPK_HOLD_SEGMENTS; i++)
	{
		Spk_Add(SPK_BPF_HZ, SPK_BPF_NEED, SPK_F0_HZ, SPK_F0_NEED, SPK_HOLD_S, 3, i);
	}
#if (SPK_PLAN == SPK_PLAN_HOLD)
	Spk_Add(SPK_BPF_HZ, SPK_BPF_NEED, 0.0f, 0.0f, SPK_MEASURE_S, 4, 0);   /* again after the hold */
	Spk_Add(SPK_F0_HZ, SPK_F0_NEED, 0.0f, 0.0f, SPK_MEASURE_S, 5, 0);
#endif
	anc_rate_reset(&g_SpkDec);
	anc_rate_reset(&g_SpkInt);
}

static void Spk_Print_Plan(void)
{
	float s = SPK_START_S;
	int i;
	for (i = 0; i < g_SpkNumSteps; i++)
	{
		s += 2.0f * SPK_RAMP_S + SPK_SETTLE_S + (float)g_SpkStep[i].n_meas / SPK_CORE_FS + SPK_GAP_S;
	}
	printf(">>> MODE: Speaker test: sines as y (ANC units, ANC interpolation, J18 Right), Err mic measured\n");
	printf(">>> Fan OFF, nobody in front of the outlet, speaker knob untouched. Loud near the outlet: earplugs\n");
#if (SPK_PLAN == SPK_PLAN_LADDERS)
	printf(">>> Ladder 1: %.0f Hz, ladder 2: %.0f Hz, each y %.1f ... %.1f; ladder 3: %.0f Hz %.2f + %.0f Hz %.2f,"
		   " then held %d x %.0f s\n", SPK_BPF_HZ, SPK_F0_HZ, SPK_LEVELS[0], SPK_LEVELS[SPK_NUM_LEVELS - 1],
		   SPK_BPF_HZ, SPK_BPF_NEED, SPK_F0_HZ, SPK_F0_NEED, SPK_HOLD_SEGMENTS, SPK_HOLD_S);
#else
	printf(">>> %.0f Hz %.2f alone, %.0f Hz %.2f alone, both together, held %d x %.0f s, then each alone again\n",
		   SPK_BPF_HZ, SPK_BPF_NEED, SPK_F0_HZ, SPK_F0_NEED, SPK_HOLD_SEGMENTS, SPK_HOLD_S);
#endif
	printf(">>> A ladder stops at gain drop > %.1f dB (two tones: > %.1f dB), H2/H3 > %.0f dBc or Err ADC peak"
		   " > %.0f dBFS. %d steps, about %.0f min\n", SPK_MAX_COMPRESSION_DB, SPK_MAX_TWO_TONE_DROP_DB,
		   SPK_MAX_HARMONIC_DBC, SPK_MAX_ERR_PEAK_DBFS, g_SpkNumSteps, s / 60.0f);
	printf(">>> Abort: unplug J18 or switch the speaker off. Do not pause the core while a tone plays (the DMA\n");
	printf(">>>        repeats its last buffers: a steady buzz)\n");
	printf(">>> Output Jack J4/J17: Left = Err Mic, Right = Ref Mic (raw, as in ANC mode)\n");
}

static void Spk_Begin(int idx)
{
	int t;
	g_SpkIdx = idx;
	g_SpkN = 0u;
	for (t = 0; t < 2; t++)
	{
		g_SpkPhase[t] = 0.0f;
		g_SpkInc[t] = g_SpkStep[idx].f[t] / SPK_CORE_FS;
	}
	memset(g_SpkPart, 0, sizeof(g_SpkPart));
	memset(g_SpkSum, 0, sizeof(g_SpkSum));
	g_SpkPartMs = 0.0f;
	g_SpkPartN = 0u;
	g_SpkSumMs = 0.0L;
	g_SpkSumN = 0u;
	g_SpkPeak = 0.0f;
}

/* Fold the float partial sums into the long double totals (float alone loses ~1% over a 30 s segment) */
static void Spk_Flush(void)
{
	int t, h;
	for (t = 0; t < 2; t++)
	{
		for (h = 0; h < 3; h++)
		{
			g_SpkSum[t][h][0] += (long double)g_SpkPart[t][h][0];
			g_SpkSum[t][h][1] += (long double)g_SpkPart[t][h][1];
			g_SpkPart[t][h][0] = 0.0f;
			g_SpkPart[t][h][1] = 0.0f;
		}
	}
	g_SpkSumMs += (long double)g_SpkPartMs;
	g_SpkPartMs = 0.0f;
	g_SpkPartN = 0u;
}

static void Spk_Post(void)
{
	int t, h;
	float n;
	Spk_Flush();
	n = (float)g_SpkSumN;
	for (t = 0; t < 2; t++)
	{
		for (h = 0; h < 3; h++)
		{
			float c = (float)g_SpkSum[t][h][0], s = (float)g_SpkSum[t][h][1];
			g_SpkRes.amp[t][h] = (n > 0.0f) ? (2.0f * sqrtf(c * c + s * s) / n) : 0.0f;
		}
		/* Err = G sin(theta + phi) sums to (n/2) G (sin phi, cos phi) against (cos theta, sin theta) */
		g_SpkRes.ph[t] = atan2f((float)g_SpkSum[t][0][0], (float)g_SpkSum[t][0][1]);
	}
	g_SpkRes.ms = (n > 0.0f) ? ((float)g_SpkSumMs / n) : 0.0f;
	g_SpkRes.err_peak = g_SpkPeak;
	g_SpkRes.step = g_SpkIdx;
	g_SpkCmd = 0;
	g_SpkPosted = 1;
}

static inline int Spk_Q31(float v)
{
	if (v >  0.9999999f) v =  0.9999999f;
	if (v < -1.0f)       v = -1.0f;
	return (int)(v * 2147483647.0f);
}

/* SPORT callback: one DMA block (count ints, whole 48 kHz core samples) */
static void Spk_Process_Block(const int *pIn, int *pOut, int count)
{
	int i, k, t;
	for (i = 0; i < count; i += 4 * ANC_DECIM)
	{
		float d = 0.0f, pk = 0.0f, y = 0.0f;
		/* Err decimated exactly as in ANC_Process_Block (last frame of the group), raw peak kept */
		for (k = 0; k < ANC_DECIM; k++)
		{
			float v = (float)pIn[i + 4 * k + J12_ERR_MIC_SLOT] * (1.0f / 2147483648.0f);
			float a = (v < 0.0f) ? -v : v;
			if (a > pk) pk = a;
			d = anc_rate_filter(&g_SpkDec, v);
		}

		if (g_SpkIdx < 0)
		{
			if ((++g_SpkN >= SPK_N(SPK_START_S)) && (g_SpkNumSteps > 0))
			{
				Spk_Begin(0);
			}
		}
		else if (!g_SpkDone)
		{
			const Spk_Step *st = &g_SpkStep[g_SpkIdx];
			const uint32_t n = g_SpkN;
			const uint32_t b = SPK_N(SPK_RAMP_S) + SPK_N(SPK_SETTLE_S);   /* measuring starts */
			const uint32_t c = b + st->n_meas;                             /* fade-out starts */
			const uint32_t e = c + SPK_N(SPK_RAMP_S);                      /* silent from here */
			if (n < e)
			{
				const int meas = (n >= b) && (n < c);
				float env = 1.0f;
				if (n < SPK_N(SPK_RAMP_S))
				{
					env = 0.5f - 0.5f * cosf(3.14159265f * (float)n / (float)SPK_N(SPK_RAMP_S));
				}
				else if (n >= c)
				{
					env = 0.5f + 0.5f * cosf(3.14159265f * (float)(n - c) / (float)SPK_N(SPK_RAMP_S));
				}
				for (t = 0; t < 2; t++)
				{
					float th, cs, sn;
					if (st->y[t] <= 0.0f)
					{
						continue;
					}
					th = 6.28318531f * g_SpkPhase[t];
					cs = cosf(th);
					sn = sinf(th);
					y += env * st->y[t] * sn;
					if (meas)
					{
						float c2 = cs * cs - sn * sn, s2 = 2.0f * sn * cs;
						float c3 = cs * c2 - sn * s2, s3 = sn * c2 + cs * s2;
						g_SpkPart[t][0][0] += d * cs;
						g_SpkPart[t][0][1] += d * sn;
						g_SpkPart[t][1][0] += d * c2;
						g_SpkPart[t][1][1] += d * s2;
						g_SpkPart[t][2][0] += d * c3;
						g_SpkPart[t][2][1] += d * s3;
					}
					g_SpkPhase[t] += g_SpkInc[t];
					if (g_SpkPhase[t] >= 1.0f) g_SpkPhase[t] -= 1.0f;
				}
				if (meas)
				{
					g_SpkPartMs += d * d;
					if (pk > g_SpkPeak) g_SpkPeak = pk;
					g_SpkSumN++;
					if (++g_SpkPartN >= SPK_PART)
					{
						Spk_Flush();
					}
				}
				if (++g_SpkN == e)
				{
					Spk_Post();
				}
			}
			else if (n < e + SPK_N(SPK_GAP_S))
			{
				g_SpkN++;
			}
			else if (g_SpkCmd != 0)
			{
				int next = g_SpkIdx + 1;
				if (g_SpkCmd == SPK_SKIP_LADDER)
				{
					while ((next < g_SpkNumSteps) && (g_SpkStep[next].ladder == st->ladder)) next++;
				}
				g_SpkPosted = 0;
				g_SpkCmd = 0;
				if (next < g_SpkNumSteps)
				{
					Spk_Begin(next);
				}
				else
				{
					g_SpkDone = 1;
				}
			}
		}

		/* y through the ANC interpolator to J18 Right; J4/J17 = raw mics as in ANC mode */
		for (k = 0; k < ANC_DECIM; k++)
		{
			float y_hi = anc_rate_filter(&g_SpkInt, (k == 0) ? (float)ANC_DECIM * y : 0.0f);
			int *pt = &pOut[i + 4 * k];
			const int *pf = &pIn[i + 4 * k];
			pt[0] = 0;                      /* J18 Left  (Tip):  Mute */
			pt[1] = Spk_Q31(y_hi);          /* J18 Right (Ring): test tone */
			pt[2] = pf[J12_ERR_MIC_SLOT];   /* J17 Left  (Tip):  Err Mic */
			pt[3] = pf[J12_REF_MIC_SLOT];   /* J17 Right (Ring): Ref Mic */
		}
	}
}

/* main(): print each step as the callback posts it, decide pass / fail, and end with a summary. Returns when done. */
static void Spk_Run(void)
{
	static const char *const tone_name[2] = { "BPF", "f0" };
	float ref_gain[2] = { 0.0f, 0.0f };          /* gain and phase of the first step per frequency: [0] BPF, [1] f0 */
	float ref_ph[2] = { 0.0f, 0.0f };
	int   ref_set[2] = { 0, 0 };
	float clean_y[3] = { 0.0f, 0.0f, 0.0f };     /* highest passing y, ladders 1 and 2 */
	float fail_y[3] = { 0.0f, 0.0f, 0.0f };
	const char *fail_why[3] = { "", "", "" };
	int   need_ok = -1;                          /* ladder 3 first step: -1 not run, 0 failed, 1 passed */
	float need_d[2] = { 0.0f, 0.0f }, need_ph[2] = { 0.0f, 0.0f };
	int   hold_n = 0, hold_ok = 0;               /* hold segments played / passed */
	float hold_lo[2] = { 1e9f, 1e9f }, hold_hi[2] = { -1e9f, -1e9f };
	float hold_ph_lo[2] = { 1e9f, 1e9f }, hold_ph_hi[2] = { -1e9f, -1e9f };
	int   after_set[2] = { 0, 0 };               /* single tones after the hold (ladders 4 and 5, SPK_PLAN_HOLD) */
	float after_d[2] = { 0.0f, 0.0f }, after_ph[2] = { 0.0f, 0.0f };
	int   l;

	if (g_SpkNumSteps == 0)
	{
		printf("[SPK] no steps to play\n");
		return;
	}
	while (!g_SpkDone)
	{
		Spk_Result r;
		const Spk_Step *st;
		const char *why = "", *note = "";
		float tone_ms = 0.0f, rest_db, pk_db;
		int t, h, fail = 0;
		/* g_SpkCmd first: the callback clears both flags when it takes a command, so with the other order an interrupt
		 * between the two reads makes the old result look new */
		if ((g_SpkCmd != 0) || !g_SpkPosted)
		{
			continue;
		}
		for (t = 0; t < 2; t++)                  /* field by field: g_SpkRes is volatile */
		{
			for (h = 0; h < 3; h++) r.amp[t][h] = g_SpkRes.amp[t][h];
			r.ph[t] = g_SpkRes.ph[t];
		}
		r.ms = g_SpkRes.ms;
		r.err_peak = g_SpkRes.err_peak;
		r.step = g_SpkRes.step;
		st = &g_SpkStep[r.step];

		for (t = 0; t < 2; t++)
		{
			float err_db, gain, ph, dlt, dph, h2, h3;
			int ri;
			char tag[16];
			if (st->y[t] <= 0.0f)
			{
				continue;
			}
			ri = (st->f[t] == SPK_BPF_HZ) ? 0 : 1;
			err_db = 20.0f * log10f(r.amp[t][0] + 1e-12f);
			gain = err_db - 20.0f * log10f(st->y[t]);
			ph = r.ph[t] * (180.0f / 3.14159265f);
			if (!ref_set[ri])
			{
				ref_gain[ri] = gain;
				ref_ph[ri] = ph;
				ref_set[ri] = 1;
			}
			dlt = gain - ref_gain[ri];
			dph = ph - ref_ph[ri];
			if (dph > 180.0f) dph -= 360.0f;
			if (dph <= -180.0f) dph += 360.0f;
			h2 = 20.0f * log10f((r.amp[t][1] + 1e-12f) / (r.amp[t][0] + 1e-12f));
			h3 = 20.0f * log10f((r.amp[t][2] + 1e-12f) / (r.amp[t][0] + 1e-12f));
			tone_ms += 0.5f * r.amp[t][0] * r.amp[t][0];
			if (st->hold)
			{
				snprintf(tag, sizeof(tag), " hold %d", st->hold);
				if (dlt < hold_lo[ri]) hold_lo[ri] = dlt;
				if (dlt > hold_hi[ri]) hold_hi[ri] = dlt;
				if (dph < hold_ph_lo[ri]) hold_ph_lo[ri] = dph;
				if (dph > hold_ph_hi[ri]) hold_ph_hi[ri] = dph;
			}
			else
			{
				tag[0] = '\0';
			}
			if ((st->ladder == 3) && (st->hold == 0))
			{
				need_d[ri] = dlt;
				need_ph[ri] = dph;
			}
			else if (st->ladder >= 4)
			{
				after_set[ri] = 1;
				after_d[ri] = dlt;
				after_ph[ri] = dph;
			}
			printf("[SPK %2d/%d L%d%s] %4.0f Hz (%s) y=%.2f  err %6.1f dBFS  gain %6.2f dB (d %+5.2f, ph %+6.1f)  "
				   "H2 %5.1f  H3 %5.1f dBc\n", r.step + 1, g_SpkNumSteps, st->ladder, tag, st->f[t], tone_name[ri],
				   st->y[t], err_db, gain, dlt, dph, h2, h3);
			if ((st->y[0] > 0.0f) && (st->y[1] > 0.0f))     /* two tones: a smaller drop is noted, the hold goes on */
			{
				if (dlt < -SPK_MAX_TWO_TONE_DROP_DB)
				{
					fail = 1;
					why = "gain dropped (two tones)";
				}
				else if (dlt < -SPK_MAX_COMPRESSION_DB)
				{
					note = ", gain drop noted";
				}
			}
			else if (dlt < -SPK_MAX_COMPRESSION_DB)
			{
				fail = 1;
				why = "gain dropped (compression)";
			}
			if ((h2 > SPK_MAX_HARMONIC_DBC) || (h3 > SPK_MAX_HARMONIC_DBC))
			{
				fail = 1;
				why = "harmonics";
			}
		}
		rest_db = 10.0f * log10f(((r.ms > tone_ms) ? (r.ms - tone_ms) : 1e-20f) / (tone_ms + 1e-20f));
		pk_db = 20.0f * log10f(r.err_peak + 1e-12f);
		if (pk_db > SPK_MAX_ERR_PEAK_DBFS)
		{
			fail = 1;
			why = "Err ADC near full scale (a loud noise, or the mic gain)";
		}
		printf("[SPK %2d/%d] rest %5.1f dB re tones, Err ADC peak %5.1f dBFS -> %s%s\n", r.step + 1, g_SpkNumSteps,
			   rest_db, pk_db, fail ? "FAIL: " : "ok", fail ? why : note);
		if (fail)
		{
			printf("[SPK] ladder %d stops here\n", st->ladder);
		}
		fflush(stdout);

		if ((st->ladder == 1) || (st->ladder == 2))
		{
			if (fail)
			{
				fail_y[st->ladder] = st->y[0];
				fail_why[st->ladder] = why;
			}
			else
			{
				clean_y[st->ladder] = st->y[0];
			}
		}
		else if (st->ladder == 3)
		{
			if (st->hold == 0)
			{
				need_ok = fail ? 0 : 1;
			}
			else
			{
				hold_n++;
				if (!fail) hold_ok++;
			}
		}
		g_SpkCmd = fail ? SPK_SKIP_LADDER : SPK_NEXT;
	}

	printf("\n[SPK] ===== done, output muted =====\n");
	for (l = 1; l <= 2; l++)
	{
		const float f = (l == 1) ? SPK_BPF_HZ : SPK_F0_HZ;
		if (fail_y[l] > 0.0f)
		{
			printf("[SPK] %4.0f Hz alone: clean up to y = %.2f, failed at %.2f (%s)\n", f, clean_y[l], fail_y[l],
				   fail_why[l]);
		}
		else
		{
			printf("[SPK] %4.0f Hz alone: clean at every level played, up to y = %.2f\n", f, clean_y[l]);
		}
	}
	printf("[SPK] Need (%.0f Hz %.2f + %.0f Hz %.2f, peak %.2f): %s\n", SPK_BPF_HZ, SPK_BPF_NEED, SPK_F0_HZ,
		   SPK_F0_NEED, SPK_BPF_NEED + SPK_F0_NEED, (need_ok < 0) ? "not run" : (need_ok ? "ok" : "FAILED"));
	if (need_ok >= 0)
	{
		printf("[SPK] Two tones re the first step per tone: BPF d %+.2f dB ph %+.1f deg, f0 d %+.2f dB ph %+.1f deg\n",
			   need_d[0], need_ph[0], need_d[1], need_ph[1]);
	}
	if (hold_n > 0)
	{
		printf("[SPK] Hold: %d of %d segments ok (%.0f s). Re the first step per tone: BPF d %+.2f .. %+.2f dB,"
			   " ph %+.1f .. %+.1f deg; f0 d %+.2f .. %+.2f dB, ph %+.1f .. %+.1f deg\n", hold_ok, SPK_HOLD_SEGMENTS,
			   (float)hold_ok * SPK_HOLD_S, hold_lo[0], hold_hi[0], hold_ph_lo[0], hold_ph_hi[0], hold_lo[1],
			   hold_hi[1], hold_ph_lo[1], hold_ph_hi[1]);
	}
	for (l = 0; l < 2; l++)
	{
		if (after_set[l])
		{
			printf("[SPK] %s alone after the hold: d %+.2f dB, ph %+.1f deg re the first step\n", tone_name[l],
				   after_d[l], after_ph[l]);
		}
	}
	printf("[SPK] ANC_SAFE_OUTPUT_LIMIT clamps the sum of all tones, so it belongs at or below the lower clean level\n");
	fflush(stdout);
}

#endif /* _SPEAKER_TEST_H_ */
