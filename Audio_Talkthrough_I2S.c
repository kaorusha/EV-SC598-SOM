/*********************************************************************************
Copyright(c) 2021-2022 Analog Devices, Inc. All Rights Reserved.
This software is proprietary. By using this software you agree
to the terms of the associated Analog Devices License Agreement.
*********************************************************************************/

/*****************************************************************************
 * Audio_Talkthrough_I2S.c
 *****************************************************************************/

#include <sys/platform.h>
#include "adi_initialize.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/adi_core.h>
#include "adi_initialize.h"
#include <services/int/adi_int.h>
#include <drivers/sport/adi_sport.h>
#include <services/spu/adi_spu.h>
#include <drivers/twi/adi_twi.h>
#include <adi_osal.h>
#include "ADAU_1962Common.h"
#include "ADAU_1979Common.h"
#include "math.h"
#include <SRU.h>
#include "Audio_Talkthrough_I2S.h"
#include "anc_fxnlms.h"
#define DO_CYCLE_COUNTS  /* enable START/STOP_CYCLE_COUNT macros (only used when ENABLE_CYCLE_METER = 1) */
#include <cycle_count.h>

/*
 * Application Mode (single switch):
 *   APP_MODE_ANC          : FxNLMS ANC, 48 kHz core between 192 kHz codec samples (anc_rate.h).
 *                           J18 Right = anti-noise y(n), J4/J17 = Error / Ref Mic monitor.
 *   APP_MODE_PATH_MEASURE : Bridge for tools/pathmeas.py (secondary / feedback path), routed through the same
 *                           192 <-> 48 kHz boundary as the ANC core, so it measures the core-domain path.
 *                           J16 Left = timing reference sweeps, J16 Right = sweep.
 *                           J18 Right = sweep to speaker (Left muted).
 *                           J17 Left  = Error Mic + reference (secondary path), J17 Right = Ref Mic + reference (feedback path).
 *   APP_MODE_DIAG_MIC     : Mic passthrough, no ANC. J18 Right = Ref Mic, J4/J17 = Error / Ref Mic.
 *   APP_MODE_DIAG_LINE_IN : J16 Left/Right -> J18 and J4/J17 Left/Right (check PC line-in, use with slot peak meter).
 *   APP_MODE_DIAG_DAC_ID  : DAC slot s beeps (s+1) times every 2 s, no input (check slot -> jack/ear wiring).
 */
#define APP_MODE_ANC            0
#define APP_MODE_PATH_MEASURE   1
#define APP_MODE_DIAG_MIC       2
#define APP_MODE_DIAG_LINE_IN   3
#define APP_MODE_DIAG_DAC_ID    4

#define APP_MODE                APP_MODE_PATH_MEASURE  /* Select application mode here */

/*
 * Gain staging (measured, Windows volume 100%): REW -12 dBFS -> J16 slot peak -6 dBFS (tracks 1:1).
 * Idle broadband floor on every slot is about -60 dBFS peak.
 */
/* Digital gain for electrical timing chirp mixed into mic return: -6 dBFS * 0.05 (-26 dB) = -32 dBFS, ~28 dB above floor */
#define CALIBRATION_CHIRP_GAIN  0.05f
#define CALIBRATION_SWEEP_GAIN  1      /* Integer gain; J16 already peaks near -6 dBFS, 2x would hit 0 dBFS */

/*
 * Hardware Slot Mapping on EV-SOMCRR-EZKIT:
 * ADC (ADAU1979), Config_array_ADC registers:
 *   SAI_CTRL1 = 0x00 (LRCLK 50% duty, same as DAC): the ADC frame starts half a TDM4 frame (2 slots)
 *   after SPORT 4B, so ADC slot k lands in SPORT slot (k+2)%4. CMAP compensates for it:
 *   CMAP12 = 0x23: AIN1 -> ADC Slot 3 (SPORT 1), AIN2 -> ADC Slot 2 (SPORT 0)
 *   CMAP34 = 0x01: AIN3 -> ADC Slot 1 (SPORT 3), AIN4 -> ADC Slot 0 (SPORT 2)
 *   Verified with APP_MODE_DIAG_LINE_IN + slot peak meter (SPORT slots):
 *     Slot 0 = Error Mic (enclosure outlet), Slot 1 = Ref Mic (next to fan)
 *     Slot 2 = J16 Right (PC R), Slot 3 = J16 Left (PC L)
 * DAC (ADAU1962), verified with APP_MODE_DIAG_DAC_ID:
 *   Slot 0/1 = J18 Left/Right (speaker)
 *   Slot 2/3 = J17 Left/Right (monitor to PC Mic In; J4 = J17 with headphone amp)
 *   NOTE: Silkscreen says J17 = DAC1/2, J18 = DAC3/4. The DAC frame start is offset by half a
 *         TDM4 frame (2 slots) from SPORT 4A, so DAC1/2 picks up SPORT slots 2/3.
 *         Cause: DAC_CTRL1 defaults (LRCLK_MODE=0 50% duty, LRCLK_POL=0 Left/Odd = LRCLK Low) make the
 *         DAC start its frame on the falling edge, while SPORT starts on the rising edge.
 *         Kept as-is on purpose (DAC_CTRL1 = 0x01); LRCLK_POL=1 (0x21) should align it but is untested.
 *         The ADC shows the same 2-slot offset in 50% duty mode (handled by CMAP above).
 *
 * PC channel configuration (tools/pathmeas.py; REW used the same channels):
 *   - Output:             R (PC Right -> J16 Ring -> Slot 2 -> Sweep to Speaker)
 *   - Timing ref output:  L (PC Left  -> J16 Tip  -> Slot 3 -> Reference to J17)
 */
#define J12_ERR_MIC_SLOT        0   /* Error Mic (enclosure outlet) */
#define J12_REF_MIC_SLOT        1   /* Ref Mic (next to fan) */
#define J16_SWEEP_SLOT          2   /* J16 AIN4 (PC Right = Sweep) */
#define J16_CHIRP_SLOT          3   /* J16 AIN3 (PC Left  = Timing Reference) */

#if (COUNT % (4 * ANC_DECIM)) != 0
#error "COUNT must hold whole 48 kHz core samples (4 * ANC_DECIM ints)"
#endif
#define BLOCK_FRAMES            (COUNT / 4)                /* TDM4 frames per DMA block */
#define CALLBACKS_PER_S         (192000u / BLOCK_FRAMES)   /* 48000 with 4-frame blocks */

/* Per-slot ADC peak meter, printed from main loop (not from the SPORT callback) */
#define ENABLE_SLOT_PEAK_METER  0  /* JTAG printf halts the core -> clicks; set 0 during sweeps */
#define PEAK_METER_CALLBACKS    (2u * CALLBACKS_PER_S)  /* print every ~2 s */

/* Raw ADC capture: ~1 s after start-up, records ADC_CAPTURE_LEN consecutive samples of one slot into
 * g_AdcCapture[] and main() writes them to ADC_CAPTURE_FILE over JTAG (decimal + hex per line, for checking
 * bit patterns). Audio glitches while the file is written. g_AdcCapture can also be viewed in the CCES Plot view. */
#define ENABLE_ADC_CAPTURE      0
#define ADC_CAPTURE_SLOT        J16_SWEEP_SLOT  /* J16 Right (PC R) */
#define ADC_CAPTURE_LEN         4096u           /* 21.3 ms @ 192kHz = 6.4 periods of 300 Hz */
#define ADC_CAPTURE_FILE        "D:/cindy_hsieh/My Documents/project/ANC/EV-SC598-SOM/Audio_Talkthrough_I2S/adc_capture.txt"

/* Diagnostic: run the codecs at 96 kHz (BCLK 12.288 MHz instead of 24.576 MHz) to check whether the
 * stair-step ADC samples at 192 kHz are a serial-port timing problem. Only the codec FS registers change;
 * ANC tuning, cycle budget and timing comments still assume 192 kHz, so use with DIAG modes only. */
#define TEST_FS_96K             0

/* SPORT 4B (ADC) sampling edge. At 192 kHz TDM4 (BCLK 24.576 MHz) the ADC SDATA line is too slow to fall
 * within half a BCLK: capture showed runs of 1s spilling into later bits (even bit 7, below the 24-bit word),
 * clean at 96 kHz. The ADC launches data on the BCLK falling edge; sampling on the next falling edge gives each
 * bit a full BCLK period to settle. FS is sampled on the same edge, so frame alignment stays consistent. */
#define SPORT_RX_SAMPLE_FALLING 1

#if TEST_FS_96K
#define DAC_CTRL0_FS            0x12  /* TDM4 (bits 5:3=010), FS 64-96kHz (bits 2:1=01) */
#define DAC_CTRL2_OSR           0x04  /* 256x fs OSR (bit 1=0), TDM 32-BCLK slot (bit 4=0), Auto Mute EN (bit 2=1) */
#define ADC_SAI_CTRL0_FS        0x13  /* TDM4 (bits 5:3=010), FS 64-96kHz (bits 2:0=011) */
#else
#define DAC_CTRL0_FS            0x16  /* 192kHz Low Delay Mode (bits 2:1=11), TDM4 mode (bits 5:3=010) */
#define DAC_CTRL2_OSR           0x06  /* 128x fs OSR (bit 1=1 for 192kHz), TDM 32-BCLK slot (bit 4=0), Auto Mute EN (bit 2=1) */
#define ADC_SAI_CTRL0_FS        0x14  /* TDM4 (bits 5:3=010), FS 192kHz (bits 2:0=100) */
#endif

/* Process_Block() cycle meter, printed from main loop every PEAK_METER_CALLBACKS (Release build only;
 * Debug -O0 starves the main loop and nothing prints). Log fields are explained at the printf in main().
 * Counts via <cycle_count.h>: EMUCLK increments once per core clock; 32-bit, wraps ~4.29 s, unsigned diff is safe.
 * "period" = EMUCLK cycles between prints / blocks = real block interval (independent of CORE_CLOCK_HZ):
 *   period ~= budget       -> real time
 *   period ~= avg + ~1.4k  -> CPU saturated, DMA blocks dropped (avg excludes callback overhead)
 * Ignore the first print: blocks count from boot, the elapsed stamp starts in main loop.
 */
#define ENABLE_CYCLE_METER      0
#define CORE_CLOCK_HZ           1000000000.0f  /* SHARC+ core clock, check against the CGU setting */
#define BLOCK_BUDGET_CYCLES     (CORE_CLOCK_HZ * (float)BLOCK_FRAMES / 192000.0f)  /* 4 frames @ 192kHz = 20.8 us */

struct Config_Table
{
	short Reg_Add;
	char  Value;
};

char Config_read_DAC[28];
char Config_read_ADC[16];
int count;
struct Config_Table Config_array_DAC[28] = {
		   	    {     ADAU1962_PDN_CTRL_1,		0x00},
		   	    {     ADAU1962_PDN_CTRL_2,	    0xff},
		   	    {     ADAU1962_PDN_CTRL_3,	    0x0f},
		   	    {     ADAU1962_DAC_CTRL0,		0x01},
		   	    {     ADAU1962_DAC_CTRL1,		0x01}, /* Master, LRCLK 50% duty + normal polarity: DAC slot 0 = SPORT slot 2 (see slot mapping NOTE) */
		   	    {     ADAU1962_DAC_CTRL2,		DAC_CTRL2_OSR},
		   	    {     ADAU1962_DAC_MUTE1,	    0x0},
		   	    {     ADAU1962_DAC_MUTE2,	    0x00},
		   	    {     ADAU1962_MSTR_VOL,		0x00},
			    {     ADAU1962_DAC1_VOL,	    0x00},
			    {     ADAU1962_DAC2_VOL,		0x00},
				{     ADAU1962_DAC3_VOL,		0x00},
				{     ADAU1962_DAC4_VOL,		0x00},
				{     ADAU1962_DAC5_VOL,		0x00},
				{     ADAU1962_DAC6_VOL,		0x00},
				{     ADAU1962_DAC7_VOL,		0x00},
				{     ADAU1962_DAC8_VOL,	    0x00},
				{     ADAU1962_DAC9_VOL,		0x00},
				{     ADAU1962_DAC10_VOL,		0x00},
				{     ADAU1962_DAC11_VOL,		0x00},
				{     ADAU1962_DAC12_VOL,		0x00},
				{     ADAU1962_PAD_STRGTH,		0x00},
				{     ADAU1962_DAC_PWR1,		0xaa},
				{     ADAU1962_DAC_PWR2,		0xaa},
				{     ADAU1962_DAC_PWR3,		0xaa},
				{     ADAU1962_PDN_CTRL_2,	    0x00}, /* Power up DAC 1~8 (0x00 = all active in ADAU1962) */
				{     ADAU1962_PDN_CTRL_3,	    0x00}, /* Power up DAC 9~12 */
				{     ADAU1962_DAC_CTRL0,		DAC_CTRL0_FS}

};

struct Config_Table Config_array_ADC[16] = {
		 {ADAU1979_REG_BOOST			,	0x00},
		 {ADAU1979_REG_MICBIAS			,	0x00},
		 {ADAU1979_REG_BLOCK_POWER_SAI	,	0x30},
		 {ADAU1979_REG_SAI_CTRL0		,	ADC_SAI_CTRL0_FS},
		 {ADAU1979_REG_SAI_CTRL1		,	0x00}, /* SDATAOUT1 (bit 7=0), LRCLK 50% duty (bit 3=0, matches DAC master DAC_CTRL1), Slave to DAC clock (bit 0=0) */
		 {ADAU1979_REG_CMAP12			,	0x23}, /* AIN1 -> ADC slot 3, AIN2 -> ADC slot 2 (+2 slot offset -> SPORT 1 / 0) */
		 {ADAU1979_REG_CMAP34			,	0x01}, /* AIN3 -> ADC slot 1, AIN4 -> ADC slot 0 (+2 slot offset -> SPORT 3 / 2) */
		 {ADAU1979_REG_SAI_OVERTEMP		,	0xf0}, /* SDATAOUT drive enable CH1~4 (bits 7:4); on/off only, no drive strength setting */
		 {ADAU1979_REG_POST_ADC_GAIN1 	,	0xA0},
		 {ADAU1979_REG_POST_ADC_GAIN2 	,	0xA0},
		 {ADAU1979_REG_POST_ADC_GAIN3 	,	0xA0},
		 {ADAU1979_REG_POST_ADC_GAIN4 	,	0xA0},
		 {ADAU1979_REG_ADC_CLIP			,	0x00},
		 {ADAU1979_REG_DC_HPF_CAL		,	0x0F}, /* Enable DC HPF on channels 1~4 to eliminate DC bias for adaptive filter stability */
		 {ADAU1979_REG_BLOCK_POWER_SAI	,	0x3f},
		 {ADAU1979_REG_MISC_CONTROL		,	0x00}
};

/*=============  L O C A L    F U N C T I O N    P R O T O T Y P E S =============*/
static int ADAU_1962_Pllinit(void);
static int ADAU_1979_Pllinit(void);
void Switch_Configurator(void);
static int ADAU_1962_init(void);
static int ADAU_1979_init(void);
void SRU_Init(void);
static int Sport_Init(void);
#if (ADI_CONFIG_CONTINUOUS_AUDIO == 0)
static int Sport_Stop(void);
#endif
static int SPU_init(void);
static int Init_TWI(void);
static int Stop_TWI(void);
extern void ConfigSoftSwitches_ADC_DAC(void);
extern void ConfigSoftSwitches_ADAU_Reset(void);

/*==============  D E F I N E S  ===============*/

/*=============  D A T A  =============*/

/* Buffers */
#if defined(__ADSPCORTEXA55__)
__attribute__((section(".l2_uncached")))
#endif
ADI_CACHE_ALIGN static int   int_SP0ABuffer1[COUNT*2];
#if defined(__ADSPCORTEXA55__)
__attribute__((section(".l2_uncached")))
#endif
ADI_CACHE_ALIGN static int   int_SP0ABuffer2[COUNT*2];

#if defined(__ADSPCORTEXA55__)
__attribute__((section(".l2_uncached")))
#endif
ADI_CACHE_ALIGN static int   int_SP0ABuffer4[COUNT];
#if defined(__ADSPCORTEXA55__)
__attribute__((section(".l2_uncached")))
#endif
ADI_CACHE_ALIGN static int   int_SP0ABuffer5[COUNT];

/* Destination Sport PDMA ring (TX): int_SP0ABuffer1, int_SP0ABuffer2, int_SP0ABuffer1, ... */
ADI_PDMA_DESC_LIST iDESC_LIST_SP4A[DMA_NUM_DESC];

/* Source Sport PDMA ring (RX): int_SP0ABuffer4, int_SP0ABuffer5, int_SP0ABuffer4, ... */
ADI_PDMA_DESC_LIST iSRC_LIST_SP4B[DMA_NUM_DESC];


/* Count of RX callbacks (start-up timing in main) */
volatile uint32_t TestCallbackCount = 0;

/* System byte addresses of int_SP0ABuffer4 / int_SP0ABuffer5 as the RX DMA uses them (set in Sport_Init) */
static uint32_t g_RxBuf4Sys, g_RxBuf5Sys;

/* Memory required for SPORT */
static uint8_t SPORTMemory4A[ADI_SPORT_MEMORY_SIZE];
static uint8_t SPORTMemory4B[ADI_SPORT_MEMORY_SIZE];

/* SPORT Handle */
static ADI_SPORT_HANDLE hSPORTDev4ATx;//TX
static ADI_SPORT_HANDLE hSPORTDev4BRx;//RX

/* Memory required for TWI */
uint8_t TwideviceMemory[ADI_TWI_MEMORY_SIZE];

/* TWI driver handle */
static ADI_TWI_HANDLE hTwiDevice;

/* SPU handle */
static ADI_SPU_HANDLE      ghSpu;

/* Memory required for the SPU operation */
uint8_t             SpuMemory[ADI_SPU_MEMORY_SIZE];


/* Dev buffer for configuring ADC-DAC through TWI*/
static uint8_t devBuffer[BUFFER_SIZE];

#if ENABLE_SLOT_PEAK_METER
/* Max |sample| per ADC slot since last print, updated in SPORT callback */
static volatile int g_SlotPeak[4];

static void Update_Slot_Peak(const int *pIn)
{
	int i, s, v;
	for(i=0; i<COUNT; i+=4)
	{
		for(s=0; s<4; s++)
		{
			v = pIn[i+s];
			v = (v < 0) ? -(v + 1) : v; /* avoid overflow on INT_MIN */
			if (v > g_SlotPeak[s]) g_SlotPeak[s] = v;
		}
	}
}
#endif

#if ENABLE_ADC_CAPTURE
int g_AdcCapture[ADC_CAPTURE_LEN];          /* non-static so the Plot / Expressions views can find it */
volatile uint32_t g_AdcCaptureIdx = 0;
volatile uint32_t g_AdcCaptureDone = 0;

static void Update_Adc_Capture(const int *pIn)
{
	int i;
	if (g_AdcCaptureDone)
	{
		return;
	}
	for(i=0; i<COUNT; i+=4)
	{
		g_AdcCapture[g_AdcCaptureIdx++] = pIn[i + ADC_CAPTURE_SLOT];
		if (g_AdcCaptureIdx >= ADC_CAPTURE_LEN)
		{
			g_AdcCaptureIdx = 0;
			g_AdcCaptureDone = 1;
			return;
		}
	}
}
#endif

#if ENABLE_CYCLE_METER
/* Process_Block() cycles since last print, updated in SPORT callback */
static volatile uint32_t g_CycleMax = 0;
static volatile uint32_t g_CycleSum = 0;  /* ~2 s of blocks, each under its budget: < 2e9 cycles, fits in 32 bits */
static volatile uint32_t g_CycleBlocks = 0;
#endif

#if (APP_MODE == APP_MODE_DIAG_DAC_ID)
#define ID_TONE_TABLE_LEN   192u     /* 1 kHz at 192 kHz */
#define ID_BEEP_ON          19200u   /* 100 ms tone per beep */
#define ID_BEEP_PERIOD      48000u   /* 250 ms per beep */
#define ID_CYCLE            384000u  /* 2 s cycle (multiple of ID_TONE_TABLE_LEN) */

static int g_IdTone[ID_TONE_TABLE_LEN];
static uint32_t g_IdSample = 0;

static void Id_Tone_Init(void)
{
	uint32_t n;
	for(n=0; n<ID_TONE_TABLE_LEN; n++)
	{
		g_IdTone[n] = (int)(0.1f * 2147483647.0f * sinf(6.2831853f * (float)n / (float)ID_TONE_TABLE_LEN)); /* -20 dBFS */
	}
}

/* DAC Slot ID: slot s beeps (s+1) times per cycle */
static void Id_Tone_Process_Block(int *pOut)
{
	int i, s;
	uint32_t beep, on;
	for(i=0; i<COUNT; i+=4)
	{
		int tone = g_IdTone[g_IdSample % ID_TONE_TABLE_LEN];
		beep = g_IdSample / ID_BEEP_PERIOD;
		on   = ((g_IdSample % ID_BEEP_PERIOD) < ID_BEEP_ON) ? 1u : 0u;
		for(s=0; s<4; s++)
		{
			pOut[i+s] = (on && (beep <= (uint32_t)s)) ? tone : 0;
		}
		g_IdSample = (g_IdSample + 1u) % ID_CYCLE;
	}
}
#endif

#if (APP_MODE == APP_MODE_PATH_MEASURE)
/* Float to int32 with saturation (2147483520.0f is the largest float below 2^31) */
static inline int Saturate_To_Int(float x)
{
	if (x >  2147483520.0f) return  2147483647;
	if (x < -2147483648.0f) return (-2147483647 - 1);
	return (int)x;
}

/* Path measurement bridge for tools/pathmeas.py, through the ANC core's 192 <-> 48 kHz boundary (anc_rate.h):
 * every input is decimated and every output interpolated exactly as in ANC_Process_Block, and the 48 kHz samples
 * are taken at the same frame of each block. The reference passes the same boundary, so the measured path runs from
 * a core output sample to a core Err (or Ref) sample, rate filters and DMA timing included, and
 * tools/export_paths.py writes it to second_path.h / feedback_path.h without conversion. */
static ANC_Rate_State g_PmDec[4];   /* Err, Ref, sweep (J16 R), timing reference (J16 L) */
static ANC_Rate_State g_PmInt[3];   /* J18 R, J17 L, J17 R */

static void Path_Measure_Process_Block(const int *pIn, int *pOut)
{
	int i, k;
	for(i=0; i<COUNT; i+=4*ANC_DECIM)
	{
		float err = 0.0f, ref = 0.0f, sweep = 0.0f, chirp = 0.0f;
		for(k=0; k<ANC_DECIM; k++)
		{
			const int *pf = &pIn[i + 4*k];
			err   = anc_rate_filter(&g_PmDec[0], (float)pf[J12_ERR_MIC_SLOT]);
			ref   = anc_rate_filter(&g_PmDec[1], (float)pf[J12_REF_MIC_SLOT]);
			sweep = anc_rate_filter(&g_PmDec[2], (float)pf[J16_SWEEP_SLOT]);
			chirp = anc_rate_filter(&g_PmDec[3], (float)pf[J16_CHIRP_SLOT]);
		}
		sweep *= (float)CALIBRATION_SWEEP_GAIN;
		chirp *= CALIBRATION_CHIRP_GAIN;
		for(k=0; k<ANC_DECIM; k++)
		{
			float z = (k == 0) ? (float)ANC_DECIM : 0.0f;   /* zero-stuffing; x ANC_DECIM restores the level */
			int *pt = &pOut[i + 4*k];
			pt[0] = 0;                                                                 /* J18 Left  (Tip):  Mute */
			pt[1] = Saturate_To_Int(anc_rate_filter(&g_PmInt[0], z * sweep));          /* J18 Right (Ring): Sweep to Speaker */
			pt[2] = Saturate_To_Int(anc_rate_filter(&g_PmInt[1], z * (err + chirp)));  /* J17 Left  (Tip):  Sec Path (Err + Ref) */
			pt[3] = Saturate_To_Int(anc_rate_filter(&g_PmInt[2], z * (ref + chirp)));  /* J17 Right (Ring): FB Path (Ref + Ref) */
		}
	}
}
#endif

/* Route one DMA block (COUNT samples = BLOCK_FRAMES TDM4 frames) according to APP_MODE; shared by both ping-pong buffers */
static void Process_Block_Body(const int *pIn, int *pOut);

static void Process_Block(const int *pIn, int *pOut)
{
#if ENABLE_CYCLE_METER
	cycle_t t0, dt;
	START_CYCLE_COUNT(t0);
	Process_Block_Body(pIn, pOut);
	STOP_CYCLE_COUNT(dt, t0);
	if ((uint32_t)dt > g_CycleMax) g_CycleMax = (uint32_t)dt;
	g_CycleSum += (uint32_t)dt;
	g_CycleBlocks++;
#else
	Process_Block_Body(pIn, pOut);
#endif
}

static void Process_Block_Body(const int *pIn, int *pOut)
{
#if ENABLE_SLOT_PEAK_METER
	Update_Slot_Peak(pIn);
#endif
#if ENABLE_ADC_CAPTURE
	Update_Adc_Capture(pIn);
#endif
#if (APP_MODE == APP_MODE_ANC)
	ANC_Process_Block(pIn, pOut, COUNT);
#elif (APP_MODE == APP_MODE_PATH_MEASURE)
	Path_Measure_Process_Block(pIn, pOut);
#elif (APP_MODE == APP_MODE_DIAG_DAC_ID)
	Id_Tone_Process_Block(pOut);
#elif (APP_MODE == APP_MODE_DIAG_MIC) || (APP_MODE == APP_MODE_DIAG_LINE_IN)
	{
		int i;
		for(i=0; i<COUNT; i+=4)
		{
#if (APP_MODE == APP_MODE_DIAG_MIC)
			pOut[i+0] = 0;                          /* J18 Left  (Tip):  Mute */
			pOut[i+1] = pIn[i + J12_REF_MIC_SLOT];  /* J18 Right (Ring): Ref Mic */
			pOut[i+2] = pIn[i + J12_ERR_MIC_SLOT];  /* J17 Left  (Tip):  Error Mic */
			pOut[i+3] = pIn[i + J12_REF_MIC_SLOT];  /* J17 Right (Ring): Ref Mic */
#else
			pOut[i+0] = pIn[i + J16_CHIRP_SLOT];    /* J18 Left  <- J16 Left  */
			pOut[i+1] = pIn[i + J16_SWEEP_SLOT];    /* J18 Right <- J16 Right */
			pOut[i+2] = pIn[i + J16_CHIRP_SLOT];    /* J17 Left  <- J16 Left  */
			pOut[i+3] = pIn[i + J16_SWEEP_SLOT];    /* J17 Right <- J16 Right */
#endif
		}
	}
#else
#error "Unknown APP_MODE"
#endif
}


static void SPORTCallback(
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg
)
{
    /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
        	{
        		/* The finished RX buffer is read off the RX DMA's current address, not counted. Counting callbacks broke
        		 * whenever the core was halted (every JTAG printf or file write): a halt spanning an even number of blocks
        		 * swapped the pair for the rest of the run, so the block still being filled was processed and the output
        		 * went to the buffer the TX DMA had already fetched, 2 blocks (8 frames, 42 us) late (DMA probe, 2026-11-06).
        		 * Here the RX DMA is at most a word into the next buffer, or parked at the end of the finished one
        		 * (Buffer4 + 4*COUNT bytes is also where Buffer5 starts when the two are adjacent). Buffer k's output goes
        		 * to the TX buffer of the same parity, which the TX DMA fetches for block k+2 about 16.7 us from now. */
        		uint32_t rxCur = (uint32_t)(uintptr_t)*pREG_DMA11_ADDR_CUR;
        		int done4 = ((rxCur - g_RxBuf5Sys) < 4u * COUNT) || (rxCur == g_RxBuf4Sys + 4u * COUNT);

        		TestCallbackCount +=1;

        		if (done4)
        		{
        		   Process_Block(int_SP0ABuffer4, int_SP0ABuffer1);
        		}
        		else
				{
				   Process_Block(int_SP0ABuffer5, int_SP0ABuffer2);
				}
        	}

        		break;
        default:
        	 break;
    }



    /* return */
}

/*
 * Prepares SPU configuration.
 *
 * Parameters
 *  None
 *
 * Returns
 *  None
 *
 */
static int SPU_init(void)
{
    if(adi_spu_Init(0, SpuMemory, NULL, NULL, &ghSpu) != ADI_SPU_SUCCESS)
    {
    	REPORT_ERROR("Failed to initialize SPU service\n");
		return FAILED;
    }

    /* Make SPORT 0A to generate secure transactions */
    if(adi_spu_EnableMasterSecure(ghSpu, SPORT_4A_SPU, true) != ADI_SPU_SUCCESS)
    {
    	REPORT_ERROR("Failed to enable Master secure for SPORT0A\n");
		return FAILED;
    }

    /* Make SPORT 0B to generate secure transactions */
    if(adi_spu_EnableMasterSecure(ghSpu, SPORT_4B_SPU, true) != ADI_SPU_SUCCESS)
    {
    	REPORT_ERROR("Failed to enable Master secure for SPORT0B\n");
		return FAILED;
    }

    return SUCCESS;
}


void Switch_Configurator()
{
	int delay11=0xffff;


	/* Software Switch Configuration for Enabling ADC-DAC */
	ConfigSoftSwitches_ADC_DAC();

	while(delay11--)
	{
		asm("nop;");
	}

	/* Software Switch Configuration for Re-Setting ADC-DAC  */
	ConfigSoftSwitches_ADAU_Reset();


	/* wait for Codec to up */
	delay11=0xffff;
	while(delay11--)
	{
		asm("nop;");
	}
}


/*
 * Prepares SRU configuration.
 *
 * Parameters
 *  None
 *
 * Returns
 *  None
 *
 */
void SRU_Init(void)
{
	*pREG_PADS0_DAI0_IE=0xfffff;
	*pREG_PADS0_DAI1_IE=0xfffff;

    SRU2(LOW,DAI1_PBEN05_I);

    SRU2(DAI1_PB05_O,SPT4_ACLK_I); /*DAC clock to SPORT 4A*/
    SRU2(DAI1_PB05_O,SPT4_BCLK_I); /*DAC clock to SPORT 4B*/

    SRU2(DAI1_PB04_O,SPT4_AFS_I);  /*DAC FS to SPORT 4A*/
    SRU2(DAI1_PB04_O,SPT4_BFS_I);  /*DAC FS to SPORT 4B*/
    SRU2(LOW,DAI1_PBEN04_I);

    SRU2(SPT4_AD0_O,DAI1_PB01_I); /* SPORT 4A to DAC*/
    SRU2(HIGH,DAI1_PBEN01_I);

    SRU2(DAI1_PB05_O,DAI1_PB12_I);  /*DAC clock to ADC */
    SRU2(HIGH,DAI1_PBEN12_I);

    SRU2(DAI1_PB04_O,DAI1_PB20_I);  /*DAC FS to ADC */
    SRU2(HIGH,DAI1_PBEN20_I);

    SRU2(DAI1_PB06_O,SPT4_BD0_I);
    SRU2(LOW,DAI1_PBEN06_I);

}

/* Circular rings of DMA_NUM_DESC descriptors; consecutive descriptors alternate between the two buffers, so the
 * ping-pong order is the same as with a 2-descriptor ring */
static void PrepareDescriptors (void)
{
	int *pTx[2] = {int_SP0ABuffer1, int_SP0ABuffer2};
	int *pRx[2] = {int_SP0ABuffer4, int_SP0ABuffer5};
	uint32_t i;

	for (i = 0u; i < DMA_NUM_DESC; i++)
	{
		uint32_t next = (i + 1u) % DMA_NUM_DESC;
#if defined(__ADSPCORTEXA55__)
		iDESC_LIST_SP4A[i].pStartAddr	= (uint32_t)(uintptr_t)pTx[i % 2u];
		iDESC_LIST_SP4A[i].pNxtDscp		= (uint32_t)(uintptr_t)&iDESC_LIST_SP4A[next];
		iSRC_LIST_SP4B[i].pStartAddr	= (uint32_t)(uintptr_t)pRx[i % 2u];
		iSRC_LIST_SP4B[i].pNxtDscp		= (uint32_t)(uintptr_t)&iSRC_LIST_SP4B[next];
#else
		iDESC_LIST_SP4A[i].pStartAddr	= pTx[i % 2u];
		iDESC_LIST_SP4A[i].pNxtDscp		= &iDESC_LIST_SP4A[next];
		iSRC_LIST_SP4B[i].pStartAddr	= pRx[i % 2u];
		iSRC_LIST_SP4B[i].pNxtDscp		= &iSRC_LIST_SP4B[next];
#endif
		iDESC_LIST_SP4A[i].Config		= ENUM_DMA_CFG_XCNT_INT;
		iDESC_LIST_SP4A[i].XCount		= COUNT;
		iDESC_LIST_SP4A[i].XModify		= 4;
		iDESC_LIST_SP4A[i].YCount		= 0;
		iDESC_LIST_SP4A[i].YModify		= 0;

		iSRC_LIST_SP4B[i].Config		= ENUM_DMA_CFG_XCNT_INT;
		iSRC_LIST_SP4B[i].XCount		= COUNT;
		iSRC_LIST_SP4B[i].XModify		= 4;
		iSRC_LIST_SP4B[i].YCount		= 0;
		iSRC_LIST_SP4B[i].YModify		= 0;
	}
}

/* Busy-waits until the TX DMA has fetched its first block (it then waits for the SPORT to drain it); 0 after 10 us */
static int Wait_Tx_Dma_Ready(void)
{
	cycle_t start, now;
	START_CYCLE_COUNT(start);
	do
	{
		if ((*pREG_DMA10_STAT & BITM_DMA_STAT_RUN) == ENUM_DMA_STAT_ACKWAIT)
		{
			return 1;
		}
		START_CYCLE_COUNT(now);
	} while ((uint32_t)now - (uint32_t)start < 10000u);
	return 0;
}

/* Busy-waits for a rising edge of LRCLK (DAI1_PB04, routed to both SPORT FS inputs); 0 after 100 us without one */
static int Wait_Lrclk_Rise(void)
{
	cycle_t start, now;
	uint32_t prev = *pREG_DAI1_PIN_STAT & BITM_DAI_PIN_STAT_PB04;
	START_CYCLE_COUNT(start);
	do
	{
		uint32_t cur = *pREG_DAI1_PIN_STAT & BITM_DAI_PIN_STAT_PB04;
		if (cur != 0u && prev == 0u)
		{
			return 1;
		}
		prev = cur;
		START_CYCLE_COUNT(now);
	} while ((uint32_t)now - (uint32_t)start < 100000u);
	return 0;
}

#define CHECK_SPORT_STEP(stepName, result) \
	do { \
		if ((result) != ADI_SPORT_SUCCESS) { \
			printf("  [ERROR] Sport_Init failed at %s, error code = 0x%08X\n", (stepName), (result)); \
			fflush(stdout); \
			return 1; \
		} \
	} while(0)

static int Sport_Init(void)
{
    /* SPORT return code */
    ADI_SPORT_RESULT    eResult;

	/* Open the SPORT Device 4A in Multichannel (TDM8) mode */
	eResult = adi_sport_Open(SPORT_DEVICE_4A,ADI_HALF_SPORT_A,ADI_SPORT_DIR_TX, ADI_SPORT_MC_MODE, SPORTMemory4A,ADI_SPORT_MEMORY_SIZE,&hSPORTDev4ATx);
	CHECK_SPORT_STEP("adi_sport_Open(4A TX)", eResult);

	/* Open the SPORT Device 4B in Multichannel (TDM8) mode */
	eResult = adi_sport_Open(SPORT_DEVICE_4B,ADI_HALF_SPORT_B,ADI_SPORT_DIR_RX, ADI_SPORT_MC_MODE, SPORTMemory4B,ADI_SPORT_MEMORY_SIZE,&hSPORTDev4BRx);
	CHECK_SPORT_STEP("adi_sport_Open(4B RX)", eResult);

	/* Configure the data, clock, frame sync and MCTL of SPORT Device 4A (TX: 4 channels, Slot 0~3 -> DAC 1~4) */
	eResult = adi_sport_ConfigData(hSPORTDev4ATx,ADI_SPORT_DTYPE_SIGN_FILL,31,false,false,false);
	CHECK_SPORT_STEP("adi_sport_ConfigData(4A)", eResult);
	eResult = adi_sport_ConfigClock(hSPORTDev4ATx,32,false,false,false);
	CHECK_SPORT_STEP("adi_sport_ConfigClock(4A)", eResult);
	eResult = adi_sport_ConfigFrameSync(hSPORTDev4ATx,31,false,false,false,true,false,false);
	CHECK_SPORT_STEP("adi_sport_ConfigFrameSync(4A)", eResult);
	eResult = adi_sport_ConfigMC(hSPORTDev4ATx,1u,3u,0u,true);
	CHECK_SPORT_STEP("adi_sport_ConfigMC(4A)", eResult);
	eResult = adi_sport_SelectChannel(hSPORTDev4ATx,0u,3u);
	CHECK_SPORT_STEP("adi_sport_SelectChannel(4A)", eResult);

	/* Configure the data, clock, frame sync and MCTL of SPORT Device 4B (RX: 4 channels on wire, capture 4 channels Slot 0~3) */
	eResult = adi_sport_ConfigData(hSPORTDev4BRx,ADI_SPORT_DTYPE_SIGN_FILL,31,false,false,false);
	CHECK_SPORT_STEP("adi_sport_ConfigData(4B)", eResult);
	eResult = adi_sport_ConfigClock(hSPORTDev4BRx,32,false,(SPORT_RX_SAMPLE_FALLING != 0),false);
	CHECK_SPORT_STEP("adi_sport_ConfigClock(4B)", eResult);
	eResult = adi_sport_ConfigFrameSync(hSPORTDev4BRx,31,false,false,false,true,false,false);
	CHECK_SPORT_STEP("adi_sport_ConfigFrameSync(4B)", eResult);
	eResult = adi_sport_ConfigMC(hSPORTDev4BRx,1u,3u,0u,true);
	CHECK_SPORT_STEP("adi_sport_ConfigMC(4B)", eResult);
	eResult = adi_sport_SelectChannel(hSPORTDev4BRx,0u,3u);
	CHECK_SPORT_STEP("adi_sport_SelectChannel(4B)", eResult);

	/* Register SPORT Callback function */
	eResult = adi_sport_RegisterCallback(hSPORTDev4BRx,SPORTCallback,NULL);
	CHECK_SPORT_STEP("adi_sport_RegisterCallback(4B)", eResult);

	/* Prepare descriptors */
	PrepareDescriptors();

	/* Submit the first buffer for Rx.  */
	eResult = adi_sport_DMATransfer(hSPORTDev4BRx,&iSRC_LIST_SP4B[0],(DMA_NUM_DESC),ADI_PDMA_DESCRIPTOR_LIST, ADI_SPORT_CHANNEL_PRIM);
	CHECK_SPORT_STEP("adi_sport_DMATransfer(4B Rx)", eResult);
	/* The driver has translated the descriptors to system addresses; SPORTCallback compares the RX DMA address with these */
	g_RxBuf4Sys = (uint32_t)(uintptr_t)iSRC_LIST_SP4B[0].pStartAddr;
	g_RxBuf5Sys = (uint32_t)(uintptr_t)iSRC_LIST_SP4B[1].pStartAddr;
	/* Submit the first buffer for Tx.  */
	eResult = adi_sport_DMATransfer(hSPORTDev4ATx,&iDESC_LIST_SP4A[0],(DMA_NUM_DESC),ADI_PDMA_DESCRIPTOR_LIST, ADI_SPORT_CHANNEL_PRIM);
	CHECK_SPORT_STEP("adi_sport_DMATransfer(4A Tx)", eResult);

	/* Start both SPORTs on the same frame, so S and F keep the same delay from boot to boot. Enabled back to back at a
	 * random moment, the TX could start one frame (5.2 us) after the RX: an FS edge between the two enables, or a TX
	 * enabled before its DMA had the first block (F moved by 1 frame between boots, 2026-11-06). So wait until the TX
	 * DMA holds block 0 (it then waits for the SPORT to drain it), then enable both right after an LRCLK rising edge
	 * (DAI1_PB04, the DAC's frame sync), a full frame before the next frame start. */
	{
		ADI_SPORT_RESULT eResultTx;
		int txReady = Wait_Tx_Dma_Ready();
		int synced;

		adi_osal_EnterCriticalRegion();
		synced = Wait_Lrclk_Rise();
		eResult   = adi_sport_Enable(hSPORTDev4BRx,true);
		eResultTx = adi_sport_Enable(hSPORTDev4ATx,true);
		adi_osal_ExitCriticalRegion();
		CHECK_SPORT_STEP("adi_sport_Enable(4B Rx)", eResult);
		CHECK_SPORT_STEP("adi_sport_Enable(4A Tx)", eResultTx);
		if (!txReady || !synced)
		{
			printf("  [WARN] SPORT start not synchronized (TX DMA ready %d, LRCLK edge %d): TX may start a frame late\n",
				   txReady, synced);
		}
	}

	printf("  [OK] SPORT 4A TX & 4B RX configured and enabled on the same frame\n");
	fflush(stdout);

	return eResult;

}

#if (ADI_CONFIG_CONTINUOUS_AUDIO == 0)
static int Sport_Stop(void)
{
    /* SPORT return code */
    ADI_SPORT_RESULT    eResult;

    /*Stop the DMA transfer of  Sport Device 4B */
	eResult = adi_sport_StopDMATransfer(hSPORTDev4BRx);
	CHECK_RESULT(eResult);
	/*Stop the DMA transfer of  Sport Device 4A */
	eResult = adi_sport_StopDMATransfer(hSPORTDev4ATx);
	CHECK_RESULT(eResult);
	/*Close Sport Device 4B */
	eResult = adi_sport_Close(hSPORTDev4BRx);
	CHECK_RESULT(eResult);
	/*Close Sport Device 4A */
	eResult = adi_sport_Close(hSPORTDev4ATx);
	CHECK_RESULT(eResult);

	return eResult;
}
#endif

void Write_TWI_8bit_Reg(unsigned char Reg_ID, unsigned char Tx_Data)
{
	devBuffer[0] = Reg_ID;
	devBuffer[1] = Tx_Data;
	adi_twi_Write(hTwiDevice, devBuffer, 2u, false);
}

unsigned char Read_TWI_8bit_Reg(unsigned char Reg_ID)
{
	ADI_TWI_RESULT eResult;
	unsigned char Rx_Data;

	/* write register address */
	devBuffer[0] = Reg_ID;
	eResult = adi_twi_Write(hTwiDevice, devBuffer, 1u, true);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI write failed 0x%08X\n", eResult);
	}

	/* read register value */
	eResult = adi_twi_Read(hTwiDevice, &Rx_Data, 1u, false);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Read failed 0x%08X\n", eResult);
	}

	return Rx_Data;
}

static int Init_TWI(void)
{

	ADI_TWI_RESULT eResult;

	eResult = adi_twi_Open(TWIDEVNUM, ADI_TWI_MASTER, &TwideviceMemory[0],
			   	ADI_TWI_MEMORY_SIZE, &hTwiDevice);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Open failed 0x%08X\n", eResult);
	}

	eResult = adi_twi_SetPrescale(hTwiDevice, PRESCALEVALUE);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Set Prescale failed 0x%08X\n", eResult);
	}

	eResult = adi_twi_SetBitRate(hTwiDevice, BITRATE);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Set Bitrate failed 0x%08X\n", eResult);
	}


	eResult = adi_twi_SetDutyCycle(hTwiDevice, DUTYCYCLE);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Set Duty cycle failed 0x%08X\n", eResult);
	}

	eResult = adi_twi_SetHardwareAddress(hTwiDevice, TARGETADDR);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Set Hw address failed 0x%08X\n", eResult);
	}

	return eResult;
}

static int Stop_TWI(void)
{
	ADI_TWI_RESULT eResult;

	eResult = adi_twi_Close(hTwiDevice);
	CHECK_RESULT(eResult);

	return eResult;
}

/*****************************************************************************************************************************/

static int ADAU_1962_init(void)
{	int i;
	ADAU_1962_Pllinit();
	for(i=0;i<28;i++)
	{
		/* write value */
		Write_TWI_8bit_Reg(Config_array_DAC[i].Reg_Add,Config_array_DAC[i].Value);
		Config_read_DAC[i]=Read_TWI_8bit_Reg(Config_array_DAC[i].Reg_Add);
		if(Config_array_DAC[i].Value!= Config_read_DAC[i])
		{
			printf("  [ERROR] ADAU_1962 failed at idx %d (reg 0x%02X): wrote 0x%02X, read back 0x%02X\n",
				   i, Config_array_DAC[i].Reg_Add, (unsigned char)Config_array_DAC[i].Value, (unsigned char)Config_read_DAC[i]);
			fflush(stdout);
			return FAILED;
		}
	}
	printf("  [OK] ADAU_1962 configured successfully (all 28 registers verified)\n");
	fflush(stdout);

	return SUCCESS;
}

static int ADAU_1962_Pllinit(void)
{
	int status,delay1=0xffff;
	int timeout = 500000;

	ADI_TWI_RESULT eResult;

   	eResult = adi_twi_SetHardwareAddress(hTwiDevice, TARGETADDR_1962);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Set Hw address failed 0x%08X\n", eResult);
	}

	Write_TWI_8bit_Reg(ADAU1962_PLL_CTL_CTRL0,0x01);
	while(delay1--)
	{
		asm("nop;");
	}

	Write_TWI_8bit_Reg(ADAU1962_PLL_CTL_CTRL0,0x05);

	delay1=0xffff;
	while(delay1--)
	{
		asm("nop;");
	}

	Write_TWI_8bit_Reg(ADAU1962_PLL_CTL_CTRL1,0x2A); /* Bit 3=1: Enable DAC Automute on PLL Unlock; Bit 1=1: VREF ON */
	delay1=0xffff;
	while(delay1--)
	{
		asm("nop;");
	}
	status=Read_TWI_8bit_Reg(ADAU1962_PLL_CTL_CTRL1);
	while(!((status & 0x4)>>2) && --timeout)
	{
		status=Read_TWI_8bit_Reg(ADAU1962_PLL_CTL_CTRL1);
	}

	if (timeout == 0) {
		printf("  [WARN] ADAU_1962 PLL lock TIMEOUT! status=0x%02X\n", status);
	} else {
		printf("  ADAU_1962 PLL locked OK (status=0x%02X)\n", status);
	}
	fflush(stdout);

	return eResult;
}




static int ADAU_1979_init(void)
{	int i;

	ADAU_1979_Pllinit();

	for(i=0;i<16;i++)
	{
		Write_TWI_8bit_Reg(Config_array_ADC[i].Reg_Add,Config_array_ADC[i].Value);
		Config_read_ADC[i]=Read_TWI_8bit_Reg(Config_array_ADC[i].Reg_Add);
		if(Config_array_ADC[i].Value!= Config_read_ADC[i])
		{
			printf("  [ERROR] ADAU_1979 failed at idx %d (reg 0x%02X): wrote 0x%02X, read back 0x%02X\n",
				   i, Config_array_ADC[i].Reg_Add, (unsigned char)Config_array_ADC[i].Value, (unsigned char)Config_read_ADC[i]);
			fflush(stdout);
			return FAILED;
		}
	}
	printf("  [OK] ADAU_1979 configured successfully (all 16 registers verified)\n");
	fflush(stdout);

	return SUCCESS;
}

static int ADAU_1979_Pllinit(void)
{

	int status,delay1=0xffff;
	int timeout = 500000;

	ADI_TWI_RESULT eResult;

   	eResult = adi_twi_SetHardwareAddress(hTwiDevice, TARGETADDR_1979);
	if(eResult!=ADI_TWI_SUCCESS)
	{
		REPORT_ERROR("TWI Set Hw address failed 0x%08X\n", eResult);
	}

   	Write_TWI_8bit_Reg(ADAU1979_REG_POWER,0x01);
	Write_TWI_8bit_Reg(ADAU1979_REG_PLL,0x43); /* 0x40 (PLL_MUTE enabled) + 0x03 (MCS=011 for 192kHz) */
	status=Read_TWI_8bit_Reg(ADAU1979_REG_PLL);
	while(delay1--)
	{
		asm("nop;");
	}
	while(!((status & 0x80)>>7) && --timeout)
	{
		status=Read_TWI_8bit_Reg(ADAU1979_REG_PLL);
		asm("nop;");
	}

	if (timeout == 0) {
		printf("  [WARN] ADAU_1979 PLL lock TIMEOUT! status=0x%02X\n", status);
	} else {
		printf("  ADAU_1979 PLL locked OK (status=0x%02X)\n", status);
	}
	fflush(stdout);

	return eResult;

}

/*
 * Main function
 */
int main(int argc, char *argv[])
{
	/**
	 * Initialize managed drivers and/or services that have been added to
	 * the project.
	 * @return zero on success
	 */

	uint32_t Result=0;

	adi_initComponents();
#if defined(__ADSPARM__)
	adi_core_enable(ADI_CORE_SHARC0);
	#if (__NUM_SHARC_CORES__ == 2)
	adi_core_enable(ADI_CORE_SHARC1);
	#endif
#elif (__NUM_ARM_CORES__ == 0)
	adi_core_enable(ADI_CORE_SHARC1);
#endif

	printf("\n========================================\n");
	printf("=== SC598 ANC Talkthrough Diagnostic ===\n");
	printf("========================================\n");
	fflush(stdout);

    /* SPU initialization */
	if (Result==0u)
	{
		Result=SPU_init();
		printf("[Step 1] SPU_init Result = %u\n", Result);
		fflush(stdout);
	}

    /* Switch Configuration */
	Switch_Configurator();
	printf("[Step 2] Switch_Configurator Done\n");
	fflush(stdout);

	/* SRU Configuration */
	SRU_Init();
	printf("[Step 3] SRU_Init Done\n");
	fflush(stdout);

	/* TWI Initialization */
	if (Result==0u)
	{
		Result=Init_TWI();
		printf("[Step 4] Init_TWI Result = %u\n", Result);
		fflush(stdout);
	}

	/* ADAU1962 Initialization */
	if (Result==0u)
	{
		printf("[Step 5] Configuring ADAU1962 DAC...\n");
		fflush(stdout);
		Result=ADAU_1962_init();
	}

	/* ADAU1979 Initialization */
	if (Result==0u)
	{
		printf("[Step 6] Configuring ADAU1979 ADC...\n");
		fflush(stdout);
		Result=ADAU_1979_init();
	}

#if (APP_MODE == APP_MODE_ANC)
	/* ANC Algorithm Initialization */
	ANC_Init();
	printf("[Step 7] ANC Algorithm Initialized\n");
	fflush(stdout);
#elif (APP_MODE == APP_MODE_DIAG_DAC_ID)
	Id_Tone_Init();
#endif

	/* SPORT Initialization */
	if (Result==0u)
	{
		printf("[Step 8] Configuring SPORT 4A/4B (TDM4 mode)...\n");
		fflush(stdout);
		Result=Sport_Init();
	}

	/* Close TWI */
	if (Result==0u)
	{
		Result=Stop_TWI();
		printf("[Step 9] TWI Stopped OK\n");
		fflush(stdout);
	}

	if (Result == 0u)
	{
		printf("\n=======================================================\n");
		printf(">>> SYSTEM READY: Running audio streaming talkthrough <<<\n");
#if (APP_MODE == APP_MODE_ANC)
		printf(">>> MODE: ANC (FxNLMS, 48 kHz core, codec 192 kHz)\n");
		printf(">>> Output Jack J18:    Left = Mute, Right = Anti-noise y(n)\n");
		printf(">>> Output Jack J4/J17: Left = Err Mic, Right = Ref Mic\n");
#elif (APP_MODE == APP_MODE_PATH_MEASURE)
		printf(">>> MODE: Path Measurement Bridge for tools/pathmeas.py (through the 48 kHz core boundary)\n");
		printf(">>> Input Jack J12:  Slot %d = Ref Mic, Slot %d = Error Mic\n", J12_REF_MIC_SLOT, J12_ERR_MIC_SLOT);
		printf(">>> Input Jack J16:  Slot %d = Sweep (R), Slot %d = Timing Ref (L)\n", J16_SWEEP_SLOT, J16_CHIRP_SLOT);
		printf(">>> Output Jack J18: Left = Mute, Right = Sweep to Speaker Amp (Gain = %dx)\n", CALIBRATION_SWEEP_GAIN);
		printf(">>> Output Jack J17: Left (Tip) = Err + Ref, Right (Ring) = Ref Mic + Ref (Ref Gain = %.4fx)\n", CALIBRATION_CHIRP_GAIN);
		printf(">>> PC: pathmeas.py measure --path S with J17 Left in the US205 mic input, --path F with J17 Right\n");
#elif (APP_MODE == APP_MODE_DIAG_MIC)
		printf(">>> MODE: DIAG Mic Passthrough\n");
		printf(">>> Output Jack J18:    Left = Mute, Right = Ref Mic\n");
		printf(">>> Output Jack J4/J17: Left = Err Mic, Right = Ref Mic\n");
#elif (APP_MODE == APP_MODE_DIAG_LINE_IN)
		printf(">>> MODE: DIAG J16 Line-In Check\n");
		printf(">>> J18 & J4/J17 Left  <- J16 Left  (Slot %d)\n", J16_CHIRP_SLOT);
		printf(">>> J18 & J4/J17 Right <- J16 Right (Slot %d)\n", J16_SWEEP_SLOT);
#elif (APP_MODE == APP_MODE_DIAG_DAC_ID)
		printf(">>> MODE: DIAG DAC Slot ID Beeps (1 kHz, -20 dBFS, every 2 s)\n");
		printf(">>> Slot 0 = 1 beep, Slot 1 = 2 beeps, Slot 2 = 3 beeps, Slot 3 = 4 beeps\n");
		printf(">>> Expected: J18 L/R = 1/2 beeps, J4/J17 L/R = 3/4 beeps\n");
#endif
		printf("=======================================================\n\n");
		fflush(stdout);
	}
	else
	{
		printf("\n>>> INITIALIZATION FAILED: Error code %u <<<\n\n", Result);
		fflush(stdout);
	}

#if (ADI_CONFIG_CONTINUOUS_AUDIO==0)

	while(1)
	{
		if(TestCallbackCount == 8000)
		{
			int delay=0xffffff;
			while(delay--)
			{
				asm("nop;");
			}

			if (Result==0u)
			{
				Result=Sport_Stop();
			}

			if(Result==0u)
			{
				DEBUG_INFORMATION("All done\n");
			}
			else
			{
				DEBUG_INFORMATION("Example failed.\n");
			}

			break;
		}
	}
#elif (ADI_CONFIG_CONTINUOUS_AUDIO==1)
#if ENABLE_ADC_CAPTURE
	/* Skip ~1 s of start-up, take a fresh capture, then write it to the host PC over JTAG
	 * (one sample per line: decimal and hex). Audio glitches while writing; capture is already done. */
	{
		uint32_t t0 = TestCallbackCount;
		uint32_t n;
		FILE *fp;
		while ((uint32_t)(TestCallbackCount - t0) < CALLBACKS_PER_S) {}
		g_AdcCaptureIdx = 0;
		g_AdcCaptureDone = 0;
		while (!g_AdcCaptureDone) {}
		fp = fopen(ADC_CAPTURE_FILE, "w");
		if (fp != NULL)
		{
			for(n=0; n<ADC_CAPTURE_LEN; n++)
			{
				fprintf(fp, "%d\t0x%08X\n", g_AdcCapture[n], (unsigned int)g_AdcCapture[n]);
			}
			fclose(fp);
			printf("[CAPTURE] wrote %u samples of slot %d to %s\n", ADC_CAPTURE_LEN, ADC_CAPTURE_SLOT, ADC_CAPTURE_FILE);
		}
		else
		{
			printf("[CAPTURE] fopen failed: %s\n", ADC_CAPTURE_FILE);
		}
		fflush(stdout);
	}
#endif
#if ENABLE_SLOT_PEAK_METER || ENABLE_CYCLE_METER
	{
		uint32_t lastCount = TestCallbackCount;
		int s, peak[4];
#if ENABLE_CYCLE_METER
		cycle_t lastStamp;
		START_CYCLE_COUNT(lastStamp);
#endif
		while(1)
		{
			if ((uint32_t)(TestCallbackCount - lastCount) >= PEAK_METER_CALLBACKS)
			{
				lastCount = TestCallbackCount;
#if ENABLE_CYCLE_METER
				{
					uint32_t cmax = g_CycleMax, csum = g_CycleSum, cn = g_CycleBlocks;
					float cavg, period;
					cycle_t elapsed;
					STOP_CYCLE_COUNT(elapsed, lastStamp);  /* core cycles since last print */
					START_CYCLE_COUNT(lastStamp);
					/* Measured block period in core cycles; should equal BLOCK_BUDGET_CYCLES if CORE_CLOCK_HZ is right */
					period = (cn > 0u) ? ((float)elapsed / (float)cn) : 0.0f;
					g_CycleMax = 0;
					g_CycleSum = 0;
					g_CycleBlocks = 0;
					cavg = (cn > 0u) ? ((float)csum / (float)cn) : 0.0f;
					/* Line 1: #n      = TestCallbackCount at print time
					 *         blocks  = blocks timed since last print
					 *         avg/max = mean / slowest Process_Block_Body() cycles, shown as us and % of budget
					 *         budget  = BLOCK_FRAMES frames @ 192 kHz in core cycles
					 * Line 2: period  = measured cycles per block over the interval
					 *         load    = avg / period, the true load without assuming CORE_CLOCK_HZ */
					printf("[CYCLES #%u] blocks=%u avg=%.0f (%.1f us, %.1f%%) max=%u (%.1f us, %.1f%%) budget=%.0f\n",
						   lastCount, cn,
						   cavg, cavg / CORE_CLOCK_HZ * 1e6f, 100.0f * cavg / BLOCK_BUDGET_CYCLES,
						   cmax, (float)cmax / CORE_CLOCK_HZ * 1e6f, 100.0f * (float)cmax / BLOCK_BUDGET_CYCLES,
						   BLOCK_BUDGET_CYCLES);
					printf("[CYCLES #%u] measured block period=%.0f cycles (load avg %.1f%%)\n",
						   lastCount, period, (period > 0.0f) ? (100.0f * cavg / period) : 0.0f);
					fflush(stdout);
				}
#endif
#if ENABLE_SLOT_PEAK_METER
				for(s=0; s<4; s++)
				{
					peak[s] = g_SlotPeak[s];
					g_SlotPeak[s] = 0;
				}
				printf("[PEAK dBFS #%u] S0=%6.1f  S1=%6.1f  S2=%6.1f  S3=%6.1f\n",
					   lastCount,
					   20.0f*log10f(((float)peak[0] + 1.0f) / 2147483648.0f),
					   20.0f*log10f(((float)peak[1] + 1.0f) / 2147483648.0f),
					   20.0f*log10f(((float)peak[2] + 1.0f) / 2147483648.0f),
					   20.0f*log10f(((float)peak[3] + 1.0f) / 2147483648.0f));
				fflush(stdout);
#endif
			}
		}
	}
#else
	while(1);
#endif
#endif
}

