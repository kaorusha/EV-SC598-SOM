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
#include "ADAU_1962Common.h"
#include "ADAU_1979Common.h"
#include "math.h"
#include <SRU.h>
#include "Audio_Talkthrough_I2S.h"
#include "anc_fxnlms.h"

/* ANC Enable Switch: 1 = Active ANC, 0 = Bypass (Pure Audio Talkthrough) */
#define ANC_ENABLE 0

/* 
 * Bypass Talkthrough Mode (when ANC_ENABLE == 0):
 * 1 = Full Stereo Talkthrough (Left = ADC[0], Right = ADC[1])
 *     -> Use this mode to test music! Both Left and Right will bypass music and mics!
 * 0 = Speaker Standby Mode (Left = Mute 0, Right = Ref Mic)
 *     -> Left speaker is muted, Right speaker plays Ref Mic (ready for ANC).
 */
#define TALKTHROUGH_MODE 1

/* Digital gain multiplier for bypass talkthrough listening (1 = 1x unity, 2 = 2x/+6dB, 4 = 4x/+12dB) */
#define TALKTHROUGH_GAIN 1

/* Enable Console logging inside SPORT callback (0 = disabled for clickless audio, 1 = debug log) */
#define ENABLE_CALLBACK_PRINTF 0

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
		   	    {     ADAU1962_DAC_CTRL1,		0x01},
		   	    {     ADAU1962_DAC_CTRL2,		0x00},
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
				{     ADAU1962_DAC_CTRL0,		0x10}  /* TDM4 mode (48kHz, 4ch on DSDATA1: DAC 1/2=J4/J17, DAC 3/4=J18) */

};

struct Config_Table Config_array_ADC[16] = {
		 {ADAU1979_REG_BOOST			,	0x00},
		 {ADAU1979_REG_MICBIAS			,	0x00},
		 {ADAU1979_REG_BLOCK_POWER_SAI	,	0x30},
		 {ADAU1979_REG_SAI_CTRL0		,	0x13}, /* TDM4 48kHz (sync with DAC TDM4 BCLK) */
		 {ADAU1979_REG_SAI_CTRL1		,	0x08}, /* 32-bit slot, Slave to DAC clock */
		 {ADAU1979_REG_CMAP12			,	0x01},
		 {ADAU1979_REG_CMAP34			,	0x23},
		 {ADAU1979_REG_SAI_OVERTEMP		,	0xf0},
		 {ADAU1979_REG_POST_ADC_GAIN1 	,	0xA0},
		 {ADAU1979_REG_POST_ADC_GAIN2 	,	0xA0},
		 {ADAU1979_REG_POST_ADC_GAIN3 	,	0xA0},
		 {ADAU1979_REG_POST_ADC_GAIN4 	,	0xA0},
		 {ADAU1979_REG_ADC_CLIP			,	0x00},
		 {ADAU1979_REG_DC_HPF_CAL		,	0x00},
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

/* Destination Sport PDMA list */
ADI_PDMA_DESC_LIST iDESC_LIST_1_SP4A;
ADI_PDMA_DESC_LIST iDESC_LIST_2_SP4A;

/* Source Sport PDMA list */
ADI_PDMA_DESC_LIST iSRC_LIST_1_SP4B;
ADI_PDMA_DESC_LIST iSRC_LIST_2_SP4B;


/* Count to track the number of callBacks for SPORT transfer */
volatile uint8_t CallbackCount = 0;

volatile uint32_t TestCallbackCount = 0;

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


static void SPORTCallback(
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg
)
{
	int i, j;
	static uint32_t s_cbCount = 0;

    /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
        		TestCallbackCount +=1;
        		CallbackCount +=1;
        		s_cbCount++;
#if ENABLE_CALLBACK_PRINTF
        		if (s_cbCount == 1 || s_cbCount == 100 || (s_cbCount % 5000 == 0))
        		{
        			printf("[SPORTCallback #%u] ADC: [0]=0x%08X [1]=0x%08X [2]=0x%08X [3]=0x%08X | DAC: [0]=0x%08X [1]=0x%08X [2]=0x%08X [3]=0x%08X\n",
        				   s_cbCount,
        				   int_SP0ABuffer4[0], int_SP0ABuffer4[1], int_SP0ABuffer4[2], int_SP0ABuffer4[3],
        				   int_SP0ABuffer1[0], int_SP0ABuffer1[1], int_SP0ABuffer1[2], int_SP0ABuffer1[3]);
        			fflush(stdout);
        		}
#endif
        		if(CallbackCount==1)
        		{
#if ANC_ENABLE
        		   ANC_Process_Block(int_SP0ABuffer4, int_SP0ABuffer1, COUNT);
#else
        		   for(i=0; i<COUNT; i+=4)
        		   {
#if (TALKTHROUGH_MODE == 1)
        			   /* Full Stereo Bypass Test Mode: Left = ADC0 (Err/Music L), Right = ADC1 (Ref/Music R) */
        			   int_SP0ABuffer1[i+0] = int_SP0ABuffer4[i+0] * TALKTHROUGH_GAIN; /* J17 Left  (Tip)  */
        			   int_SP0ABuffer1[i+1] = int_SP0ABuffer4[i+1] * TALKTHROUGH_GAIN; /* J17 Right (Ring) */
#else
        			   /* ANC Speaker Standby Mode: Left = 0 (Mute), Right = Ref Mic (ready for ANC) */
        			   int_SP0ABuffer1[i+0] = 0;                                       /* J17 Left  (Tip): Mute */
        			   int_SP0ABuffer1[i+1] = int_SP0ABuffer4[i+1] * TALKTHROUGH_GAIN; /* J17 Right (Ring): Ref Mic */
#endif
        			   /* Monitor Out: Broadcast to J18 (DAC 3/4) */
        			   int_SP0ABuffer1[i+2] = int_SP0ABuffer4[i+0];                    /* J18 Left  (Tip):  Error Mic */
        			   int_SP0ABuffer1[i+3] = int_SP0ABuffer4[i+1];                    /* J18 Right (Ring): Ref Mic */
        		   }
#endif
        		}

        		if(CallbackCount==2)
				{
#if ANC_ENABLE
        		   ANC_Process_Block(int_SP0ABuffer5, int_SP0ABuffer2, COUNT);
#else
				   for(i=0; i<COUNT; i+=4)
				   {
#if (TALKTHROUGH_MODE == 1)
					   /* Full Stereo Bypass Test Mode: Left = ADC0 (Err/Music L), Right = ADC1 (Ref/Music R) */
					   int_SP0ABuffer2[i+0] = int_SP0ABuffer5[i+0] * TALKTHROUGH_GAIN; /* J17 Left  (Tip)  */
					   int_SP0ABuffer2[i+1] = int_SP0ABuffer5[i+1] * TALKTHROUGH_GAIN; /* J17 Right (Ring) */
#else
					   /* ANC Speaker Standby Mode: Left = 0 (Mute), Right = Ref Mic (ready for ANC) */
					   int_SP0ABuffer2[i+0] = 0;                                       /* J17 Left  (Tip): Mute */
					   int_SP0ABuffer2[i+1] = int_SP0ABuffer5[i+1] * TALKTHROUGH_GAIN; /* J17 Right (Ring): Ref Mic */
#endif
					   /* Monitor Out: Broadcast to J18 (DAC 3/4) */
					   int_SP0ABuffer2[i+2] = int_SP0ABuffer5[i+0];                    /* J18 Left  (Tip):  Error Mic */
					   int_SP0ABuffer2[i+3] = int_SP0ABuffer5[i+1];                    /* J18 Right (Ring): Ref Mic */
				   }
#endif
				   CallbackCount=0;
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

static void PrepareDescriptors (void)
{
#if defined(__ADSPCORTEXA55__)
	iDESC_LIST_1_SP4A.pStartAddr	= (uint32_t)(uintptr_t)(int *)int_SP0ABuffer1;
	iDESC_LIST_1_SP4A.pNxtDscp		= (uint32_t)(uintptr_t)&iDESC_LIST_2_SP4A;
#else
	iDESC_LIST_1_SP4A.pStartAddr	= (int *)int_SP0ABuffer1;
	iDESC_LIST_1_SP4A.pNxtDscp		= &iDESC_LIST_2_SP4A;
#endif
	iDESC_LIST_1_SP4A.Config		= ENUM_DMA_CFG_XCNT_INT ;
	iDESC_LIST_1_SP4A.XCount		= COUNT;
	iDESC_LIST_1_SP4A.XModify		= 4;
	iDESC_LIST_1_SP4A.YCount		= 0;
	iDESC_LIST_1_SP4A.YModify		= 0;

#if defined(__ADSPCORTEXA55__)
	iDESC_LIST_2_SP4A.pStartAddr	= (uint32_t)(uintptr_t)(int *)int_SP0ABuffer2;
	iDESC_LIST_2_SP4A.pNxtDscp		= (uint32_t)(uintptr_t)&iDESC_LIST_1_SP4A;
#else
	iDESC_LIST_2_SP4A.pStartAddr	= (int *)int_SP0ABuffer2;
	iDESC_LIST_2_SP4A.pNxtDscp		= &iDESC_LIST_1_SP4A;
#endif
	iDESC_LIST_2_SP4A.Config		= ENUM_DMA_CFG_XCNT_INT ;
	iDESC_LIST_2_SP4A.XCount		= COUNT;
	iDESC_LIST_2_SP4A.XModify		= 4;
	iDESC_LIST_2_SP4A.YCount		= 0;
	iDESC_LIST_2_SP4A.YModify		= 0;

#if defined(__ADSPCORTEXA55__)
	iSRC_LIST_1_SP4B.pStartAddr		= (uint32_t)(uintptr_t)(int *)int_SP0ABuffer4;
	iSRC_LIST_1_SP4B.pNxtDscp		= (uint32_t)(uintptr_t)&iSRC_LIST_2_SP4B;
#else
	iSRC_LIST_1_SP4B.pStartAddr		=(int *)int_SP0ABuffer4;
	iSRC_LIST_1_SP4B.pNxtDscp		= &iSRC_LIST_2_SP4B;
#endif
	iSRC_LIST_1_SP4B.Config			= ENUM_DMA_CFG_XCNT_INT ;
	iSRC_LIST_1_SP4B.XCount			= COUNT;
	iSRC_LIST_1_SP4B.XModify		= 4;
	iSRC_LIST_1_SP4B.YCount			= 0;
	iSRC_LIST_1_SP4B.YModify		= 0;

#if defined(__ADSPCORTEXA55__)
	iSRC_LIST_2_SP4B.pStartAddr		= (uint32_t)(uintptr_t)(int *)int_SP0ABuffer5;
	iSRC_LIST_2_SP4B.pNxtDscp		= (uint32_t)(uintptr_t)&iSRC_LIST_1_SP4B;
#else
	iSRC_LIST_2_SP4B.pStartAddr		=(int *)int_SP0ABuffer5;
	iSRC_LIST_2_SP4B.pNxtDscp		= &iSRC_LIST_1_SP4B;
#endif
	iSRC_LIST_2_SP4B.Config			= ENUM_DMA_CFG_XCNT_INT;
	iSRC_LIST_2_SP4B.XCount			= COUNT;
	iSRC_LIST_2_SP4B.XModify		= 4;
	iSRC_LIST_2_SP4B.YCount			= 0;
	iSRC_LIST_2_SP4B.YModify		= 0;

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
	eResult = adi_sport_ConfigClock(hSPORTDev4BRx,32,false,false,false);
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
	eResult = adi_sport_DMATransfer(hSPORTDev4BRx,&iSRC_LIST_1_SP4B,(DMA_NUM_DESC),ADI_PDMA_DESCRIPTOR_LIST, ADI_SPORT_CHANNEL_PRIM);
	CHECK_SPORT_STEP("adi_sport_DMATransfer(4B Rx)", eResult);
	/* Submit the first buffer for Tx.  */
	eResult = adi_sport_DMATransfer(hSPORTDev4ATx,&iDESC_LIST_1_SP4A,(DMA_NUM_DESC),ADI_PDMA_DESCRIPTOR_LIST, ADI_SPORT_CHANNEL_PRIM);
	CHECK_SPORT_STEP("adi_sport_DMATransfer(4A Tx)", eResult);

	/*Enable the Sport Device 4B */
	eResult = adi_sport_Enable(hSPORTDev4BRx,true);
	CHECK_SPORT_STEP("adi_sport_Enable(4B Rx)", eResult);
	/*Enable the Sport Device 4A */
	eResult = adi_sport_Enable(hSPORTDev4ATx,true);
	CHECK_SPORT_STEP("adi_sport_Enable(4A Tx)", eResult);

	printf("  [OK] SPORT 4A TX & 4B RX configured and enabled successfully\n");
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

	Write_TWI_8bit_Reg(ADAU1962_PLL_CTL_CTRL1,0x22);
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
	Write_TWI_8bit_Reg(ADAU1979_REG_PLL,0x03);
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

#if ANC_ENABLE
	/* ANC Algorithm Initialization */
	ANC_Init();
	printf("[Step 7] ANC Algorithm Initialized\n");
	fflush(stdout);
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
#if (TALKTHROUGH_MODE == 1)
		printf(">>> MODE: Full Stereo Bypass Test (Gain = %dx)\n", TALKTHROUGH_GAIN);
		printf(">>> Output Jack J17: Left=ADC[0] (Err/Music L), Right=ADC[1] (Ref/Music R)\n");
		printf(">>> Output Jack J18: Left=ADC[0] (Err/Music L), Right=ADC[1] (Ref/Music R)\n");
#else
		printf(">>> MODE: ANC Speaker Standby\n");
		printf(">>> Output Jack J17: Left=Mute(0), Right=Ref Mic\n");
		printf(">>> Output Jack J18: Left=Err Mic, Right=Ref Mic\n");
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
	while(1);
#endif
}

