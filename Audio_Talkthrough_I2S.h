/*********************************************************************************
Copyright(c) 2021 Analog Devices, Inc. All Rights Reserved.
This software is proprietary. By using this software you agree
to the terms of the associated Analog Devices License Agreement.
*********************************************************************************/

/*****************************************************************************
 * Audio_Talkthrough_I2S.h
 *****************************************************************************/

#ifndef __AUDIO_TALKTHROUGH_I2S_H__
#define __AUDIO_TALKTHROUGH_I2S_H__

/* Add your custom header content here */

#include <stdint.h>
/* add for vscode check*/
#include <sys/adi_core.h>

#ifndef ADI_CORE_SHARC1
#define ADI_CORE_SHARC1 ((ADI_CORE_ID)2)
#endif
/* end for vscode check*/

/* Enable this macro for continuous audio */
#define ADI_CONFIG_CONTINUOUS_AUDIO		1

/*
 * DMA Buffer Size (TDM4 Mode: 4 frames per buffer block):
 * COUNT = 16 (4 frames * 4 ADC/DAC channels) = one 48 kHz ANC core sample (anc_rate.h ANC_DECIM).
 * A core sample is taken at the last frame of a block and its output starts two blocks later on the wire,
 * so the DMA adds 5 frames (26 us) to the loop; 16-frame blocks added 32 frames (167 us).
 * Must stay a multiple of 4 * ANC_DECIM.
 */
#define COUNT 16

#define SPORT_DEVICE_4A 			4u			/* SPORT device number */
#define SPORT_DEVICE_4B 			4u			/* SPORT device number */
#define TWIDEVNUM     				2u         /* TWI device number */

/* Codec TWI (ADAU1962A, ADAU1979; the soft switches set their own 100 kHz). 400 kHz since 2026-11-24: the DAC1-12
 * power-down then lands about 0.08 ms after PUP instead of 0.3 ms (boot pop, DAC_POWER_ORDER 3). The driver counts in
 * 10 MHz ticks but the prescaler gives 9.375 MHz: 25 ticks = 375 kHz, SCL high 10 ticks (1.07 us), low 15 (1.6 us);
 * both codecs need low >= 1.3 us, high >= 0.6 us, so not 50 %. */
#define BITRATE       				(400u)      /* kHz */
#define DUTYCYCLE     				(40u)       /* percent of the period SCL is high */
#define PRESCALEVALUE 				(12u)       /* fSCLK/10MHz (112.5 sclk0_0) */
#define BUFFER_SIZE   				(8u)

#define TARGETADDR    				(0x38u)     /* hardware address */
#define TARGETADDR_1962    			(0x04u)     /* hardware address of adau1962 DAC */
#define TARGETADDR_1979    			(0x11u)     /* hardware address of adau1979 ADC */


#define SPORT_4A_SPU  					71
#define SPORT_4B_SPU   					72

/* Descriptors per DMA ring. The two buffers alternate, so 2 would do; 4 lets the DMA probe
 * (ENABLE_DMA_PROBE) tell four consecutive blocks apart. */
#define DMA_NUM_DESC 				4u

#define SUCCESS   0
#define FAILED   -1

#define CHECK_RESULT(eResult) \
        if(eResult != 0)\
		{\
			return (1);\
        }

#define REPORT_ERROR        	 printf
#define DEBUG_INFORMATION        printf


#endif /* __AUDIO_TALKTHROUGH_I2S_H__ */
