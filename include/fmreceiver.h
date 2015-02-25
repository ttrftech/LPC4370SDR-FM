/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef __FMRECEIVER_H__
#define __FMRECEIVER_H__

#include <arm_math.h>
#include <lpc43xx_gpio.h>

#define STEREO	1

#define AUDIO_RATE			48000
#define IF_RATE				(13 * AUDIO_RATE / 2)
#define CIC_DECIMATION_RATIO	16
#define FIR_DECIMATION_RATIO	2
#define ADC_RATE			(CIC_DECIMATION_RATIO * FIR_DECIMATION_RATIO * IF_RATE)


#define CAPTUREBUFFER_SIZE	0x10000
#define CAPTUREBUFFER0		((uint8_t*)0x20000000)
#define CAPTUREBUFFER1		((uint8_t*)0x20008000)
#define CAPTUREBUFFER_SIZEHALF	0x8000

#define NCO_SIN_TABLE		((int16_t*)0x1008F000)
#define NCO_COS_TABLE		((int16_t*)0x1008F800)
#define NCO_TABLE_SIZE		0x800
#define NCO_SAMPLES			1024
//#define NCO_AMPL			32
//#define NCO_AMPL			64
#define NCO_AMPL			(SHRT_MAX / 128)
//#define NCO_AMPL			(SHRT_MAX / 64)
//#define NCO_AMPL			(SHRT_MAX / 32)
//#define NCO_AMPL			(SHRT_MAX / 16)
//#define NCO_AMPL			(SHRT_MAX / 4)

#define I_FIR_STATE			((q15_t*)0x10080000)
#define I_FIR_BUFFER		((q15_t*)0x10080040)
#define Q_FIR_STATE			((q15_t*)0x10081000)
#define Q_FIR_BUFFER		((q15_t*)0x10081040)
/*  0x10000 / 2 / 16 */
#define FIR_BUFFER_SIZE		0x800
#define FIR_STATE_SIZE		0x40
#define FIR_GAINBITS		5	/* 0 ~ 6 */

#define DEMOD_BUFFER 		((q15_t*)0x10088000)
#define DEMOD_BUFFER_SIZE	0x800
//#define DEMOD_GAINBITS		6	/* 0 ~ 6 */
#define DEMOD_GAINBITS		9	/* 0 ~ 10 */

#define RESAMPLE_STATE 		((q15_t*)0x10089000)
#define RESAMPLE_STATE_SIZE	0x100
#define RESAMPLE_BUFFER 	((q15_t*)0x10089100)
#define RESAMPLE_BUFFER_SIZE 0x400
#define RESAMPLE_GAINBITS	3	/* 0 ~ 6 */

#define RESAMPLE2_STATE 		((q15_t*)0x10089500)
#define RESAMPLE2_STATE_SIZE	0x100
#define RESAMPLE2_BUFFER 	((q15_t*)0x10089600)
#define RESAMPLE2_BUFFER_SIZE 0x400

#define AUDIO_BUFFER 		((q15_t*)0x1008A000)
#define AUDIO_BUFFER_SIZE	0x2000
#define AUDIO_TEST_BUFFER 	((q15_t*)0x1008C000)



// dsp.c
extern void DMA_IRQHandler(void);
extern void nco_set_frequency(float32_t freq);
extern void generate_test_tone(int freq);
extern void dsp_init();

extern void update_adc_dc_offset(void);
extern void audio_set_gain(int gain);

#define AUDIO_GAIN_MAX 29
#define AUDIO_GAIN_MIN -7
#define AUDIO_GAIN_REF 7

// ui.c
extern void ui_init();
extern void ui_process();

// clkcfg.h
extern void setup_systemclock();
extern void setup_pll0audio(uint32_t msel, uint32_t nsel, uint32_t psel);
extern void setup_i2s_clock(LPC_I2Sn_Type *I2Sx, uint32_t Freq, uint8_t TRMode);


extern volatile int32_t capture_count;


typedef struct {
	uint16_t write_current;
	uint16_t write_total;
	uint16_t read_total;
	uint16_t read_current;
	uint16_t rebuffer_count;
} audio_state_t;

typedef struct {
	uint32_t last;
	int32_t carrier;
} fm_demod_state_t;

typedef struct {
	float32_t carrier_i;
	float32_t carrier_q;
	float32_t step_cos;
	float32_t step_sin;
	float32_t basestep_cos;
	float32_t basestep_sin;
	float32_t delta_cos[12];
	float32_t delta_sin[12];
	int16_t corr;
	int16_t corr_ave;
	int16_t corr_std;
	int32_t sdi;
	int32_t sdq;
} stereo_separate_state_t;

typedef struct {
	q15_t *dest;
	int16_t *nco_base;
	int32_t dest_idx;
	int32_t s0;
	int32_t s1;
	int32_t s2;
	int32_t d0;
	int32_t d1;
	int32_t d2;
	uint32_t dc_offset;
} cic_state_t;

extern cic_state_t cic_i;
extern cic_state_t cic_q;



#define LED_INIT()	     (LPC_GPIO_PORT->DIR[0] |= (1UL << 8))
#define LED_ON()		 (LPC_GPIO_PORT->SET[0] |= (1UL << 8))
#define LED_OFF()		 (LPC_GPIO_PORT->CLR[0] = (1UL << 8))
#define LED_TOGGLE()	 (LPC_GPIO_PORT->NOT[0] = (1UL << 8))

#define ROTLED_INIT()	     (LPC_GPIO_PORT->DIR[1] |= (1UL << 3)|(1UL << 4))
#define ROTLED_RED()		 do{LPC_GPIO_PORT->SET[1] |= (1UL << 3);LPC_GPIO_PORT->CLR[1] = (1UL << 4);}while(0)
#define ROTLED_GREEN()		 do{LPC_GPIO_PORT->SET[1] |= (1UL << 4);LPC_GPIO_PORT->CLR[1] = (1UL << 3);}while(0)
#define ROTLED_OFF()		 (LPC_GPIO_PORT->CLR[1] = (1UL << 3)|(1UL << 4))

#define TESTPOINT_INIT() \
do {scu_pinmux(0x6, 11, PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW | FILTER_ENABLE, FUNC0); \
	LPC_GPIO_PORT->DIR[3] |= (1UL << 7); \
	LPC_GPIO_PORT->SET[3] |= (1UL << 7); } while(0)
#define TESTPOINT_ON() 	(LPC_GPIO_PORT->SET[3] |= (1UL << 7))
#define TESTPOINT_OFF()	(LPC_GPIO_PORT->CLR[3] = (1UL << 7))
#define TESTPOINT_TOGGLE()	(LPC_GPIO_PORT->NOT[3] = (1UL << 7))
#define TESTPOINT_SPIKE()	TESTPOINT_TOGGLE();TESTPOINT_TOGGLE()

#define DMA_HALT()	(LPC_GPDMA->C0CONFIG |= (1 << 18))
#define DMA_RUN()	(LPC_GPDMA->C0CONFIG &= ~(1 << 18))

#endif /* __FMRECEIVER_H__ */
