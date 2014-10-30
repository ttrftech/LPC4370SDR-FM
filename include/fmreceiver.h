#ifndef __FMRECEIVER_H__
#define __FMRECEIVER_H__

#include <arm_math.h>
#include <lpc43xx_gpio.h>

#define AUDIO_RATE			48000
#define IF_RATE				(13 * AUDIO_RATE / 2)
#define DECIMATION_RATIO	32
#define ADC_RATE			(DECIMATION_RATIO * IF_RATE)


//#define CAPTUREBUFFER		((uint8_t*)0x20000000)
//#define CAPTUREBUFFER_SIZE	0x10000
//#define CAPTUREBUFFER		((uint8_t*)0x20008000)
#define CAPTUREBUFFER_SIZE	0x10000
#define CAPTUREBUFFER0		((uint8_t*)0x20000000)
#define CAPTUREBUFFER1		((uint8_t*)0x20008000)
#define CAPTUREBUFFER_SIZEHALF	0x8000

#define NCO_SIN_TABLE		((int16_t*)0x1008F000)
#define NCO_COS_TABLE		((int16_t*)0x1008F800)
#define NCO_TABLE_SIZE		0x800
#define NCO_SAMPLES			1024
//#define NCO_AMPL			32
#define NCO_AMPL			64
//#define NCO_AMPL			(SHRT_MAX / 128)
//#define NCO_AMPL			(SHRT_MAX / 64)
//#define NCO_AMPL			(SHRT_MAX / 32)
//#define NCO_AMPL			(SHRT_MAX / 16)
//#define NCO_AMPL			(SHRT_MAX / 4)

#define I_FIR_STATE			((q15_t*)0x10080000)
#define I_FIR_BUFFER		((q15_t*)0x10080040)
#define Q_FIR_STATE			((q15_t*)0x10084040)
#define Q_FIR_BUFFER		((q15_t*)0x10084080)
/*  0x10000 / 2 / 32 */
#define FIR_BUFFER_SIZE		0x400
#define FIR_STATE_SIZE		0x40
#define FIR_GAINBITS		4	/* 0 ~ 6 */

#define DEMOD_BUFFER 		((q15_t*)0x10088000)
#define DEMOD_BUFFER_SIZE	0x800
#define DEMOD_GAINBITS		6	/* 0 ~ 6 */

#define RESAMPLE_STATE 		((q15_t*)0x10089000)
#define RESAMPLE_STATE_SIZE	0x100
#define RESAMPLE_BUFFER 	((q15_t*)0x10089100)
#define RESAMPLE_BUFFER_SIZE 0x400
#define RESAMPLE_GAINBITS	5	/* 0 ~ 6 */

#define AUDIO_BUFFER 		((q15_t*)0x1008A000)
#define AUDIO_BUFFER_SIZE	0x2000
#define AUDIO_TEST_BUFFER 	((q15_t*)0x1008C000)



// dsp.c
extern void DMA_IRQHandler(void);
extern void nco_set_frequency(float32_t freq);
extern void generate_test_tone(int freq);
extern void dsp_init();

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

#define LED_INIT()	     (LPC_GPIO_PORT->DIR[0] |= (1UL << 8))
#define LED_ON()		 (LPC_GPIO_PORT->SET[0] |= (1UL << 8))
#define LED_OFF()		 (LPC_GPIO_PORT->CLR[0] = (1UL << 8))
#define LED_TOGGLE()	 (LPC_GPIO_PORT->NOT[0] = (1UL << 8))

#define TESTPOINT_INIT() \
do {scu_pinmux(0x6, 11, PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW | FILTER_ENABLE, FUNC0); \
	LPC_GPIO_PORT->DIR[3] |= (1UL << 7); \
	LPC_GPIO_PORT->SET[3] |= (1UL << 7); } while(0)
#define TESTPOINT_ON() 	(LPC_GPIO_PORT->SET[3] |= (1UL << 7))
#define TESTPOINT_OFF()	(LPC_GPIO_PORT->CLR[3] = (1UL << 7))
#define TESTPOINT_TOGGLE()	(LPC_GPIO_PORT->NOT[3] = (1UL << 7))
#define TESTPOINT_SPIKE()	TESTPOINT_TOGGLE();TESTPOINT_TOGGLE()

#endif /* __FMRECEIVER_H__ */
