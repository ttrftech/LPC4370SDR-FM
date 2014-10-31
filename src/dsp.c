
#ifdef __USE_CMSIS
#include "LPC43xx.h"
#endif

#include <cr_section_macros.h>
#include <limits.h>
#include <arm_math.h>
#include "lpc43xx_i2s.h"
#include "fmreceiver.h"


#define NCO_CYCLE 1024
#define NCO_SAMPLES 1024
#define NCO_COS_OFFSET (NCO_CYCLE/4)

void nco_set_frequency(float32_t freq)
{
	int16_t *costbl = NCO_SIN_TABLE;
	int16_t *sintbl = NCO_COS_TABLE;
	int f;
	int i;

	freq -= (int)(freq / ADC_RATE) * ADC_RATE;
	f = (int)(freq / ADC_RATE * NCO_CYCLE);
	for (i = 0; i < NCO_SAMPLES; i++) {
		float32_t phase = 2*PI*f*(i+0.5)/NCO_CYCLE;
		costbl[i] = (int16_t)(arm_cos_f32(phase) * NCO_AMPL);
		sintbl[i] = (int16_t)(arm_sin_f32(phase) * NCO_AMPL);
	}
}

#define FIR_NUM_TAPS			32

q15_t fir_coeff[FIR_NUM_TAPS] = {
			-204,   -42,   328,   144,  -687,  -430,  1301,  1060, -2162,
		   -2298,  3208,  4691, -4150, -9707,  3106, 22273, 22273,  3106,
		   -9707, -4150,  4691,  3208, -2298, -2162,  1060,  1301,  -430,
			-687,   144,   328,   -42,  -204
};

typedef struct {
	q15_t *result;
	int16_t *nco_base;
	int32_t s0;
	int32_t s1;
	int32_t s2;
	int32_t d0;
	int32_t d1;
	int32_t d2;
	int32_t dest;
} CICState;

static CICState cic_i;
static CICState cic_q;

__RAMFUNC(RAM)
void cic_decimate(CICState *cic, uint8_t *buf, int len)
{
	//const uint32_t offset = 0x08000800;
	const uint32_t offset = 0x08800880;
	int16_t *const result = (int16_t*)(cic->result);
	uint32_t *const capture = (uint32_t*)buf;
	const uint32_t *const nco_base = (uint32_t*)(cic->nco_base);

	int32_t s0 = cic->s0;
	int32_t s1 = cic->s1;
	int32_t s2 = cic->s2;
	int32_t d0 = cic->d0;
	int32_t d1 = cic->d1;
	int32_t d2 = cic->d2;
	int32_t e0, e1, e2;
	int i, l;

	l = cic->dest;
	for (i = 0; i < len / 4; ) {
		int j;
		const uint32_t *nco = nco_base;
		for (j = 0; j < NCO_SAMPLES/2; ) {
#if 1 /* unroll manually */
#define CIC0()	do { \
			uint32_t x = capture[i++]; \
			uint32_t f = *nco++; \
			x = __SSUB16(x, offset); \
			s0 = __SMLAD(x, f, s0); \
			s1 += s0; \
			s2 += s1; \
			j++; \
		} while(0)
			CIC0();CIC0();CIC0();CIC0();
			CIC0();CIC0();CIC0();CIC0();
			CIC0();CIC0();CIC0();CIC0();
			CIC0();CIC0();CIC0();CIC0();
#else
			int k;
			for (k = 0; k < DECIMATION_RATIO / 2; k++) {
				uint32_t x = capture[i++];
				uint32_t f = *nco++;
				x = __SSUB16(x, offset);
				s0 = __SMLAD(x, f, s0);
				s1 += s0;
				s2 += s1;
				j++;
			}
#endif
			e0 = d0 - s2;
			d0 = s2;
			e1 = d1 - e0;
			d1 = e0;
			e2 = d2 - e1;
			d2 = e1;
			result[l++] = __SSAT(e2 >> (16 - FIR_GAINBITS), 16);
			l %=  FIR_BUFFER_SIZE/2;
		}
	}
	cic->dest = l;
	cic->s0 = s0;
	cic->s1 = s1;
	cic->s2 = s2;
	cic->d0 = d0;
	cic->d1 = d1;
	cic->d2 = d2;
}

__RAMFUNC(RAM)
void fir_filter_iq()
{
	const uint32_t *coeff = (uint32_t*)fir_coeff;
	const uint32_t *in_i = (const uint32_t *)I_FIR_STATE;
	const uint32_t *in_q = (const uint32_t *)Q_FIR_STATE;
	int32_t length = FIR_BUFFER_SIZE / sizeof(uint32_t);
	uint32_t *dest = (uint32_t *)DEMOD_BUFFER;
	int i;

	for (i = 0; i < length; i++) {
		q31_t acc0_i = 0;
		q31_t acc1_i = 0;
		q31_t acc0_q = 0;
		q31_t acc1_q = 0;
		uint32_t x0 = in_i[0];
		uint32_t y0 = in_q[0];

#if 1 /* unroll manually */
#define STEP(j) \
	do {uint32_t c0 = coeff[j]; \
		uint32_t x2 = in_i[j+1]; \
		uint32_t y2 = in_q[j+1]; \
		acc0_i = __SMLAD(x0, c0, acc0_i); \
		acc0_q = __SMLAD(y0, c0, acc0_q); \
		acc1_i = __SMLADX(__PKHBT(x2, x0, 0), c0, acc1_i); \
		acc1_q = __SMLADX(__PKHBT(y2, y0, 0), c0, acc1_q); \
		x0 = x2; \
		y0 = y2; } while(0)

		STEP(0); STEP(1); STEP(2); STEP(3);
		STEP(4); STEP(5); STEP(6); STEP(7);
		STEP(8); STEP(9); STEP(10); STEP(11);
		STEP(12); STEP(13); STEP(14); STEP(15);
#else
		int j;
		for (j = 0; j < FIR_NUM_TAPS / 2; ) {
			uint32_t c0 = coeff[j++];
			uint32_t x2 = in_i[j];
			uint32_t y2 = in_q[j];
			acc0_i = __SMLAD(x0, c0, acc0_i);
			acc0_q = __SMLAD(y0, c0, acc0_q);
			acc1_i = __SMLADX(__PKHBT(x2, x0, 0), c0, acc1_i);
			acc1_q = __SMLADX(__PKHBT(y2, y0, 0), c0, acc1_q);
			x0 = x2;
			y0 = y2;
		}
#endif
		dest[i*2] = __PKHBT(__SSAT((acc0_i >> 15), 16), __SSAT((acc0_q >> 15), 16), 16);
		dest[i*2+1] = __PKHBT(__SSAT((acc1_i >> 15), 16), __SSAT((acc1_q >> 15), 16), 16);
		in_i++;
		in_q++;
	}

	uint32_t *state_i = (uint32_t *)I_FIR_STATE;
	for (i = 0; i < FIR_STATE_SIZE; i += 4) {
		//*state_i++ = *in_i++;
	    __asm__ volatile ("ldr r0, [%0, %2]\n"
	    				  "str r0, [%1, %2]\n" :: "l"(in_i), "l"(state_i), "X"(i): "r0");
	}
	uint32_t *state_q = (uint32_t *)Q_FIR_STATE;
	for (i = 0; i < FIR_STATE_SIZE; i += 4) {
		//*state_q++ = *in_q++;
	    __asm__ volatile ("ldr r0, [%0, %2]\n"
	    				  "str r0, [%1, %2]\n" :: "l"(in_q), "l"(state_q), "X"(i): "r0");
	}
}

struct {
	uint32_t last;
	int32_t carrier;
} fm_demod_state;

__RAMFUNC(RAM)
void fm_demod()
{
	uint32_t *src = (uint32_t *)DEMOD_BUFFER;
	int16_t *dest = (int16_t *)RESAMPLE_BUFFER;
	int32_t length = DEMOD_BUFFER_SIZE / sizeof(uint32_t);
	int i;

	uint32_t x0 = fm_demod_state.last;
	int32_t n = __SMUAD(x0, x0) >> DEMOD_GAINBITS;
	for (i = 0; i < length; i++) {
		uint32_t x1 = src[i];
#if 0
		// I*(I-I0)-Q*(Q-Q0)
		int32_t d = __SMUSDX(__SSUB16(x1, x0), x1);
		// I^2 + Q^2
		n = __SMUAD(x1, x1) >> DEMOD_GAINBITS;
		int32_t y = d / n;
		//dest[i] = y;
		dest[i] = __SSAT(y, 16);
		//dest[i] = __SSAT((y * ((1<<12) + (y>>2) * (y>>2) / 3)) >> 12, 16);
#else
#define AMPL_CONV (4*312e6f/(2*PI*150e3f))
		int32_t re = __SMUAD(x1, x0);
		int32_t im = __SMUSDX(x1, x0);
		uint8_t neg = FALSE;
		float32_t d;
		float32_t ang = 0;
		if (im < 0) {
			im = -im;
			neg = !neg;
		}
		if (re < 0) {
			re = -re;
			neg = !neg;
		}
		if (im >= re) {
			d = (float)re / (float)im;
			neg = !neg;
			ang = -PI / 2;
		} else {
			d = (float)im / (float)re;
		}
		d = d / (0.98419158358617365f + d * (0.093485702629671305f + d * 0.19556307900617517f));
		d += ang;
		if (neg)
			d = -d;
        d *= AMPL_CONV;
		dest[i] = __SSAT((int32_t)d, 16);
#endif
		x0 = x1;
	}
	fm_demod_state.last = x0;
	fm_demod_state.carrier = n;
}

#define RESAMPLE_NUM_TAPS	128

q15_t resample_fir_coeff_even[RESAMPLE_NUM_TAPS] = {
		   3,    0,   -3,   -6,   -7,   -5,   -1,    4,    9,   11,    9,
		          3,   -5,  -14,  -19,  -17,   -8,    5,   21,   31,   30,   18,
		         -4,  -28,  -46,  -49,  -33,   -1,   36,   66,   76,   57,   13,
		        -42,  -91, -111,  -93,  -35,   44,  119,  160,  145,   72,  -39,
		       -154, -228, -226, -136,   22,  200,  334,  363,  255,   23, -274,
		       -539, -662, -550, -163,  475, 1270, 2076, 2732, 3100, 3100, 2732,
		       2076, 1270,  475, -163, -550, -662, -539, -274,   23,  255,  363,
		        334,  200,   22, -136, -226, -228, -154,  -39,   72,  145,  160,
		        119,   44,  -35,  -93, -111,  -91,  -42,   13,   57,   76,   66,
		         36,   -1,  -33,  -49,  -46,  -28,   -4,   18,   30,   31,   21,
		          5,   -8,  -17,  -19,  -14,   -5,    3,    9,   11,    9,    4,
		         -1,   -5,   -7,   -6,   -3,    0,    3};
q15_t resample_fir_coeff_odd[RESAMPLE_NUM_TAPS] = {
		   1,   -2,   -5,   -7,   -6,   -3,    1,    7,   11,   11,    7,
		          0,  -10,  -17,  -19,  -14,   -1,   13,   27,   32,   25,    7,
		        -16,  -39,  -50,  -44,  -19,   17,   53,   74,   70,   38,  -14,
		        -69, -105, -107,  -68,    3,   84,  146,  160,  115,   19,  -99,
		       -199, -238, -191,  -63,  112,  276,  364,  327,  152, -122, -417,
		       -625, -639, -391,  129,  861, 1682, 2433, 2959, 3148, 2959, 2433,
		       1682,  861,  129, -391, -639, -625, -417, -122,  152,  327,  364,
		        276,  112,  -63, -191, -238, -199,  -99,   19,  115,  160,  146,
		         84,    3,  -68, -107, -105,  -69,  -14,   38,   70,   74,   53,
		         17,  -19,  -44,  -50,  -39,  -16,    7,   25,   32,   27,   13,
		         -1,  -14,  -19,  -17,  -10,    0,    7,   11,   11,    7,    1,
		         -3,   -6,   -7,   -5,   -2,    1,    0};

struct {
	int32_t index;
	float deemphasis_mult;
	float deemphasis_rest;
	float deemphasis_value;
} resample_state;

void
set_deemphasis(int timeconst_us)
{
	resample_state.deemphasis_value = 0;
	resample_state.deemphasis_mult = exp(-1e6/(timeconst_us * AUDIO_RATE));
	resample_state.deemphasis_rest = 1 - resample_state.deemphasis_mult;
}

volatile struct {
	uint16_t write_current;
	uint16_t write_total;
	uint16_t read_total;
	uint16_t read_current;
	uint16_t rebuffer_count;
} audio_state;

__RAMFUNC(RAM)
void resample_fir_filter()
{
	const uint32_t *coeff;
	const uint16_t *src = (const uint16_t *)RESAMPLE_STATE;
	const uint32_t *s;
	int32_t tail = RESAMPLE_BUFFER_SIZE;
	int32_t idx = resample_state.index;
	int32_t acc;
	int i, j;
	int cur = audio_state.write_current;
	uint16_t *dest = (uint16_t *)AUDIO_BUFFER;
	float value = resample_state.deemphasis_value;

	while (idx < tail) {
		//i = idx >> 16;
		if (idx & 0x1)
			coeff = (uint32_t*)resample_fir_coeff_odd;
		else
			coeff = (uint32_t*)resample_fir_coeff_even;

		acc = 0;
		s = (const uint32_t*)&src[idx >> 1];
		for (j = 0; j < RESAMPLE_NUM_TAPS / 2; j++) {
			acc = __SMLAD(*s++, *coeff++, acc);
		}

		// deemphasis with time constant
		value = (float)acc * resample_state.deemphasis_rest + value * resample_state.deemphasis_mult;
		dest[cur++] = __SSAT((int32_t)value >> (16 - RESAMPLE_GAINBITS), 16);
		//dest[cur++] = __PKHBT(__SSAT((acc0 >> 15), 16), __SSAT((acc1 >> 15), 16), 16);
		cur %= AUDIO_BUFFER_SIZE / 2;
		audio_state.write_total++;
		idx += 13;
	}

	resample_state.deemphasis_value = value;
	audio_state.write_current = cur;
	resample_state.index = idx - tail;
	uint32_t *state = (uint32_t *)RESAMPLE_STATE;
	src = &src[tail / sizeof(*src)];
	for (i = 0; i < RESAMPLE_STATE_SIZE / sizeof(uint32_t); i++) {
		//*state++ = *src++;
	    __asm__ volatile ("ldr r0, [%0], #+4\n" : : "r" (src) : "r0");
	    __asm__ volatile ("str r0, [%0], #+4\n" : : "r" (state) : "r0");
	}
}


#define REBUFFER_THRESHOLD0 	(1 * (AUDIO_BUFFER_SIZE/2) / 8)
#define REBUFFER_THRESHOLD1 	(7 * (AUDIO_BUFFER_SIZE/2) / 8)
#define REBUFFER_WR_GAP 		(4 * (AUDIO_BUFFER_SIZE/2) / 8)

__RAMFUNC(RAM)
void
audio_adjust_buffer()
{
	uint16_t d = audio_state.write_current - audio_state.read_current;
	d %= AUDIO_BUFFER_SIZE / 2;
	if (d < REBUFFER_THRESHOLD0 || d > REBUFFER_THRESHOLD1) {
		int cur = audio_state.write_current - REBUFFER_WR_GAP;
		if (cur < 0)
			cur += AUDIO_BUFFER_SIZE / 2;
		audio_state.read_current = cur;
		audio_state.rebuffer_count++;
	}
}

void generate_test_tone(int freq)
{
	int i;
	int16_t *buf = (int16_t*)AUDIO_BUFFER;
	int samples = AUDIO_BUFFER_SIZE / 2;
	int n = freq * samples / 48000;
	for (i = 0; i < AUDIO_BUFFER_SIZE / 2; i++) {
		float res = arm_sin_f32(((float)i * 2.0 * PI * n) / 4096);
		buf[i] = (int)(res * 20000.0);
	}
}


__RAMFUNC(RAM)
void DMA_IRQHandler (void)
{
  if (LPC_GPDMA->INTERRSTAT & 1)
  {
    LPC_GPDMA->INTERRCLR = 1;
  }

  if (LPC_GPDMA->INTTCSTAT & 1)
  {
	LPC_GPDMA->INTTCCLEAR = 1;

	TESTPOINT_ON();
    if ((capture_count & 1) == 0) {
    	cic_decimate(&cic_i, CAPTUREBUFFER0, CAPTUREBUFFER_SIZEHALF);
    	cic_decimate(&cic_q, CAPTUREBUFFER0, CAPTUREBUFFER_SIZEHALF);
    } else {
    	cic_decimate(&cic_i, CAPTUREBUFFER1, CAPTUREBUFFER_SIZEHALF);
    	cic_decimate(&cic_q, CAPTUREBUFFER1, CAPTUREBUFFER_SIZEHALF);
    }
	TESTPOINT_SPIKE();
	fir_filter_iq();
	TESTPOINT_SPIKE();
	fm_demod();
	TESTPOINT_SPIKE();
	resample_fir_filter();

	//audio_adjust_buffer();
	TESTPOINT_OFF();
    capture_count ++;

    //HALT_DMA(); // halt DMA for inspecting contents of buffer

    {
    	// toggle LED with every 1024 interrupts
    	int c = capture_count % 1024;
    	if (c == 0)
    	 	LED_ON();
    	else if (c == 512)
    	 	LED_OFF();
    }
  }
}

__RAMFUNC(RAM)
void I2S0_IRQHandler()
{
#if 1
	uint32_t txLevel = I2S_GetLevel(LPC_I2S0, I2S_TX_MODE);
	//TESTPOINT_ON();
	if (txLevel < 8) {
		// Fill the remaining FIFO
		int cur = audio_state.read_current;
		int16_t *buffer = (int16_t*)AUDIO_BUFFER;
		int i;
		for (i = 0; i < (8 - txLevel); i++) {
			uint32_t x = *(uint32_t *)&buffer[cur]; // read TWO samples
			LPC_I2S0->TXFIFO = x;//__PKHTB(x, x, 0);
			cur += 2;
			cur %= AUDIO_BUFFER_SIZE / 2;
			audio_state.read_total += 2;
		}
		audio_state.read_current = cur;
	}
	//TESTPOINT_OFF();
#endif
}

void
dsp_init()
{
	memset(&cic_i, 0, sizeof cic_i);
	memset(&cic_q, 0, sizeof cic_q);
	cic_i.result = I_FIR_BUFFER;
	cic_i.nco_base = NCO_SIN_TABLE;
	cic_q.result = Q_FIR_BUFFER;
	cic_q.nco_base = NCO_COS_TABLE;
	//set_deemphasis(75);
	set_deemphasis(50);
}
