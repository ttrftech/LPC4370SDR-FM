/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef __USE_CMSIS
#include "LPC43xx.h"
#endif

#include <cr_section_macros.h>
#include <limits.h>
#include <arm_math.h>
#include "lpc43xx_i2s.h"
#include "receiver.h"

#define NCO_CYCLE 1024
#define NCO_SAMPLES 1024
#define NCO_COS_OFFSET (NCO_CYCLE/4)

//float32_t arm_cos_f32(float32_t radian) __RAMFUNC(RAM);
//float32_t arm_sin_f32(float32_t radian) __RAMFUNC(RAM);

__RAMFUNC(RAM)
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
#if 0
			// original coeff
			-204,   -42,   328,   144,  -687,  -430,  1301,  1060, -2162,
		   -2298,  3208,  4691, -4150, -9707,  3106, 22273, 22273,  3106,
		   -9707, -4150,  4691,  3208, -2298, -2162,  1060,  1301,  -430,
			-687,   144,   328,   -42,  -204
#else
#if 0
			// bw*1.5
			 -414,    384,    -26,   -730,   1457,  -1183,   -755,   3523,
			-4611,   1579,   5072, -10479,   7881,   5033, -19714,  16931,
			16931, -19714,   5033,   7881, -10479,   5072,   1579,  -4611,
			 3523,   -755,  -1183,   1457,   -730,    -26,    384,   -414
#else
#if 0
			 // bw*1.5, flat
			 -55,    62,   -26,   -77,   210,  -242,    31,   409,  -801,
			 698,   174, -1569,  2597, -1942, -2137, 19028, 19028, -2137,
		   -1942,  2597, -1569,   174,   698,  -801,   409,    31,  -242,
			 210,   -77,   -26,    62,   -55
#else
#if 1
			 // bw=156kHz, flat
			 -59,    -5,    96,    24,  -208,   -82,   411,   213,  -732,
			-480,  1240,  1027, -2176, -2424,  5158, 14351, 14351,  5158,
		   -2424, -2176,  1027,  1240,  -480,  -732,   213,   411,   -82,
			-208,    24,    96,    -5,   -59
#else
			// fc=120kHz
			  10,   -60,   -73,    51,   213,    96,  -341,  -502,   175,
			1061,   640, -1323, -2434,   289,  6566, 11984, 11984,  6566,
			 289, -2434, -1323,   640,  1061,   175,  -502,  -341,    96,
			 213,    51,   -73,   -60,    10
#endif
#endif
#endif
#endif
};

/*typedef struct {
	q15_t *result;
	int16_t *nco_base;
	int32_t s0;
	int32_t s1;
	int32_t s2;
	int32_t d0;
	int32_t d1;
	int32_t d2;
	int32_t dest;
	uint32_t offset;
} cic_state_t;
*/

cic_state_t cic_i;
cic_state_t cic_q;

__RAMFUNC(RAM)
static uint32_t compute_adc_dc_offset(uint8_t *buf, int len)
{
	uint16_t *const capture = (uint16_t*)buf;
	int count = len / sizeof(uint16_t);
	uint32_t acc = 0;
	uint16_t offset;
	int i = 0;
	for (i = 0; i < count; i++)
		acc += capture[i];
	offset = acc / count;
	// place same 2 offset values on uint32_t
	return __PKHBT(offset, offset, 16);
}

static void cic_init(void)
{
	memset(&cic_i, 0, sizeof cic_i);
	memset(&cic_q, 0, sizeof cic_q);
	cic_i.dest = I_FIR_BUFFER;
	cic_i.nco_base = NCO_SIN_TABLE;
	cic_q.dest = Q_FIR_BUFFER;
	cic_q.nco_base = NCO_COS_TABLE;
	cic_i.dc_offset = 0x08800880;
	cic_q.dc_offset = 0x08800880;
}

void
update_adc_dc_offset(void)
{
	uint32_t offset = compute_adc_dc_offset(CAPTUREBUFFER0, CAPTUREBUFFER_SIZEHALF);
	cic_i.dc_offset = offset;
	cic_q.dc_offset = offset;
}

__RAMFUNC(RAM)
void cic_decimate(cic_state_t *cic, uint8_t *buf, int len)
{
	const uint32_t offset = cic->dc_offset;
	int16_t *const dest = (int16_t*)cic->dest;
	uint32_t *const capture = (uint32_t*)buf;
	const uint32_t *const nco_base = (uint32_t*)cic->nco_base;

	int32_t s0 = cic->s0;
	int32_t s1 = cic->s1;
	int32_t s2 = cic->s2;
	int32_t d0 = cic->d0;
	int32_t d1 = cic->d1;
	int32_t d2 = cic->d2;
	int32_t e0, e1, e2;
	int i, l;

	l = cic->dest_idx;
	for (i = 0; i < len / 4; ) {
		int j;
		for (j = 0; j < NCO_SAMPLES/2; ) {
#if 1 /* unroll manually */
#define CIC0()	do { \
			uint32_t f = nco_base[j++]; \
			uint32_t x = capture[i++]; \
			x = __SSUB16(x, offset); \
			s0 = __SMLAD(x, f, s0); \
			s1 += s0; \
			s2 += s1; \
		} while(0)
			CIC0();CIC0();CIC0();CIC0();
			CIC0();CIC0();CIC0();CIC0();
#else
			int k;
			for (k = 0; k < DECIMATION_RATIO / 2; k++) {
				uint32_t x = capture[i++];
				uint32_t f = nco_base[j];
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
			dest[l++] = __SSAT(e2 >> (16 - FIR_GAINBITS), 16);
			l %=  FIR_BUFFER_SIZE/2;
		}
	}
	cic->dest_idx = l;
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

	for (i = 0; i < length / 2; i++) {
		q31_t acc0_i = 0;
		q31_t acc0_q = 0;
		q31_t acc1_i = 0;
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
		acc1_i = __SMLAD(x2, c0, acc1_i); \
		acc1_q = __SMLAD(y2, c0, acc1_q); \
		x0 = x2; \
		y0 = y2; \
	} while(0)

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
		in_i += 2;
		in_q += 2;
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

const int16_t arctantbl[256+2] = {
	    0,   128,   256,   384,   512,   640,   768,   896,  1024,
	        1152,  1279,  1407,  1535,  1663,  1790,  1918,  2045,  2173,
	        2300,  2428,  2555,  2682,  2809,  2936,  3063,  3190,  3317,
	        3443,  3570,  3696,  3823,  3949,  4075,  4201,  4327,  4452,
	        4578,  4703,  4829,  4954,  5079,  5204,  5329,  5453,  5578,
	        5702,  5826,  5950,  6073,  6197,  6320,  6444,  6567,  6689,
	        6812,  6935,  7057,  7179,  7301,  7422,  7544,  7665,  7786,
	        7907,  8027,  8148,  8268,  8388,  8508,  8627,  8746,  8865,
	        8984,  9102,  9221,  9339,  9456,  9574,  9691,  9808,  9925,
	       10041, 10158, 10274, 10389, 10505, 10620, 10735, 10849, 10964,
	       11078, 11192, 11305, 11418, 11531, 11644, 11756, 11868, 11980,
	       12092, 12203, 12314, 12424, 12535, 12645, 12754, 12864, 12973,
	       13082, 13190, 13298, 13406, 13514, 13621, 13728, 13835, 13941,
	       14047, 14153, 14258, 14363, 14468, 14573, 14677, 14781, 14884,
	       14987, 15090, 15193, 15295, 15397, 15499, 15600, 15701, 15801,
	       15902, 16002, 16101, 16201, 16300, 16398, 16497, 16595, 16693,
	       16790, 16887, 16984, 17080, 17176, 17272, 17368, 17463, 17557,
	       17652, 17746, 17840, 17933, 18027, 18119, 18212, 18304, 18396,
	       18488, 18579, 18670, 18760, 18851, 18941, 19030, 19120, 19209,
	       19297, 19386, 19474, 19561, 19649, 19736, 19823, 19909, 19995,
	       20081, 20166, 20252, 20336, 20421, 20505, 20589, 20673, 20756,
	       20839, 20922, 21004, 21086, 21168, 21249, 21331, 21411, 21492,
	       21572, 21652, 21732, 21811, 21890, 21969, 22047, 22126, 22203,
	       22281, 22358, 22435, 22512, 22588, 22664, 22740, 22815, 22891,
	       22966, 23040, 23115, 23189, 23262, 23336, 23409, 23482, 23555,
	       23627, 23699, 23771, 23842, 23914, 23985, 24055, 24126, 24196,
	       24266, 24335, 24405, 24474, 24542, 24611, 24679, 24747, 24815,
	       24882, 24950, 25017, 25083, 25150, 25216, 25282, 25347, 25413,
	       25478, 25543, 25607, 25672, 25736, 25736
};

#define Q15_PI_4	25736	// 3.14159/4*32768


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
		int32_t d = __SMUSDX(__QSUB16(x1, x0), x1);
		// I^2 + Q^2
		n = __SMUAD(x1, x1) >> DEMOD_GAINBITS;
		int32_t y = d / n;
		//dest[i] = y;
		dest[i] = __SSAT(y, 16);
		//dest[i] = __SSAT((y * ((1<<12) + (y>>2) * (y>>2) / 3)) >> 14, 16);
		//dest[i] = __SSAT((y * (32768 - y * ((y*7838 + 6226)>>16))) >> 15, 16);
#endif
#if 1
		int32_t re = __SMUAD(x1, x0);	// I0*I1 + Q0*Q1
		int32_t im = __SMUSDX(x1, x0);	// I0*Q1 - I1*Q0
		int32_t ang = 0;
		uint8_t neg = FALSE;
		uint32_t d, f;
		int32_t a, b;
		int idx;
		if (re < 0) {
			re = -re;
			neg = !neg;
			ang += -Q15_PI_4 * 4;
		}
		if (im < 0) {
			im = -im;
			neg = !neg;
		}
		if (im >= re) {
			int32_t x = im;
			im = re;
			re = x;
			neg = !neg;
			ang = -ang - Q15_PI_4 * 2;
		}
#if 1
		d = im << 0;
		d /= re >> 16;
#else
		float32_t x = (float32_t)im * 65536;
		d = x / re;
#endif
		idx = (d >> 8) & 0xff;
		f = d & 0xff;
		a = arctantbl[idx];
		b = arctantbl[idx+1];
		ang += a + (((b - a) * f) >> 8);
		if (neg)
			ang = -ang;
		dest[i] = __SSAT(ang/16, 16);
#endif
#if 0
#define AMPL_CONV (4*312e6f/(2*PI*150e3f))
		int32_t re = __SMUAD(x1, x0);	// I0*I1 + Q0*Q1
		int32_t im = __SMUSDX(x1, x0);	// I0*Q1 - I1*Q0
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

// state variables for stereo separation
stereo_separate_state_t stereo_separate_state;

const int16_t sin_table[256][2] = {
{     0, -804 },{  -804, -804 },{ -1608, -802 },{ -2410, -802 },{ -3212, -799 },{ -4011, -797 },
{ -4808, -794 },{ -5602, -791 },{ -6393, -786 },{ -7179, -783 },{ -7962, -777 },{ -8739, -773 },
{ -9512, -766 },{ -10278, -761 },{ -11039, -754 },{ -11793, -746 },{ -12539, -740 },{ -13279, -731 },
{ -14010, -722 },{ -14732, -714 },{ -15446, -705 },{ -16151, -695 },{ -16846, -684 },{ -17530, -674 },
{ -18204, -664 },{ -18868, -651 },{ -19519, -640 },{ -20159, -628 },{ -20787, -616 },{ -21403, -602 },
{ -22005, -589 },{ -22594, -576 },{ -23170, -561 },{ -23731, -548 },{ -24279, -532 },{ -24811, -518 },
{ -25329, -503 },{ -25832, -487 },{ -26319, -471 },{ -26790, -455 },{ -27245, -438 },{ -27683, -422 },
{ -28105, -405 },{ -28510, -388 },{ -28898, -370 },{ -29268, -353 },{ -29621, -335 },{ -29956, -317 },
{ -30273, -298 },{ -30571, -281 },{ -30852, -261 },{ -31113, -243 },{ -31356, -224 },{ -31580, -205 },
{ -31785, -186 },{ -31971, -166 },{ -32137, -148 },{ -32285, -127 },{ -32412, -109 },{ -32521,  -88 },
{ -32609,  -69 },{ -32678,  -50 },{ -32728,  -29 },{ -32757,  -10 },{ -32767,   10 },{ -32757,   29 },
{ -32728,   50 },{ -32678,   69 },{ -32609,   88 },{ -32521,  109 },{ -32412,  127 },{ -32285,  148 },
{ -32137,  166 },{ -31971,  186 },{ -31785,  205 },{ -31580,  224 },{ -31356,  243 },{ -31113,  261 },
{ -30852,  281 },{ -30571,  298 },{ -30273,  317 },{ -29956,  335 },{ -29621,  353 },{ -29268,  370 },
{ -28898,  388 },{ -28510,  405 },{ -28105,  422 },{ -27683,  438 },{ -27245,  455 },{ -26790,  471 },
{ -26319,  487 },{ -25832,  503 },{ -25329,  518 },{ -24811,  532 },{ -24279,  548 },{ -23731,  561 },
{ -23170,  576 },{ -22594,  589 },{ -22005,  602 },{ -21403,  616 },{ -20787,  628 },{ -20159,  640 },
{ -19519,  651 },{ -18868,  664 },{ -18204,  674 },{ -17530,  684 },{ -16846,  695 },{ -16151,  705 },
{ -15446,  714 },{ -14732,  722 },{ -14010,  731 },{ -13279,  740 },{ -12539,  746 },{ -11793,  754 },
{ -11039,  761 },{ -10278,  766 },{ -9512,  773 },{ -8739,  777 },{ -7962,  783 },{ -7179,  786 },
{ -6393,  791 },{ -5602,  794 },{ -4808,  797 },{ -4011,  799 },{ -3212,  802 },{ -2410,  802 },
{ -1608,  804 },{  -804,  804 },{     0,  804 },{   804,  804 },{  1608,  802 },{  2410,  802 },
{  3212,  799 },{  4011,  797 },{  4808,  794 },{  5602,  791 },{  6393,  786 },{  7179,  783 },
{  7962,  777 },{  8739,  773 },{  9512,  766 },{ 10278,  761 },{ 11039,  754 },{ 11793,  746 },
{ 12539,  740 },{ 13279,  731 },{ 14010,  722 },{ 14732,  714 },{ 15446,  705 },{ 16151,  695 },
{ 16846,  684 },{ 17530,  674 },{ 18204,  664 },{ 18868,  651 },{ 19519,  640 },{ 20159,  628 },
{ 20787,  616 },{ 21403,  602 },{ 22005,  589 },{ 22594,  576 },{ 23170,  561 },{ 23731,  548 },
{ 24279,  532 },{ 24811,  518 },{ 25329,  503 },{ 25832,  487 },{ 26319,  471 },{ 26790,  455 },
{ 27245,  438 },{ 27683,  422 },{ 28105,  405 },{ 28510,  388 },{ 28898,  370 },{ 29268,  353 },
{ 29621,  335 },{ 29956,  317 },{ 30273,  298 },{ 30571,  281 },{ 30852,  261 },{ 31113,  243 },
{ 31356,  224 },{ 31580,  205 },{ 31785,  186 },{ 31971,  166 },{ 32137,  148 },{ 32285,  127 },
{ 32412,  109 },{ 32521,   88 },{ 32609,   69 },{ 32678,   50 },{ 32728,   29 },{ 32757,   10 },
{ 32767,  -10 },{ 32757,  -29 },{ 32728,  -50 },{ 32678,  -69 },{ 32609,  -88 },{ 32521, -109 },
{ 32412, -127 },{ 32285, -148 },{ 32137, -166 },{ 31971, -186 },{ 31785, -205 },{ 31580, -224 },
{ 31356, -243 },{ 31113, -261 },{ 30852, -281 },{ 30571, -298 },{ 30273, -317 },{ 29956, -335 },
{ 29621, -353 },{ 29268, -370 },{ 28898, -388 },{ 28510, -405 },{ 28105, -422 },{ 27683, -438 },
{ 27245, -455 },{ 26790, -471 },{ 26319, -487 },{ 25832, -503 },{ 25329, -518 },{ 24811, -532 },
{ 24279, -548 },{ 23731, -561 },{ 23170, -576 },{ 22594, -589 },{ 22005, -602 },{ 21403, -616 },
{ 20787, -628 },{ 20159, -640 },{ 19519, -651 },{ 18868, -664 },{ 18204, -674 },{ 17530, -684 },
{ 16846, -695 },{ 16151, -705 },{ 15446, -714 },{ 14732, -722 },{ 14010, -731 },{ 13279, -740 },
{ 12539, -746 },{ 11793, -754 },{ 11039, -761 },{ 10278, -766 },{  9512, -773 },{  8739, -777 },
{  7962, -783 },{  7179, -786 },{  6393, -791 },{  5602, -794 },{  4808, -797 },{  4011, -799 },
{  3212, -802 },{  2410, -802 },{  1608, -804 },{   804, -804 }
};

static inline
uint32_t cos_sin(uint16_t phase)
{
    uint16_t sidx = phase / 256;
    uint16_t cidx = (sidx + 64) & 0xff;
    uint16_t mod = phase & 0xff;
    uint32_t sd = *(uint32_t*)&sin_table[sidx];
    uint32_t cd = *(uint32_t*)&sin_table[cidx];
    uint32_t r = __PKHBT(0x0100, mod, 16);
    int32_t c = __SMUAD(r, cd);
    int32_t s = __SMUAD(r, sd);
    c /= 256;
    s /= 256;
    return __PKHBT(s, c, 16);
}

#define PHASESTEP_NCO19KHz 	((19000L*65536)/IF_RATE)*65536

void
cos_sin_test(uint32_t *buf, int len)
{
	uint32_t phase = 0;
	int i;
	for (i = 0; i < len; i++) {
		*buf++ = cos_sin(phase >> 16);
		phase += PHASESTEP_NCO19KHz;
	}
}

void
stereo_separate_init(float32_t pilotfreq)
{
#if 0
	float32_t angle = 2*PI * pilotfreq / IF_RATE;
	int i;
	stereo_separate_state.carrier_i = 1;
	stereo_separate_state.carrier_q = 0;
	stereo_separate_state.basestep_cos = arm_cos_f32(angle);
	stereo_separate_state.basestep_sin = arm_sin_f32(angle);
	stereo_separate_state.step_cos = stereo_separate_state.basestep_cos;
	stereo_separate_state.step_sin = stereo_separate_state.basestep_sin;
#else
	stereo_separate_state.phase_accum = 0;
	stereo_separate_state.phase_step_default = pilotfreq * 65536 / IF_RATE * 65536;
	stereo_separate_state.phase_step = stereo_separate_state.phase_step_default;
#endif
	stereo_separate_state.corr = 0;
	stereo_separate_state.sdi = 0;
	stereo_separate_state.sdq = 0;
#if 0
	angle /= 1024.0f;
	for (i = 0; i < 12; i++) {
		stereo_separate_state.delta_cos[i] = arm_cos_f32(angle);
		stereo_separate_state.delta_sin[i] = arm_sin_f32(angle);
		angle /= 2.0f;
	}
#endif
}

__RAMFUNC(RAM)
void stereo_separate()
{
	int16_t *src = (int16_t *)RESAMPLE_BUFFER;
	int16_t *dest = (int16_t *)RESAMPLE2_BUFFER;
	int32_t length = RESAMPLE_BUFFER_SIZE / sizeof(int16_t);
	int i;
#if 0
	float32_t carr_i = stereo_separate_state.carrier_i;
	float32_t carr_q = stereo_separate_state.carrier_q;
	float32_t ampl;
	float32_t step_cos = stereo_separate_state.step_cos;
	float32_t step_sin = stereo_separate_state.step_sin;
	float32_t di = 0;
	float32_t dq = 0;
#else
	int32_t di = 0;
	int32_t dq = 0;
	uint32_t phase_accum = stereo_separate_state.phase_accum;
	uint32_t phase_step = stereo_separate_state.phase_step;
#endif
	int32_t corr = 0;

	for (i = 0; i < length; i++) {
#if 0
		float32_t x1 = src[i];
		dest[i] = x1 * (2 * carr_i * carr_q) * 2;
		di += carr_i * x1;
		dq += carr_q * x1;
		float32_t new_i = carr_i * step_cos - carr_q * step_sin;
		float32_t new_q = carr_i * step_sin + carr_q * step_cos;
		carr_i = new_i;
		carr_q = new_q;
#else
		int32_t x = src[i];
		uint32_t cs = cos_sin(phase_accum >> 16);
		int16_t s = cs & 0xffff;
		int16_t c = cs >> 16;
		int16_t ss = (int32_t)(c * s) >> (15-1);
		dest[i] = (x * ss) >> (15-1);
		di += (c * x) >> 16;
		dq += (s * x) >> 16;
		phase_accum += phase_step;
#endif
	}
#if 0
	arm_sqrt_f32(carr_i * carr_i + carr_q * carr_q, &ampl);
	stereo_separate_state.carrier_i = carr_i / ampl;
	stereo_separate_state.carrier_q = carr_q / ampl;
#endif

	// averaging correlation
	di = (stereo_separate_state.sdi * 15 + di) / 16;
	dq = (stereo_separate_state.sdq * 15 + dq) / 16;
	stereo_separate_state.sdi = di;
	stereo_separate_state.sdq = dq;
	if (di > 0) {
		corr = 1024 * dq / di;
		//corr += stereo_separate_state.corr;
		if (corr > 4095)
			corr = 4095;
		else if (corr < -4095)
			corr = -4095;
	} else {
		if (dq > 0)
			corr = 4095;
		else if (dq < 0)
			corr = -4095;
	}
	if (corr != 0) {
#if 0
		float32_t step_cos = stereo_separate_state.basestep_cos;
		float32_t step_sin = stereo_separate_state.basestep_sin;
		int k;
		int kc = 2048;
		int c = corr;
		if (c < 0)
			c = -c;
		for (k = 0; kc > 0; k++, kc >>= 1) {
			if (c >= kc) {
				float32_t dc = stereo_separate_state.delta_cos[k];
				float32_t ds = stereo_separate_state.delta_sin[k];
				if (corr > 0)
					ds = -ds;
				float32_t sc = step_cos * dc - step_sin * ds;
				step_sin = step_cos * ds + step_sin * dc;
				step_cos = sc;
				c -= kc;
			}
		}
		stereo_separate_state.step_cos = step_cos;
		stereo_separate_state.step_sin = step_sin;
#else
		phase_step = stereo_separate_state.phase_step_default - corr*15;
		//phase_step = stereo_separate_state.phase_step_default - corr*30;
		//phase_step = stereo_separate_state.phase_step_default - corr*124;
		//phase_step -= corr/10;
		//if (phase_step > stereo_separate_state.phase_step_default + 10000000
		// || phase_step < stereo_separate_state.phase_step_default - 10000000)
		//	phase_step = stereo_separate_state.phase_step_default;
		stereo_separate_state.phase_step = phase_step;
		stereo_separate_state.phase_accum = phase_accum;

#endif
		stereo_separate_state.corr = corr;

		stereo_separate_state.corr_ave = (stereo_separate_state.corr_ave * 15 + corr) / 16;
		int32_t d = stereo_separate_state.corr_ave - corr;
		int32_t sd = (stereo_separate_state.corr_std * 15 + d * d) / 16;
		if (sd > 32767)
			sd = 32767;
		stereo_separate_state.corr_std = sd;
	}
}

#define RESAMPLE_NUM_TAPS	128

// two arrays of FIR coefficients
q15_t resample_fir_coeff[2][RESAMPLE_NUM_TAPS] = {
#if 0
{	  1,   -2,   -5,   -7,   -6,   -3,    1,    7,   11,   11,    7,
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
	 -3,   -6,   -7,   -5,   -2,    1,    0},
{     3,    0,   -3,   -6,   -7,   -5,   -1,    4,    9,   11,    9,
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
	 -1,   -5,   -7,   -6,   -3,    0,    3}
#else
#if 0
		// designed by firwin(15e3)
{   	   0,   -1,   -3,   -5,   -7,   -8,   -9,   -9,   -8,   -6,   -3,
           0,    5,   10,   16,   21,   24,   26,   25,   21,   14,    3,
          -8,  -22,  -36,  -49,  -58,  -63,  -63,  -55,  -40,  -19,    7,
          37,   68,   96,  120,  134,  137,  126,  101,   61,    9,  -51,
        -117, -181, -238, -280, -300, -294, -256, -184,  -77,   61,  229,
         419,  622,  829, 1028, 1209, 1362, 1478, 1551, 1576, 1551, 1478,
        1362, 1209, 1028,  829,  622,  419,  229,   61,  -77, -184, -256,
        -294, -300, -280, -238, -181, -117,  -51,    9,   61,  101,  126,
         137,  134,  120,   96,   68,   37,    7,  -19,  -40,  -55,  -63,
         -63,  -58,  -49,  -36,  -22,   -8,    3,   14,   21,   25,   26,
          24,   21,   16,   10,    5,    0,   -3,   -6,   -8,   -9,   -9,
          -8,   -7,   -5,   -3,   -1,    0,    0},
{		   1,    0,   -2,   -4,   -6,   -7,   -9,   -9,   -9,   -7,   -5,
          -1,    2,    8,   13,   19,   23,   26,   26,   23,   18,    9,
          -2,  -15,  -29,  -43,  -54,  -61,  -64,  -60,  -48,  -30,   -6,
          21,   52,   82,  109,  128,  137,  133,  115,   83,   37,  -20,
         -84, -149, -211, -261, -293, -301, -279, -224, -135,  -11,  142,
         322,  520,  726,  930, 1122, 1290, 1425, 1521, 1569, 1569, 1521,
        1425, 1290, 1122,  930,  726,  520,  322,  142,  -11, -135, -224,
        -279, -301, -293, -261, -211, -149,  -84,  -20,   37,   83,  115,
         133,  137,  128,  109,   82,   52,   21,   -6,  -30,  -48,  -60,
         -64,  -61,  -54,  -43,  -29,  -15,   -2,    9,   18,   23,   26,
          26,   23,   19,   13,    8,    2,   -1,   -5,   -7,   -9,   -9,
          -9,   -7,   -6,   -4,   -2,    0,    1}
#else
          // designed by remez [0,12e3,18.8e3,312e3]
          {  58,   30,   16,    8,    2,   -3,   -9,  -15,  -19,  -23,  -24,
                   -23,  -18,  -11,   -1,   10,   22,   33,   42,   48,   48,   43,
                    32,   16,   -4,  -26,  -49,  -69,  -84,  -90,  -88,  -75,  -52,
                   -20,   17,   59,   99,  133,  157,  166,  158,  131,   85,   24,
                   -48, -127, -203, -268, -314, -332, -316, -261, -164,  -26,  147,
                   350,  573,  805, 1031, 1239, 1416, 1552, 1637, 1665, 1637, 1552,
                  1416, 1239, 1031,  805,  573,  350,  147,  -26, -164, -261, -316,
                  -332, -314, -268, -203, -127,  -48,   24,   85,  131,  158,  166,
                   157,  133,   99,   59,   17,  -20,  -52,  -75,  -88,  -90,  -84,
                   -69,  -49,  -26,   -4,   16,   32,   43,   48,   48,   42,   33,
                    22,   10,   -1,  -11,  -18,  -23,  -24,  -23,  -19,  -15,   -9,
                    -3,    2,    8,   16,   30,   58},
           { -85,   42,   22,   12,    5,    0,   -6,  -12,  -17,  -21,  -24,
                   -24,  -21,  -15,   -6,    4,   16,   28,   38,   45,   49,   46,
                    38,   24,    6,  -15,  -38,  -59,  -77,  -88,  -90,  -83,  -65,
                   -37,   -2,   38,   79,  117,  146,  163,  164,  147,  110,   56,
                   -11,  -87, -166, -238, -294, -327, -329, -293, -217, -100,   56,
                   245,  460,  689,  919, 1138, 1332, 1490, 1601, 1658, 1658, 1601,
                  1490, 1332, 1138,  919,  689,  460,  245,   56, -100, -217, -293,
                  -329, -327, -294, -238, -166,  -87,  -11,   56,  110,  147,  164,
                   163,  146,  117,   79,   38,   -2,  -37,  -65,  -83,  -90,  -88,
                   -77,  -59,  -38,  -15,    6,   24,   38,   46,   49,   45,   38,
                    28,   16,    4,   -6,  -15,  -21,  -24,  -24,  -21,  -17,  -12,
                    -6,    0,    5,   12,   22,   42,  -85}
#endif
#endif
};

struct {
	int32_t index;
	float deemphasis_mult;
	float deemphasis_rest;
	float deemphasis_value;
	float deemphasis_value2;
} resample_state;

void
set_deemphasis(int timeconst_us)
{
	resample_state.deemphasis_value = 0;
	resample_state.deemphasis_value2 = 0;
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
		coeff = (uint32_t*)resample_fir_coeff[idx % 2];
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
		idx += 13; /* 2/13 decimation: 2 samples per loop */
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

__RAMFUNC(RAM)
void stereo_matrix()
{
	uint32_t *s1 = (uint32_t *)RESAMPLE_BUFFER;
	uint32_t *s2 = (uint32_t *)RESAMPLE2_BUFFER;
	int i;
	for (i = 0; i < RESAMPLE_BUFFER_SIZE/4; i++) {
		uint32_t x1 = *s1;
		uint32_t x2 = *s2;
		uint32_t l = __QADD16(x1, x2);
		uint32_t r = __QSUB16(x1, x2);
		*s1++ = l;
		*s2++ = r;
	}
}

__RAMFUNC(RAM)
__RAMFUNC(RAM)
void resample_fir_filter_stereo()
{
	const uint32_t *coeff;
	const uint16_t *src1 = (const uint16_t *)RESAMPLE_STATE;
	const uint16_t *src2 = (const uint16_t *)RESAMPLE2_STATE;
	const uint32_t *s1, *s2;
	int32_t tail = RESAMPLE_BUFFER_SIZE;
	int32_t idx = resample_state.index;
	int32_t acc1, acc2;
	int i, j;
	int cur = audio_state.write_current;
	uint16_t *dest = (uint16_t *)AUDIO_BUFFER;
	float val1 = resample_state.deemphasis_value;
	float val2 = resample_state.deemphasis_value2;

	while (idx < tail) {
		coeff = (uint32_t*)resample_fir_coeff[idx % 2];
		acc1 = 0;
		acc2 = 0;
		s1 = (const uint32_t*)&src1[idx >> 1];
		s2 = (const uint32_t*)&src2[idx >> 1];
		for (j = 0; j < RESAMPLE_NUM_TAPS / 2; j++) {
			uint32_t x1 = *s1++;
			uint32_t x2 = *s2++;
			//uint32_t l = __SADD16(x1, x2);
			//uint32_t r = __SSUB16(x1, x2);
			acc1 = __SMLAD(x1, *coeff, acc1);
			acc2 = __SMLAD(x2, *coeff, acc2);
			coeff++;
		}

		// deemphasis with time constant
		val1 = (float)acc1 * resample_state.deemphasis_rest + val1 * resample_state.deemphasis_mult;
		val2 = (float)acc2 * resample_state.deemphasis_rest + val2 * resample_state.deemphasis_mult;
		dest[cur++] = __SSAT((int32_t)(val1+val2) >> (16 - RESAMPLE_GAINBITS), 16);
		dest[cur++] = __SSAT((int32_t)(val1-val2) >> (16 - RESAMPLE_GAINBITS), 16);
		//dest[cur++] = __SSAT((int32_t)(val1) >> (16 - RESAMPLE_GAINBITS), 16);
		//dest[cur++] = __SSAT((int32_t)(val1) >> (16 - RESAMPLE_GAINBITS), 16);
		//dest[cur++] = 0;
		cur %= AUDIO_BUFFER_SIZE / 2;
		audio_state.write_total += 2;
		idx += 13; /* 2/13 decimation: 2 samples per loop */
	}

	resample_state.deemphasis_value = val1;
	resample_state.deemphasis_value2 = val2;
	audio_state.write_current = cur;
	resample_state.index = idx - tail;
	uint32_t *state = (uint32_t *)RESAMPLE_STATE;
	src1 = &src1[tail / sizeof(*src1)];
	for (i = 0; i < RESAMPLE_STATE_SIZE / sizeof(uint32_t); i++) {
		//*state++ = *src1++;
	    __asm__ volatile ("ldr r0, [%0], #+4\n" : : "r" (src1) : "r0");
	    __asm__ volatile ("str r0, [%0], #+4\n" : : "r" (state) : "r0");
	}
	state = (uint32_t *)RESAMPLE2_STATE;
	src2 = &src2[tail / sizeof(*src2)];
	for (i = 0; i < RESAMPLE_STATE_SIZE / sizeof(uint32_t); i++) {
		//*state++ = *src2++;
	    __asm__ volatile ("ldr r0, [%0], #+4\n" : : "r" (src2) : "r0");
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
	int samples = AUDIO_BUFFER_SIZE / 2 / 2;
	int n = freq * samples / AUDIO_RATE;
	for (i = 0; i < samples; i++) {
		float res = arm_sin_f32(((float)i * 2.0 * PI * n) / samples);
		buf[i*2  ] = (int)(res * 8000.0);
		buf[i*2+1] = (int)(res * 8000.0);
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
#if STEREO
	stereo_separate();
	//TESTPOINT_SPIKE();
	//stereo_matrix();
	TESTPOINT_SPIKE();
	resample_fir_filter_stereo();
#else
	resample_fir_filter();
#endif

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
	cic_init();
	//set_deemphasis(75);
	set_deemphasis(50);
	stereo_separate_init(19e3f);
}
