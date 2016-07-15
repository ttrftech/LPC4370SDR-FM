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
/*
 * @copyright Copyright 2013 Embedded Artists AB
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <lpc43xx.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_emc.h>
#include <lpc43xx_i2s.h>
#include <lpc43xx_scu.h>

#include "receiver.h"
#include "vadc.h"

/*! Frequency of external xtal */
#define XTAL_FREQ  (12000000UL)

extern uint32_t CGU_ClockSourceFrequency[CGU_CLKSRC_NUM];



void setup_systemclock()
{
	/* enable the crystal oscillator */
	CGU_SetXTALOSC(XTAL_FREQ);
	CGU_EnableEntity(CGU_CLKSRC_XTAL_OSC, ENABLE);

	/* connect the cpu to the xtal */
	CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_BASE_M4);

	/* connect the PLL to the xtal */
	CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_CLKSRC_PLL1);

	/* configure the PLL to 120 MHz */
	CGU_SetPLL1(10);
	while((LPC_CGU->PLL1_STAT&1) == 0x0);

	/* enable the PLL */
	CGU_EnableEntity(CGU_CLKSRC_PLL1, ENABLE);

	/* connect to the CPU core */
	CGU_EntityConnect(CGU_CLKSRC_PLL1, CGU_BASE_M4);

	SystemCoreClock = 120000000;

	/* wait one msec */
	emc_WaitUS(1000);

	/* Change the clock to 204 MHz */
	CGU_SetPLL1(17);
	while((LPC_CGU->PLL1_STAT&1) == 0x0);

	SystemCoreClock = 204000000;

    CGU_ClockSourceFrequency[CGU_CLKSRC_PLL1] = SystemCoreClock;
}

#define PLL0_MSEL_MAX (1<<15)
#define PLL0_NSEL_MAX (1<<8)
#define PLL0_PSEL_MAX (1<<5)

static uint32_t FindMDEC(uint32_t msel)
{
  /* multiplier: compute mdec from msel */
  uint32_t x = 0x4000;
  uint32_t im;

  switch (msel)
  {
    case 0:
      return 0xffffffff;
    case 1:
      return 0x18003;
    case 2:
      return 0x10003;
    default:
      for (im = msel; im <= PLL0_MSEL_MAX; im++)
      {
        x = (((x ^ x>>1) & 1) << 14) | (x>>1 & 0xFFFF);
      }
      return x;
  }
}

static uint32_t FindNDEC(uint32_t nsel)
{
  /* pre-divider: compute ndec from nsel */
  uint32_t x = 0x80;
  uint32_t in;

  switch (nsel)
  {
    case 0:
      return 0xffffffff;
    case 1:
      return 0x302;
    case 2:
      return 0x202;
    default:
      for (in = nsel; in <= PLL0_NSEL_MAX; in++)
      {
        x = (((x ^ x>>2 ^ x>>3 ^ x>>4) & 1) << 7) | (x>>1 & 0xFF);
      }
      return x;
  }
}

static uint32_t FindPDEC(uint32_t psel)
{
  /* post-divider: compute pdec from psel */
  uint32_t x = 0x10;
  uint32_t ip;

  switch (psel)
  {
    case 0:
      return 0xffffffff;
    case 1:
      return 0x62;
    case 2:
      return 0x42;
    default:
      for (ip = psel; ip <= PLL0_PSEL_MAX; ip++)
      {
        x = (((x ^ x>>2) & 1) << 4) | (x>>1 & 0x3F);
      }
      return x;
  }
}

void setup_pll0audio(uint32_t msel, uint32_t nsel, uint32_t psel)
{
  uint32_t ClkSrc;

  //CGU_EnableEntity(CGU_BASE_PERIPH, DISABLE);
  CGU_EnableEntity(CGU_BASE_VADC, DISABLE);

#if 0//EXTCLK_10MHZ
  scu_pinmux(0xF, 4, MD_PLN_FAST, FUNC1);     // GP_CLKIN
  CGU_ClockSourceFrequency[CGU_CLKSRC_GP_CLKIN] = 10000000*4;
  ClkSrc = CGU_CLKSRC_GP_CLKIN;
#else
  /* source = XTAL OSC 12 MHz */
  ClkSrc = CGU_CLKSRC_XTAL_OSC;
#endif

  /* disable clock, disable skew enable, power down pll,
  * (dis/en)able post divider, (dis/en)able pre-divider,
  * disable free running mode, disable bandsel,
  * enable up limmiter, disable bypass
  */
  LPC_CGU->PLL0AUDIO_CTRL = (ClkSrc << 24) | _BIT(0);  /* power down */

  /* set NDEC, PDEC and MDEC register */
  LPC_CGU->PLL0AUDIO_NP_DIV = (FindNDEC(nsel)<<12) | (FindPDEC(psel) << 0);
  LPC_CGU->PLL0AUDIO_MDIV = FindMDEC(msel);

  LPC_CGU->PLL0AUDIO_CTRL = (ClkSrc << 24) | (6<< 12);     // fractional divider off and bypassed

  /* wait for lock */
  while (!(LPC_CGU->PLL0AUDIO_STAT & 1));

  /* enable clock output */
  LPC_CGU->PLL0AUDIO_CTRL |= (1<<4); /* CLKEN */

  CGU_ClockSourceFrequency[CGU_CLKSRC_PLL0_AUDIO] =
    msel * (CGU_ClockSourceFrequency[ClkSrc] / (psel * nsel));

  //CGU_UpdateClock();

  // Re-enable the clocks that uses PLL0AUDIO
  //CGU_EnableEntity(CGU_BASE_PERIPH, ENABLE);
  CGU_EnableEntity(CGU_BASE_VADC, ENABLE);
}

void setup_i2s_clock(LPC_I2Sn_Type *I2Sx, uint32_t Freq, uint8_t TRMode)
{
#if 0
	/* Calculate bit rate
	 * The formula is:
	 *      bit_rate = channel*wordwidth - 1
	 * 48kHz sample rate for 16 bit stereo date requires
	 * a bit rate of 48000*16*2=1536MHz (MCLK)
	 */
	uint32_t i2sPclk;
	uint64_t divider;
	uint8_t bitrate, wordwidth;
	uint32_t x, y;
	uint16_t dif;
	uint16_t error;
	uint16_t x_divide, y_divide;
	uint16_t ErrorOptimal = 0xFFFF;
	int32_t N;

	//CGU_EntityConnect(CGU_CLKSRC_PLL0_AUDIO, CGU_BASE_APB1);
	//CGU_EntityConnect(CGU_CLKSRC_GP_CLKIN, CGU_BASE_APB1);
	//CGU_EntityConnect(CGU_CLKSRC_GP_CLKIN, CGU_BASE_APLL);
	i2sPclk = CGU_GetPCLKFrequency(CGU_PERIPHERAL_I2S);
	wordwidth = 16;
	//wordwidth = 16 * 2;
	bitrate = 2 * wordwidth - 1;

	/* Calculate X and Y divider
	 * The MCLK rate for the I2S transmitter is determined by the value
	 * in the I2STXRATE/I2SRXRATE register. The required I2STXRATE/I2SRXRATE
	 * setting depends on the desired audio sample rate desired, the format
	 * (stereo/mono) used, and the data size.
	 * The formula is:
	 * 		I2S_MCLK = PCLK * (X/Y) / 2
	 * We have:
	 * 		I2S_MCLK = Freq * bit_rate * I2Sx->TXBITRATE;
	 * So: (X/Y) = (Freq * bit_rate * I2Sx->TXBITRATE)/PCLK*2
	 * We use a loop function to chose the most suitable X,Y value
	 */

	/* divider is a fixed point number with 16 fractional bits */
	divider = ((uint64_t)(Freq *( bitrate+1) * 2)<<16) / i2sPclk;

	/* find N that make x/y <= 1 -> divider <= 2^16 */
	for(N=64;N>0;N--){
		if((divider*N) < (1<<16)) break;
	}

	if(N == 0) return;

	divider *= N;

	for (y = 255; y > 0; y--) {
		x = y * divider;
		if(x & (0xFF000000)) continue;
		dif = x & 0xFFFF;
		if(dif>0x8000) error = 0x10000-dif;
		else error = dif;
		if (error == 0)
		{
			y_divide = y;
			break;
		}
		else if (error < ErrorOptimal)
		{
			ErrorOptimal = error;
			y_divide = y;
		}
	}
	x_divide = ((uint64_t)y_divide * Freq *( bitrate+1)* N * 2)/i2sPclk;
	if(x_divide >= 256) x_divide = 0xFF;
	if(x_divide == 0) x_divide = 1;
#else
	// 12MHz * (64 / 125) / 2 / 1 / 64 = 48kHz
	uint16_t x_divide = 64;
	uint16_t y_divide = 125;
	int32_t N = 2;
	CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_BASE_APB1);
#endif
	if (TRMode == I2S_TX_MODE)// Transmitter
	{
		I2Sx->TXBITRATE = N - 1;
		//I2Sx->TXBITRATE = 0; // for I2S slave
		I2Sx->TXRATE = y_divide | (x_divide << 8);
	} else //Receiver
	{
		I2Sx->RXBITRATE = N - 1;
		I2Sx->RXRATE = y_divide | (x_divide << 8);
	}
}
