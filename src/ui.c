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

#include <stdio.h>
#include "lpc43xx_gpio.h"
#include "lpc43xx_cgu.h"
#include "lpc43xx_i2c.h"
#include "lpc43xx_rgu.h"
#include "lpc43xx_scu.h"

#include "receiver.h"


volatile uint32_t msTicks; // counter for 1ms SysTicks

// ****************
//  SysTick_Handler - just increment SysTick counter
__attribute__ ((section(".after_vectors")))
void SysTick_Handler(void) {
	msTicks++;
}

// ****************
// systick_delay - creates a delay of the appropriate number of Systicks (happens every 1 ms)
void systick_delay(uint32_t delayTicks) {
	uint32_t currentTicks;

	currentTicks = msTicks;	// read current tick counter
	// Now loop until required number of ticks passes.
	while ((msTicks - currentTicks) < delayTicks);
}

Status i2clcd_data(uint8_t data)
{
	I2C_M_SETUP_Type setup;
    uint8_t buf[2];
    buf[0] = 0x40;
    buf[1] = data;
    setup.sl_addr7bit = 0x7c >> 1;
    setup.tx_data = buf;
    setup.tx_length = 2;
    setup.rx_data = NULL;
    setup.rx_length = 0;
    setup.retransmissions_max = 3;
    return I2C_MasterTransferData(LPC_I2C0, &setup, I2C_TRANSFER_POLLING);
}

void i2clcd_str(char *p)
{
	while (*p) {
		i2clcd_data(*p++);
	}
}

Status i2clcd_cmd(uint8_t cmd)
{
	I2C_M_SETUP_Type setup;
	Status s;
    uint8_t buf[2];
    buf[0] = 0;
    buf[1] = cmd;
    setup.sl_addr7bit = 0x7c >> 1;
    setup.tx_data = buf;
    setup.tx_length = 2;
    setup.rx_data = NULL;
    setup.rx_length = 0;
    setup.retransmissions_max = 3;
    s = I2C_MasterTransferData(LPC_I2C0, &setup, I2C_TRANSFER_POLLING);
    if (s != SUCCESS) {
    	//printf("I2C Failed\n");
    }
    return s;
}

void i2clcd_pos(uint8_t x, uint8_t y)
{
	i2clcd_cmd(0x80 | (0x40 * y) | x);
}

void i2clcd_init()
{
    I2C_Init(LPC_I2C0, 10000);
    I2C_Cmd(LPC_I2C0, ENABLE);
    systick_delay(40);
    i2clcd_cmd(0x38);
    i2clcd_cmd(0x39);
    i2clcd_cmd(0x14);
    i2clcd_cmd(0x70);
    i2clcd_cmd(0x56);
    i2clcd_cmd(0x6c);
    systick_delay(200);
    i2clcd_cmd(0x38);
    i2clcd_cmd(0x0c);
    i2clcd_cmd(0x01);
    systick_delay(2);
}

#define NO_EVENT					0
#define EVT_BUTTON_SINGLE_CLICK		0x01
#define EVT_BUTTON_DOUBLE_CLICK		0x02
#define EVT_BUTTON_DOWN_LONG		0x04
#define ENCODER_UP					0x10
#define ENCODER_DOWN				0x20

#define BUTTON_DOWN_LONG_TICKS		2000
#define BUTTON_DOUBLE_TICKS			500
#define BUTTON_DEBOUNCE_TICKS		10

#define ENCODER0 0x01
#define ENCODER1 0x02
#define BUTTON0  0x04

static uint8_t last_button = 0b111;
static uint32_t last_button_down_ticks;

int btn_check()
{
	int cur_button = GPIO_ReadValue(1) & 0b00000111;
	int status = 0;
	int changed = last_button ^ cur_button;
	if (changed & BUTTON0) {
		if (msTicks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS) {
			if (cur_button & BUTTON0) {
				// button released
				status |= EVT_BUTTON_SINGLE_CLICK;
			} else {
				// button pushed
				if (msTicks < last_button_down_ticks + BUTTON_DOUBLE_TICKS) {
					status |= EVT_BUTTON_DOUBLE_CLICK;
				} else {
					last_button_down_ticks = msTicks;
				}
			}
		}
	} else {
		// button unchanged
		if (!(cur_button & BUTTON0)
			&& msTicks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
			status |= EVT_BUTTON_DOWN_LONG;
		}
	}

	if ((changed & ENCODER0) && !(cur_button & ENCODER0)) {
		if (msTicks > last_button_down_ticks + 1) {
			int e = cur_button & 0x03;
			//printf("%d\n", e);
			if (e == 0)
				status |= ENCODER_UP;
			else if (e == 2)
				status |= ENCODER_DOWN;
			last_button_down_ticks = msTicks;
		}
	}

	last_button = cur_button;
	return status;
}

#define CHANNEL_MAX	9
#define TP_MAX		16
#define FREQ_STEP	100000

static float32_t channel_freqs[CHANNEL_MAX] = {
		80.4e6f,
		82.5e6f,
		85.2e6f,
		80.0e6f,
		81.3e6f,
		76.1e6f,
		77.1e6f,
		145e6f,
};

extern audio_state_t audio_state;
extern fm_demod_state_t fm_demod_state;
extern stereo_separate_state_t stereo_separate_state;

void
ui_update()
{
	char buf[16];
	switch (UISTAT->mode) {
	case GAIN:
		if (UISTAT->gain < -6)
			sprintf(buf, "Vol:mute");
		else
			sprintf(buf, "Vol:%ddB", UISTAT->gain);
		break;
	case FREQ:
		sprintf(buf, "%2.1fMHz", UISTAT->freq / 1000000);
		break;
	case CHANNEL:
		sprintf(buf, "Ch%d %2.1f", UISTAT->channel, UISTAT->freq / 1000000);
		break;
	case TESTP:
		switch (UISTAT->tp) {
		case 0:
			sprintf(buf, "CAP:%04x", *(uint16_t*)CAPTUREBUFFER0);
			break;
		case 1:
			sprintf(buf, "CIC:%04x", *(uint16_t*)0x10080040);
			break;
		case 2:
			sprintf(buf, "FIR:%04x", *(uint16_t*)0x10088000);
			break;
		case 3:
			sprintf(buf, "DEM:%04x", *(uint16_t*)0x10089100);
			break;
		case 4:
			sprintf(buf, "AUD:%04x", *(uint16_t*)0x1008A000);
			break;
		case 5:
			sprintf(buf, "RBF:%d", audio_state.rebuffer_count);
			break;
		case 6:
		{
			uint16_t d = audio_state.write_current - audio_state.read_current;
			d %= AUDIO_BUFFER_SIZE / 2;
			sprintf(buf, "D:%d", d);
			break;
		}
		case 7:
			sprintf(buf, "CR:%d", fm_demod_state.carrier);
			break;
		case 8:
			sprintf(buf, "ST:%d", stereo_separate_state.corr);
			break;
		case 9:
			sprintf(buf, "SAV:%d", stereo_separate_state.corr_ave);
			break;
		case 10:
			sprintf(buf, "SSD:%d", stereo_separate_state.corr_std);
			break;
#if 0
		case 11:
			sprintf(buf, "NA:%f", stereo_separate_state.carrier_i*stereo_separate_state.carrier_i+stereo_separate_state.carrier_q*stereo_separate_state.carrier_q);
			break;
		case 12:
			sprintf(buf, "SA:%f", stereo_separate_state.step_cos*stereo_separate_state.step_cos+stereo_separate_state.step_sin*stereo_separate_state.step_sin);
			break;
#else
		case 11:
			sprintf(buf, "SI%d", stereo_separate_state.sdi);
			break;
		case 12:
			sprintf(buf, "SQ%d", stereo_separate_state.sdq);
			break;
#endif
		case 13:
			sprintf(buf, "OFS:%04x", cic_i.dc_offset & 0xffff);
			break;
		default:
			sprintf(buf, "undef");
			break;
		}
		break;
	case DEBUGMODE:
	    switch (UISTAT->debugmode) {
	    case 0:
			sprintf(buf, "DMA:RUN");
			break;
	    case 1:
			sprintf(buf, "DMA:HALT");
			break;
	    case 2:
			sprintf(buf, "DMA:TONE");
			break;
		}
		break;
	default:
		return;
	}
	i2clcd_pos(0, 1);
	i2clcd_str(buf);
	i2clcd_str("        ");

	if (stereo_separate_state.corr_std < 1000) {
		ROTLED_GREEN();
	} else if (fm_demod_state.carrier > 10000) {
		ROTLED_RED();
	} else {
		ROTLED_OFF();
	}
}

void
ui_init()
{
	scu_pinmux(1, 7, GPIO_PUP, FUNC0); // GPIO1-0
	scu_pinmux(1, 8, GPIO_PUP, FUNC0); // GPIO1-1
	scu_pinmux(1, 9, GPIO_PUP, FUNC0); // GPIO1-2
	LED_INIT();
	ROTLED_INIT();

	i2clcd_init();
	i2clcd_str("HelloSDR");

	UISTAT->mode = GAIN;
	UISTAT->gain = 10;
	UISTAT->channel = 1;
	UISTAT->freq = 82500000;
	UISTAT->modulation = MOD_LSB;
	UISTAT->digit = 5;
	UISTAT->agcmode = 0;
	UISTAT->rfgain = 0;
	UISTAT->spdispmode = SPDISP_CIC;
	UISTAT->tp = 0;
	UISTAT->debugmode = 0;
	ui_update();

	nco_set_frequency(UISTAT->freq);
	audio_set_gain(UISTAT->gain);
}

void
ui_process()
{
	int status = btn_check();
	if (status != 0) {
		if (status & EVT_BUTTON_SINGLE_CLICK) {
			UISTAT->mode = (UISTAT->mode + 1) % MODE_MAX;
		} else if (UISTAT->mode == GAIN) {
			if ((status & ENCODER_UP) && UISTAT->gain < AUDIO_GAIN_MAX)
				UISTAT->gain++;
			if ((status & ENCODER_DOWN) && UISTAT->gain > AUDIO_GAIN_MIN)
				UISTAT->gain--;
			audio_set_gain(UISTAT->gain);
		} else if (UISTAT->mode == FREQ) {
			if ((status & ENCODER_UP))
				UISTAT->freq += FREQ_STEP;
			if ((status & ENCODER_DOWN))
				UISTAT->freq -= FREQ_STEP;
			nco_set_frequency(UISTAT->freq);
		} else if (UISTAT->mode == CHANNEL) {
			if ((status & ENCODER_UP) && UISTAT->channel < CHANNEL_MAX) {
				UISTAT->channel++;
				UISTAT->freq = channel_freqs[UISTAT->channel];
				nco_set_frequency(UISTAT->freq);
			}
			if ((status & ENCODER_DOWN) && UISTAT->channel > 0) {
				UISTAT->channel--;
				UISTAT->freq = channel_freqs[UISTAT->channel];
				nco_set_frequency(UISTAT->freq);
			}
		} else if (UISTAT->mode == SPDISP) {
			if ((status & ENCODER_UP) && UISTAT->spdispmode < SPDISP_MODE_MAX-1) {
				UISTAT->spdispmode++;
			}
			if ((status & ENCODER_DOWN) && UISTAT->spdispmode > 0) {
				UISTAT->spdispmode--;
			}
			SPDISPINFO->ui_update_flag = TRUE;
		} else if (UISTAT->mode == TESTP) {
			if (status & ENCODER_UP)
				UISTAT->tp++;
			if (status & ENCODER_DOWN)
				UISTAT->tp--;
			UISTAT->tp &= TP_MAX-1; // assume 2^n
		} else if (UISTAT->mode == DEBUGMODE) {
			if ((status & ENCODER_UP) && UISTAT->debugmode < 2) {
				UISTAT->debugmode++;
				DMA_HALT();
				if (UISTAT->debugmode == 2) {
					generate_test_tone(1000.0f);
				}
			}
			if ((status & ENCODER_DOWN) && UISTAT->debugmode > 0) {
				UISTAT->debugmode--;
				if (UISTAT->debugmode == 0)
					DMA_RUN();
			}
		}
		SPDISPINFO->ui_update_flag = TRUE;
		ui_update();
	} else if (capture_count % 512 == 0) {
		ui_update();
		//update_adc_dc_offset();
	}

    //systick_delay(1);
}
