#ifdef __USE_CMSIS
#include "LPC43xx.h"
#endif

#include <stdio.h>
#include "lpc43xx_gpio.h"
#include "lpc43xx_cgu.h"
#include "lpc43xx_i2c.h"
#include "lpc43xx_rgu.h"
#include "lpc43xx_scu.h"

#include "fmreceiver.h"


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
#define TP_MAX		10
#define FREQ_STEP	100000

static struct {
	enum { GAIN, CHANNEL, FREQ, TESTP, MODE_MAX } mode;
	int gain;
	int channel;
	float32_t freq;
	int tp;
} uistat;

static float32_t channel_freqs[CHANNEL_MAX] = {
		80.4e6f,
		82.5e6f,
		85.2e6f,
		80.0e6f,
		80.0e6f,
		80.0e6f,
		80.0e6f,
		80.0e6f,
};

extern volatile struct {
	uint16_t write_current;
	uint16_t write_total;
	uint16_t read_total;
	uint16_t read_current;
	uint16_t rebuffer_count;
} audio_state;

extern struct {
	uint32_t last;
	int32_t carrier;
} fm_demod_state;

extern struct {
	float32_t carrier_i;
	float32_t carrier_q;
	float32_t step_cos;
	float32_t step_sin;
	float32_t delta_cos[12];
	float32_t delta_sin[12];
	int16_t corr;
} stereo_separate_state;


void
ui_update()
{
	char buf[16];
	switch (uistat.mode) {
	case GAIN:
		sprintf(buf, "Vol:%ddB", uistat.gain + AUDIO_GAIN_REF);
		break;
	case FREQ:
		sprintf(buf, "%2.1fMHz", uistat.freq / 1000000);
		break;
	case CHANNEL:
		sprintf(buf, "Ch%d %2.1f", uistat.channel, uistat.freq / 1000000);
		break;
	case TESTP:
		switch (uistat.tp) {
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
			sprintf(buf, "CAR:%d", fm_demod_state.carrier);
			break;
		case 8:
			sprintf(buf, "ST:%d", stereo_separate_state.corr);
			break;
		default:
			sprintf(buf, "undef");
			break;
		}
		break;
	default:
		return;
	}
	i2clcd_pos(0, 1);
	i2clcd_str(buf);
	i2clcd_str("        ");
}

void
ui_init()
{
	scu_pinmux(1, 7, GPIO_PUP, FUNC0); // GPIO1-0
	scu_pinmux(1, 8, GPIO_PUP, FUNC0); // GPIO1-1
	scu_pinmux(1, 9, GPIO_PUP, FUNC0); // GPIO1-2

	GPIO_SetDir(0,1<<8, 1);
	GPIO_ClearValue(0,1<<8);

	GPIO_SetDir(1,0x7, 0);
	GPIO_SetDir(1,1<<3, 1);
	GPIO_SetDir(1,1<<4, 1);
	GPIO_ClearValue(1,1<<3);
	GPIO_ClearValue(1,1<<4);

	i2clcd_init();
	i2clcd_str("HelloSDR");

	uistat.mode = GAIN;
	uistat.gain = 0;
	uistat.channel = 0;
	uistat.freq = 82500000;
	uistat.tp = 0;
	ui_update();

	nco_set_frequency(uistat.freq);
	audio_set_gain(uistat.gain);
}

void
ui_process()
{
	int status = btn_check();
	if (status != 0) {
#if 0
		if (status & EVT_BUTTON_SINGLE_CLICK)
			value += 10;
		if (status & EVT_BUTTON_DOUBLE_CLICK)
			value -= 10;
		if (status & EVT_BUTTON_DOWN_LONG)
			value = 0;
#endif
		if (status & EVT_BUTTON_SINGLE_CLICK) {
			uistat.mode = (uistat.mode + 1) % MODE_MAX;
		} else if (uistat.mode == GAIN) {
			if ((status & ENCODER_UP) && uistat.gain < AUDIO_GAIN_MAX)
				uistat.gain++;
			if ((status & ENCODER_DOWN) && uistat.gain > AUDIO_GAIN_MIN)
				uistat.gain--;
			audio_set_gain(uistat.gain);
		} else if (uistat.mode == FREQ) {
			if ((status & ENCODER_UP))
				uistat.freq += FREQ_STEP;
			if ((status & ENCODER_DOWN))
				uistat.freq -= FREQ_STEP;
			nco_set_frequency(uistat.freq);
		} else if (uistat.mode == CHANNEL) {
			if ((status & ENCODER_UP) && uistat.channel < CHANNEL_MAX) {
				uistat.channel++;
				uistat.freq = channel_freqs[uistat.channel];
				nco_set_frequency(uistat.freq);
			}
			if ((status & ENCODER_DOWN) && uistat.channel > 0) {
				uistat.channel--;
				uistat.freq = channel_freqs[uistat.channel];
				nco_set_frequency(uistat.freq);
			}
		} else if (uistat.mode == TESTP) {
			if ((status & ENCODER_UP) && uistat.tp < TP_MAX)
				uistat.tp++;
			if ((status & ENCODER_DOWN) && uistat.tp > 0)
				uistat.tp--;
		}
		ui_update();

#if 0
		if (status & EVT_BUTTON_SINGLE_CLICK) {
			//uint32_t *p = (uint32_t *)CAPTUREBUFFER;
			//uint32_t *p = (uint32_t *)(CAPTUREBUFFER + 4094);
			//uint32_t *p = (uint32_t *)(CAPTUREBUFFER + 16384);
			//uint32_t *p = (uint32_t *)0x20004000;
			uint32_t *p = (uint32_t *)0x20008000;
			//uint32_t *p = (uint32_t *)(CAPTUREBUFFER + 32768);
	    	sprintf(buf, "%08x", p[0]);
	    	//sprintf(buf, "%08x%08x", LPC_VADC->FIFO_OUTPUT[0], LPC_VADC->FIFO_OUTPUT[1]);
			//sprintf(buf, "%d  ", capture_count);
			i2clcd_pos(0, 1);
			i2clcd_str(buf);
			LPC_GPDMA->C0CONFIG |= (1 << 18); //halt further requests

			generate_test_tone(value);
		}
#endif
	} else if (capture_count % 512 == 0)
		ui_update();

    systick_delay(1);
}
