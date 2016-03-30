/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC43xx.h"
#endif

#include <cr_section_macros.h>

#include <stdio.h>

#include "lpc43xx_gpio.h"

#include "lpc43xx_cgu.h"

#include "lpc43xx_i2c.h"
#include "lpc43xx_rgu.h"
#include "lpc43xx_scu.h"
#include "lpc43xx_ssp.h"
#include <lpc43xx_gpdma.h>

#include "receiver.h"

extern uint32_t CGU_ClockSourceFrequency[CGU_CLKSRC_NUM];


volatile uint32_t msTicks; // counter for 1ms SysTicks

#if 0
// ****************
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
	msTicks++;
}
#else
void M0_RIT_OR_WWDT_IRQHandler(void) {
	msTicks++;
	LPC_RITIMER->CTRL =
			(1 << 0)  /* RITINT */
			| (1 << 1)  /* RITENCLR */
			| (1 << 2)  /* RITENBR */
			| (1 << 3)  /* RITEN */
			;
}

void RITConfig() {
	LPC_RITIMER->COMPVAL = 204000*2; // 1000Hz @ 204MHz
	LPC_RITIMER->COUNTER = 0;
	LPC_RITIMER->CTRL =
			(0 << 0)  /* RITINT */
			| (1 << 1)  /* RITENCLR */
			| (1 << 2)  /* RITENBR */
			| (1 << 3)  /* RITEN */
			;
    NVIC_SetPriority(RITIMER_IRQn, 0);
    NVIC_EnableIRQ(RITIMER_IRQn);
}
#endif

// ****************
// systick_delay - creates a delay of the appropriate number of Systicks (happens every 1 ms)
void systick_delay(uint32_t delayTicks) {
	uint32_t currentTicks;

	currentTicks = msTicks;	// read current tick counter
	// Now loop until required number of ticks passes.
	while ((msTicks - currentTicks) < delayTicks);
}


#define RESET_ASSERT	GPIO_ClearValue(0, 1<<14)
#define RESET_NEGATE	GPIO_SetValue(0, 1<<14)
#define CS_LOW			//GPIO_ClearValue(0, 1<<15)
#define CS_HIGH			//GPIO_SetValue(0, 1<<15)
#define DC_CMD			GPIO_ClearValue(1, 1<<11)
#define DC_DATA			GPIO_SetValue(1, 1<<11)
#define LED_ON			GPIO_SetValue(3, 1<<5)
#define LED_OFF			GPIO_ClearValue(3, 1<<5)

void
ssp_senddata(int x)
{
	SSP_SendData(LPC_SSP1, x);
	while (LPC_SSP1->SR & SSP_SR_BSY)
		;
}

void
ssp_senddata16(int x)
{
	uint32_t CR0 = LPC_SSP1->CR0;
	LPC_SSP1->CR0 = (CR0 & 0xfff0) | SSP_DATABIT_16;
	SSP_SendData(LPC_SSP1, x);
	while (LPC_SSP1->SR & SSP_SR_BSY)
		;
	LPC_SSP1->CR0 = CR0;
}

void
ssp_databit8(void)
{
	LPC_SSP1->CR0 = (LPC_SSP1->CR0 & 0xfff0) | SSP_DATABIT_8;
}

void
ssp_databit16(void)
{
	LPC_SSP1->CR0 = (LPC_SSP1->CR0 & 0xfff0) | SSP_DATABIT_16;
}

void
spi_init()
{
	SSP_CFG_Type ssp_config;

	scu_pinmux(1, 3, SSP_IO, FUNC5); // SSP1_MISO
	scu_pinmux(1, 4, SSP_IO, FUNC5); // SSP1_MOSI
	scu_pinmux(1, 20, SSP_IO, FUNC1); // SSP1_SSEL
	//scu_pinmux(1, 20, SSP_IO, FUNC0); // CS <- G0_15
	scu_pinmux(15, 4, CLK_OUT, FUNC0); // SSP1_SCK
	GPIO_SetDir(3, 1<<5, 1); // LED
	GPIO_SetDir(1, 1<<11, 1); // D/C
	GPIO_SetDir(0, 1<<14, 1); // RESET
	//GPIO_SetDir(0, 1<<15, 1); // CS

	SSP_ConfigStructInit(&ssp_config);
	ssp_config.ClockRate = 30000000; // 30MHz
	SSP_Init(LPC_SSP1, &ssp_config);

	SSP_Cmd(LPC_SSP1, ENABLE);
}

void
spi_test()
{
	while (1) {
		ssp_senddata(0xf0);
		ssp_senddata(0x55);
		CS_LOW;
		GPIO_SetValue(3, 1<<5);
		GPIO_SetValue(1, 1<<11);
		GPIO_SetValue(0, 1<<14);
		systick_delay(10);
		ssp_senddata(0xaa);
		ssp_senddata(0x0f);
		CS_HIGH;
		GPIO_ClearValue(3, 1<<5);
		GPIO_ClearValue(1, 1<<11);
		GPIO_ClearValue(0, 1<<14);
		systick_delay(20);
	}
}

void
send_command(uint8_t cmd, int len, const uint8_t *data)
{
	CS_LOW;
	DC_CMD;
	ssp_senddata(cmd);
	DC_DATA;
	while (len-- > 0) {
		ssp_senddata(*data++);
	}
	//CS_HIGH;
}

void
send_command16(uint8_t cmd, int data)
{
	CS_LOW;
	DC_CMD;
	ssp_senddata(cmd);
	DC_DATA;
	ssp_senddata16(data);
	CS_HIGH;
}

const uint8_t ili9341_init_seq[] = {
		// cmd, len, data...,
		// Power control B
		0xCF, 3, 0x00, 0x83, 0x30,
		// Power on sequence control
		0xED, 4, 0x64, 0x03, 0x12, 0x81,
		//0xED, 4, 0x55, 0x01, 0x23, 0x01,
		// Driver timing control A
		0xE8, 3, 0x85, 0x01, 0x79,
		//0xE8, 3, 0x84, 0x11, 0x7a,
		// Power control A
		0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
		// Pump ratio control
		0xF7, 1, 0x20,
		// Driver timing control B
		0xEA, 2, 0x00, 0x00,
		// POWER_CONTROL_1
		0xC0, 1, 0x26,
		// POWER_CONTROL_2
		0xC1, 1, 0x11,
		// VCOM_CONTROL_1
		0xC5, 2, 0x35, 0x3E,
		// VCOM_CONTROL_2
		0xC7, 1, 0xBE,
		// MEMORY_ACCESS_CONTROL
		//0x36, 1, 0x48 + 0x20,
		0x36, 1, 0x28,
		// COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
		0x3A, 1, 0x55,
		// Frame Rate
		0xB1, 2, 0x00, 0x1B,
		// Gamma Function Disable
		0xF2, 1, 0x08,
		// gamma set for curve 01/2/04/08
		0x26, 1, 0x01,
		// positive gamma correction
		0xE0, 15, 0x1F,  0x1A,  0x18,  0x0A,  0x0F,  0x06,  0x45,  0x87,  0x32,  0x0A,  0x07,  0x02,  0x07, 0x05,  0x00,
		// negativ gamma correction
		0xE1, 15, 0x00,  0x25,  0x27,  0x05,  0x10,  0x09,  0x3A,  0x78,  0x4D,  0x05,  0x18,  0x0D,  0x38, 0x3A,  0x1F,

		// Column Address Set
	    0x2A, 4, 0x00, 0x00, 0x01, 0x3f, // width 320
	    // Page Address Set
	    0x2B, 4, 0x00, 0x00, 0x00, 0xef, // height 240

		// entry mode
		0xB7, 1, 0x06,
		// display function control
		0xB6, 4, 0x0A, 0x82, 0x27, 0x00,
		// sleep out
		0x11, 0,
		0 // sentinel
};

void
ili9341_init()
{
	DC_DATA;
	RESET_ASSERT;
	systick_delay(1);
	RESET_NEGATE;

	send_command(0x01, 0, NULL); // SW reset
	systick_delay(5);
	send_command(0x28, 0, NULL); // display off

	const uint8_t *p;
	for (p = ili9341_init_seq; *p; ) {
		send_command(p[0], p[1], &p[2]);
		p += 2 + p[1];
	}

	systick_delay(100);
	send_command(0x29, 0, NULL); // display on
	LED_ON;
}

void ili9341_pixel(int x, int y, int color)
{
	uint8_t xx[4] = { x >> 8, x, (x+1) >> 8, (x+1) };
	uint8_t yy[4] = { y >> 8, y, (y+1) >> 8, (y+1) };
	uint8_t cc[2] = { color >> 8, color };
	send_command(0x2A, 4, xx);
    send_command(0x2B, 4, yy);
    send_command(0x2C, 2, cc);
    //send_command16(0x2C, color);
}


void
ili9341_test()
{
	//while (1) {
		int x, y;
		for (y = 0; y < 320; y++) {
			for (x = 0; x < 240; x++) {
				ili9341_pixel(x, y, (y<<8)|x);
			}
		}

	//	systick_delay(100);
	//}
}

uint16_t spi_buffer[2048];

void
spi_dma_setup()
{
	GPDMA_Init();
	LPC_GPDMA->CONFIG |= 0x01 << 1;  /* Enable DMA channels, little endian */
	while ( !(LPC_GPDMA->CONFIG & (0x01 << 1)) );
	//NVIC_EnableIRQ(DMA_IRQn);
}

void
spi_dma_transfer(void *data, int length)
{
#if 0
	GPDMA_Channel_CFG_Type ssp_dma_cfg;
	ssp_dma_cfg.ChannelNum = 0;
	ssp_dma_cfg.SrcMemAddr = data;
	ssp_dma_cfg.DstConn = GPDMA_CONN_SSP1_Tx;
	ssp_dma_cfg.DstMemAddr = &LPC_SSP1->DR;
	ssp_dma_cfg.TransferSize = length;
	ssp_dma_cfg.TransferWidth = GPDMA_WIDTH_HALFWORD;
	//ssp_dma_cfg.TransferWidth = GPDMA_WIDTH_WORD;
	//ssp_dma_cfg.TransferType = GPDMA_TRANSFERTYPE_M2P_CONTROLLER_PERIPHERAL;
	ssp_dma_cfg.TransferType = GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA;
	ssp_dma_cfg.DMALLI = 0;
	GPDMA_Setup(&ssp_dma_cfg);
	GPDMA_ChannelCmd(0, ENABLE);
#endif

#define GPDMA_SSP1_TX_CHANNEL 12
	//LPC_CREG->DMAMUX &= ~3<<(GPDMA_SSP1_TX_CHANNEL*2);
	//LPC_CREG->DMAMUX |= 0<<(GPDMA_SSP1_TX_CHANNEL*2);
	LPC_GPDMA->C1SRCADDR = (uint32_t)data;
	LPC_GPDMA->C1DESTADDR = (uint32_t)&LPC_SSP1->DR;
	LPC_GPDMA->C1LLI = 0;
	LPC_GPDMA->C1CONTROL = (length >> 1) |      // Transfersize (does not matter when flow control is handled by peripheral)
                           (0x2 << 12)  |          // Source Burst Size
                           (0x0 << 15)  |          // Destination Burst Size
                           (0x2 << 18)  |          // Source width // 32 bit width
                           (0x1 << 21)  |          // Destination width   // 16 bits
                           (0x0 << 24)  |          // Source AHB master 0 / 1
                           (0x1 << 25)  |          // Dest AHB master 0 / 1
                           (0x1 << 26)  |          // Source increment(LAST Sample)
                           (0x0 << 27)  |          // Destination increment
                           (0x1UL << 31);          // Terminal count interrupt disabled
	LPC_GPDMA->C1CONFIG  =  (0x1)        |          // Enable bit
						  (0x0 << 1) |  // SRCPERIPHERAL - memory
						  (GPDMA_SSP1_TX_CHANNEL << 6)   |          // Destination peripheral - memory - no setting
						  (0x1 << 11)  |          // Flow control - peripheral to memory - DMA control
//                        (0x5 << 11)  |          // Flow control - peripheral to memory - peripheral control
						  (0x0 << 14)  |          // Int error mask
						  (0x1 << 15);            // ITC - term count error mask
	SSP_DMACmd(LPC_SSP1, SSP_DMA_TX, ENABLE);

	// software trigger
	LPC_GPDMA->SOFTBREQ = 1<<GPDMA_SSP1_TX_CHANNEL;
}

void
spi_dma_sync()
{
	while (!(LPC_GPDMA->INTTCSTAT & GPDMA_DMACIntTCStat_Ch(1)))
		;
	LPC_GPDMA->INTTCCLEAR = GPDMA_DMACIntTCClear_Ch(1);
	//while (LPC_SSP1->SR & SSP_SR_BSY)
	//	;
}

void
spi_dma_stop()
{
	SSP_DMACmd(LPC_SSP1, SSP_DMA_TX, DISABLE);
}

#if 0
void DMA_IRQHandler (void)
{
  if (LPC_GPDMA->INTERRSTAT & GPDMA_DMACIntErrStat_Ch(1))
  {
    LPC_GPDMA->INTERRCLR = GPDMA_DMACIntErrClr_Ch(1);
  }

  if (LPC_GPDMA->INTTCSTAT & GPDMA_DMACIntTCStat_Ch(1))
  {
	LPC_GPDMA->INTTCCLEAR = GPDMA_DMACIntTCClear_Ch(1);
  }
}
#endif

void
ili9341_dma_test()
{
	int sx = 40, ex = 200;
	uint8_t xx[4] = { sx >> 8, sx, ex >> 8, ex };
	int x;
	int y = 0;
	spi_dma_setup();

	//while (1) {
		for (y = 0; y < 320; y++) {
			int sy = y, ey = sy + 1;
			uint8_t yy[4] = { sy >> 8, sy, ey >> 8, ey };
			for (x = 0; x < 160; x++) {
				//int c = ((~x & 0x7) ? 0x7 : 0x0) | (~x & ~0x7);
				//spi_buffer[x] = (y<<8)|(c&0xff);
				spi_buffer[x] = (x<<8)|y;
			}

			ssp_databit8();
			send_command(0x2A, 4, xx);
			send_command(0x2B, 4, yy);
			send_command(0x2C, 3, xx);
			ssp_databit16();
			spi_dma_transfer(spi_buffer, 160);
			spi_dma_sync();
		}
		//systick_delay(10);
	//}
	//spi_dma_stop();
}


extern const UNS_16 x5x7_bits [];
extern const uint32_t numfont20x24[][24];
extern const uint32_t numfont32x24[][24];

typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t scaley;
	uint16_t slide;
	const uint32_t *bitmap;
} font_t;

const font_t NF20x24 = { 20, 24, 1, 24, (const uint32_t *)numfont20x24 };
const font_t NF32x24 = { 32, 24, 1, 24, (const uint32_t *)numfont32x24 };
const font_t NF32x48 = { 32, 48, 2, 24, (const uint32_t *)numfont32x24 };

void
ili9341_drawfont_dma(uint8_t ch, const font_t *font, int x, int y, uint16_t fg, uint16_t bg)
{
	int ex = x + font->width-1;
	int ey = y + font->height-1;
	uint8_t xx[4] = { x >> 8, x, ex >> 8, ex };
	uint8_t yy[4] = { y >> 8, y, ey >> 8, ey };
	uint16_t *buf = spi_buffer;
	uint32_t bits;
	const uint32_t *bitmap = &font->bitmap[font->slide * ch];
	int c, r, j;

	for (c = 0; c < font->slide; c++) {
		for (j = 0; j < font->scaley; j++) {
			bits = bitmap[c];
			for (r = 0; r < font->width; r++) {
				*buf++ = (0x80000000UL & bits) ? fg : bg;
				bits <<= 1;
			}
		}
	}
	ssp_databit8();
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
	ssp_databit16();
	//spi_dma_transfer(spi_buffer, 4);
	//spi_dma_sync();
	spi_dma_transfer(spi_buffer, buf - spi_buffer);
	spi_dma_sync();
}

void
ili9341_drawchar_dma(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg)
{
	int ex = x + 4;
	int ey = y + 7;
	uint8_t xx[4] = { x >> 8, x, ex >> 8, ex };
	uint8_t yy[4] = { y >> 8, y, ey >> 8, ey };
	uint16_t *buf = spi_buffer;

	uint16_t bits;
	int c, r;
	for(c = 0; c < 7; c++) {
		bits = x5x7_bits[(ch * 7) + c];
		for (r = 0; r < 5; r++) {
			*buf++ = (0x8000 & bits) ? fg : bg;
			bits <<= 1;
		}
	}
	ssp_databit8();
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
	ssp_databit16();
	spi_dma_transfer(spi_buffer, 35);
	spi_dma_sync();
}

void
ili9341_drawstring_dma(char *str, int x, int y, uint16_t fg, uint16_t bg)
{
	spi_dma_setup();
	while (*str) {
		ili9341_drawchar_dma(*str, x, y, fg, bg);
		x += 5;
		str++;
	}
}

void
ili9341_drawchar(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg)
{
	uint16_t bits;
	int c, r;
	for(c = 0; c < 7; c++) {
		bits = x5x7_bits[(ch * 7) + c];
		for (r = 0; r < 5; r++) {
			ili9341_pixel(x+r, y+c, (0x8000 & bits) ? fg : bg);
			bits <<= 1;
		}
	}
}

void
ili9341_drawstring(char *str, int x, int y, uint16_t fg, uint16_t bg)
{
	while (*str) {
		ili9341_drawchar(*str, x, y, fg, bg);
		x += 5;
		str++;
	}
}


void
ili9341_bulk_test()
{
	int x, y;
	//while (1) {
		for(y = 0; y < 320; y++) {
			int sx = 0, ex = 240;
			int sy = y, ey = y+1;
			uint8_t xx[4] = { sx >> 8, sx, ex >> 8, ex };
			uint8_t yy[4] = { sy >> 8, sy, ey >> 8, ey };
			ssp_databit8();
			send_command(0x2A, 4, xx);
			send_command(0x2B, 4, yy);
			send_command(0x2C, 0, NULL);
			ssp_databit16();
			for (x = 0; x < 240; x++)
				ssp_senddata((y<<8)|x);
		}
		//systick_delay(100);
	//}
}

static Status i2clcd_data(uint8_t data)
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

static void i2clcd_str(char *p)
{
	while (*p) {
		i2clcd_data(*p++);
	}
}

static Status i2clcd_cmd(uint8_t cmd)
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

static void i2clcd_pos(uint8_t x, uint8_t y)
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

// result is 8.8 format
static inline uint16_t
log2_q31(uint32_t x)
{
	uint32_t mask = 0xffff0000;
	uint16_t bit = 16;
	uint16_t y = 31-15;
	uint8_t i;

	if (x == 0)
		return 0;
	// 16
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 8
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 4
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 2
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	bit >>= 1;
	mask <<= bit;
	// 1
	if ((x & mask) == 0) {
		x <<= bit;
		y -= bit;
	}
	// msb should be 1. take next 8 bits.
	i = (x >> 23) & 0xff;
	// lookup logarythm table
	return (y << 8) | i;
}

void draw_samples()
{
	int sx = 0, ex = 320;
	int sy = 180, ey = 208;
	uint8_t xx[4] = { sx >> 8, sx, ex >> 8, ex };
	uint8_t yy[4] = { sy >> 8, sy, ey >> 8, ey };
	ssp_databit8();
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
	ssp_databit16();
	spi_dma_transfer(ANALYZEINFO->buffer, ANALYZE_BUFFER_SIZE/sizeof(uint16_t));
	spi_dma_sync();
}

void
draw_block_32x32(int x, int y, uint16_t *buf)
{
	int ex = x + 32-1;
	int ey = y + 32-1;
	uint8_t xx[4] = { x >> 8, x, ex >> 8, ex };
	uint8_t yy[4] = { y >> 8, y, ey >> 8, ey };
	ssp_databit8();
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
	ssp_databit16();
	spi_dma_transfer(buf, 32*32);
	spi_dma_sync();
}

arm_cfft_radix4_instance_q31 cfft_inst;

void
show_spectrogram()
{
	q31_t *buf = ANALYZEINFO->buffer;
	arm_cfft_radix4_q31(&cfft_inst, buf);
	arm_cmplx_mag_q31(buf, buf, 1024);
	draw_samples();
	//return;
	q31_t *p = buf;
	//int i = 511;
	//int stride = -1;
	int i = 512;
	int stride = -1;
	uint16_t (*block)[32] = spi_buffer;
	int sx, x, y;
	for (sx = 0; sx < 320; sx += 32) {
		for (x = 0; x < 32; x++) {
			int v = log2_q31(p[i]) >> 8;
			for (y = 0; y < 32; y++)
				block[31-y][x] = v < y ? 0 : 0xffff;
			i += stride;
		}
		draw_block_32x32(sx, 208, block);
	}
}

volatile int count;

// event handler sent from M4 core
void M0_M4CORE_IRQHandler(void) {
	LPC_CREG->M4TXEVENT = 0;
	ili9341_drawfont_dma(count++ % 10, &NF32x48, 0, 140, 0xffff, 0x0000);
	if (ANALYZEINFO->semaphoe) {
		show_spectrogram();
		ANALYZEINFO->semaphoe = 0;
	}
}

volatile int8_t pending_launch = 0;

int main(void) {
	char buf[16];
	int i;

	//while (pending_launch)
	//	;

    CGU_ClockSourceFrequency[CGU_CLKSRC_XTAL_OSC] = 12000000;
    CGU_ClockSourceFrequency[CGU_CLKSRC_PLL1] = 204000000;

    //printf("Hello M0APP\n");

	// Setup SysTick Timer to interrupt at 1 msec intervals
	//SysTick_Config(CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE)/1000);
    RITConfig();

	systick_delay(1000);

	arm_cfft_radix4_init_q31(&cfft_inst, 1024, FALSE, TRUE);

    //RGU_SoftReset(RGU_SIG_I2C0);

    //NVIC_SetPriority(DMA_IRQn,   ((0x01<<3)|0x01));

	scu_pinmux(1, 7, GPIO_PUP, FUNC0); // GPIO1-0
	scu_pinmux(1, 8, GPIO_PUP, FUNC0); // GPIO1-1
	scu_pinmux(1, 9, GPIO_PUP, FUNC0); // GPIO1-2

	spi_init();
	//spi_test();
#if 1
	ili9341_init();
	//ili9341_test();
	//ili9341_dma_test();
	//ili9341_bulk_test();
	//ili9341_drawstring_dma("Hello", 100, 100, 0xffff, 0x0000);
	for (i = 0; i < 10; i++)
		ili9341_drawfont_dma(i, &NF20x24, i*20, 20, 0xffe0, 0x0000);
	for (i = 0; i < 5; i++)
		ili9341_drawfont_dma(i, &NF32x48, i*32, 44, 0xffff, 0x0000);
	for (i = 0; i < 5; i++)
		ili9341_drawfont_dma(i+5, &NF32x48, i*32, 92, 0xffff, 0x0000);

    NVIC_SetPriority(M0_M4CORE_IRQn, 4);
    NVIC_EnableIRQ(M0_M4CORE_IRQn);
#if 0
	while (1) {
		for (i = 0; i < 10; i++) {
			ili9341_drawfont_dma(i, &NF32x48, 0, 140, 0xffff, 0x0000);
			systick_delay(200);
		}
	}
#endif
#endif
#if 0

	GPIO_SetDir(0,1<<8, 1);
	GPIO_ClearValue(0,1<<8);

	GPIO_SetDir(1,0x7, 0);
	GPIO_SetDir(1,1<<3, 1);
	GPIO_SetDir(1,1<<4, 1);
	GPIO_ClearValue(1,1<<3);
	GPIO_ClearValue(1,1<<4);
#endif
	//i2clcd_init();
	//i2clcd_str("Hello");

    // Force the counter to be placed into memory
    //volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
#if 0
#if 1
		int status = btn_check();
		if (status == 0)
			continue;
		if (status & EVT_BUTTON_SINGLE_CLICK)
			i += 10;
		if (status & EVT_BUTTON_DOUBLE_CLICK)
			i -= 10;
		if (status & EVT_BUTTON_DOWN_LONG)
			i = 0;
		if (status & ENCODER_UP)
			i++;
		if (status & ENCODER_DOWN)
			i--;
#else
		systick_delay(500);
		GPIO_SetValue(0,1<<8);
		GPIO_SetValue(1,1<<3);
		systick_delay(500);
		GPIO_ClearValue(1,1<<3);
		GPIO_SetValue(1,1<<4);
		systick_delay(500);
		GPIO_SetValue(1,1<<3);
		systick_delay(500);
		GPIO_ClearValue(0,1<<8);
		GPIO_ClearValue(1,1<<3);
		GPIO_ClearValue(1,1<<4);
        i++ ;
#endif
#endif
#if 0
		sprintf(buf, "%d  ", i);
		i2clcd_pos(0, 1);
		//i2clcd_data(0x30 + (i % 10));
		i2clcd_str(buf);
        //value = GPIO_ReadValue(1);
        //sprintf(buf, "%d", value & 0x7);
        //i2clcd_data(' ');
        //i2clcd_str(buf);
#endif
    }
    return 0 ;
}

#ifdef DEBUG
void check_failed(uint8_t *file, uint32_t line)
{
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

/* Infinite loop */
while(1);
}
#endif
