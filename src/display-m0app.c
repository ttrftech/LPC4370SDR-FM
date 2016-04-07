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

#define BG_ACTIVE 0x8208
#define BG_NORMAL 0x0000



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
#define BACKLIGHT_ON	GPIO_SetValue(3, 1<<5)
#define BACKLIGHT_OFF	GPIO_ClearValue(3, 1<<5)

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
	BACKLIGHT_ON;
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
spi_dma_transfer(void *data, int length, int noincrement)
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
	LPC_GPDMA->C1CONTROL = (length >> 1) |         // Transfersize (does not matter when flow control is handled by peripheral)
						   (0x2 << 12)  |          // Source Burst Size
                           (0x0 << 15)  |          // Destination Burst Size
                           (0x2 << 18)  |          // Source width // 32 bit width
                           (0x1 << 21)  |          // Destination width   // 16 bits
                           (0x0 << 24)  |          // Source AHB master 0 / 1
                           (0x1 << 25)  |          // Dest AHB master 0 / 1
            (noincrement?0:(0x1 << 26)) |	 	   // Source increment(LAST Sample)
                           (0x0 << 27)  |          // Destination increment
                           (0x1UL << 31);          // Terminal count interrupt disabled
	LPC_GPDMA->C1CONFIG  =  (0x1)       |          // Enable bit
						  (0x0 << 1)    |  	       // SRCPERIPHERAL - memory
		 (GPDMA_SSP1_TX_CHANNEL << 6)  |           // Destination peripheral - memory - no setting
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
			spi_dma_transfer(spi_buffer, 160, 0);
			spi_dma_sync();
		}
		//systick_delay(10);
	//}
	//spi_dma_stop();
}

void
ili9341_draw_bitmap(int x, int y, int w, int h, uint16_t *buf)
{
	int ex = x + w-1;
	int ey = y + h-1;
	uint8_t xx[4] = { x >> 8, x, ex >> 8, ex };
	uint8_t yy[4] = { y >> 8, y, ey >> 8, ey };
	ssp_databit8();
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
	ssp_databit16();
	spi_dma_transfer(buf, w*h, 0);
	spi_dma_sync();
}

void
ili9341_fill(int x, int y, int w, int h, uint16_t color)
{
	uint16_t buf[2] = { color, color }; //32bit buffer
	int ex = x + w-1;
	int ey = y + h-1;
	uint8_t xx[4] = { x >> 8, x, ex >> 8, ex };
	uint8_t yy[4] = { y >> 8, y, ey >> 8, ey };
	ssp_databit8();
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
	ssp_databit16();
	spi_dma_transfer(buf, w*h, 1);
	spi_dma_sync();
}

extern const UNS_16 x5x7_bits [];
extern const uint32_t numfont20x24[][24];
extern const uint32_t numfont32x24[][24];
extern const uint32_t icons48x20[][20*2];

typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t scaley;
	uint16_t slide;
	uint16_t stride;
	const uint32_t *bitmap;
} font_t;

const font_t NF20x24 = { 20, 24, 1, 24, 1, (const uint32_t *)numfont20x24 };
const font_t NF32x24 = { 32, 24, 1, 24, 1, (const uint32_t *)numfont32x24 };
const font_t NF32x48 = { 32, 48, 2, 24, 1, (const uint32_t *)numfont32x24 };
const font_t ICON48x20 = { 48, 20, 1, 40, 2, (const uint32_t *)icons48x20 };

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
	int c, r, j, b;

	for (c = 0; c < font->slide; c += font->stride) {
		for (j = 0; j < font->scaley; j++) {
			int cc = c;
			for (r = 0; r < font->width;) {
				bits = bitmap[cc++];
				for (b = 0; b < 32 && r < font->width; b++,r++) {
					*buf++ = (0x80000000UL & bits) ? fg : bg;
					bits <<= 1;
				}
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
	spi_dma_transfer(spi_buffer, buf - spi_buffer, 0);
	spi_dma_sync();
}

void
ili9341_drawfont_string(char *str, const font_t *font, int x, int y, uint16_t fg, uint16_t bg)
{
	while (*str) {
		char c = *str++;
		if (c >= '0' && c <= '9')
			ili9341_drawfont_dma(c - '0', font, x, y, fg, bg);
		else if (c > 0 && c < 7)
			ili9341_drawfont_dma(c + 9, font, x, y, fg, bg);
		else if (c == '.')
			ili9341_drawfont_dma(10, font, x, y, fg, bg);
		else if (c == '-')
			ili9341_drawfont_dma(11, font, x, y, fg, bg);
		else
			ili9341_fill(x, y, font->width, font->height, bg);
		x += font->width;
	}
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
	spi_dma_transfer(spi_buffer, 35, 0);
	spi_dma_sync();
}

void
ili9341_drawstring_dma(char *str, int x, int y, uint16_t fg, uint16_t bg)
{
	//spi_dma_setup();
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

// result is 8.8 format
static inline uint16_t
log2_q31(uint32_t x)
{
	uint32_t mask = 0xffff0000;
	uint16_t bit = 16;
	uint16_t y = 32;//-15;
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

arm_cfft_radix4_instance_q31 cfft_inst;

//#define mag(r,i) (q31_t)(((q63_t)r*r)>>33)+(q31_t)(((q63_t)i*i)>>33)

void
draw_spectrogram()
{
	q31_t *buf = SPDISPINFO->buffer;
	arm_cfft_radix4_q31(&cfft_inst, buf);
	//arm_cmplx_mag_q31(buf, buf, 1024);
//	arm_cmplx_mag_squared_q31(buf, buf, 1024);
	//draw_samples();
	//return;
	uint16_t gainshift = SPDISPINFO->p.overgain;
	int i = SPDISPINFO->p.offset;
	int stride = SPDISPINFO->p.stride;
	uint16_t (*block)[32] = (uint16_t (*)[32])spi_buffer;
	int sx, x, y;
	for (sx = 0; sx < 320; sx += 32) {
		for (x = 0; x < 32; x++) {
			q31_t ii = buf[(i&1023)*2];
			q31_t qq = buf[(i&1023)*2+1];
			q31_t mag = ((int64_t)ii*ii + (int64_t)qq*qq)>>(33-gainshift);
			//q31_t mag = buf[i & 1023];
			int v = log2_q31(mag) >> 6;
			if (v > 64) v = 64;
			for (y = 0; y < v; y++)
				block[63-y][x] = 0xffff;
			for ( ; y < 64; y++)
				block[63-y][x] = 0;
			i += stride;
		}
		ili9341_draw_bitmap(sx, 72, 32, 64, (uint16_t*)block);
	}
}

#define RGB565(r,g,b)     ( (((r)<<8)&0xf800) | (((g)<<3)&0x07e0) | ((b)&0x001f) )

const struct { uint8_t r,g,b; } colormap[] = {
		{ 0, 0, 0 },
		{ 0, 0, 255 },
		{ 0, 255, 0 },
		{ 255, 0, 0 },
		{ 255, 255, 255 }
};

uint16_t
pick_color(int mag) /* mag: 0 - 63 */
{
	int idx = (mag >> 4) & 0x3;
	int prop = mag & 0x0f;
	int nprop = 0x10 - prop;
	int r = colormap[idx].r * nprop + colormap[idx+1].r * prop;
	int g = colormap[idx].g * nprop + colormap[idx+1].g * prop;
	int b = colormap[idx].b * nprop + colormap[idx+1].b * prop;
	return RGB565(r>>4, g>>4, b>>4);
}

void
waterfall_init(void)
{
	// Vertical Scroll Definition
	uint16_t tfa = 152;
	uint16_t vsa = 240 - tfa;
	uint16_t bfa = 80;
	uint8_t vsd[6] = { tfa>>8, tfa, vsa>>8, vsa, bfa>>8, bfa };
	send_command(0x33, 6, vsd);
}

int vsa = 152;

void
draw_waterfall(void)
{
	int x;
	q31_t *buf = SPDISPINFO->buffer;
	uint16_t *block = spi_buffer;
	int i = SPDISPINFO->p.offset;
	int stride = SPDISPINFO->p.stride;
	uint16_t gainshift = SPDISPINFO->p.overgain;

	for (x = 0; x < 320; x++) {
		q31_t ii = buf[(i&1023)*2];
		q31_t qq = buf[(i&1023)*2+1];
		q31_t mag = ((int64_t)ii*ii + (int64_t)qq*qq)>>(33-gainshift);
		//q31_t mag = buf[i & 1023];
		int v = log2_q31(mag) >> 6;
		if (v > 63) v = 63;
		*block++ = pick_color(v);
		i += stride;
	}

	vsa++;
	if (vsa >= 240)
		vsa = 152;

	// Vertical Scroll Address
	uint8_t vscrsadd[2] = { vsa>>8, vsa };
	send_command(0x37, 2, vscrsadd);

	ili9341_draw_bitmap(0, vsa, 320, 1, spi_buffer);
}

void
draw_tick(void)
{
	char str[10];
	int x = SPDISPINFO->p.origin;
	int base = SPDISPINFO->p.tickbase;
	int xx;
	uint16_t bg = UISTAT->mode == SPDISP ? BG_ACTIVE : BG_NORMAL;

	ili9341_fill(0, 136, 320, 16, bg);
	sprintf(str, "%d%s", base, SPDISPINFO->p.unitname);
	xx = x - strlen(str) * 5 / 2;
	if (xx < 0) xx = 0;
	ili9341_drawstring_dma(str, xx, 142, 0xffff, bg);
	ili9341_fill(x, 136, 2, 5, 0xffff);

	base += SPDISPINFO->p.tickunit;
	x += SPDISPINFO->p.tickstep;
	while (x < 320) {
		sprintf(str, "%d", base);
		ili9341_fill(x, 136, 2, 5, 0xffff);
		ili9341_drawstring_dma(str, x, 142, 0xffff, bg);
		base += SPDISPINFO->p.tickunit;
		x += SPDISPINFO->p.tickstep;
	}
	x = SPDISPINFO->p.origin;
	base = SPDISPINFO->p.tickbase;
	base -= SPDISPINFO->p.tickunit;
	x -= SPDISPINFO->p.tickstep;
	while (x >= 0) {
		sprintf(str, "%d", base);
		ili9341_fill(x, 136, 2, 5, 0xffff);
		ili9341_drawstring_dma(str, x, 142, 0xffff, bg);
		base -= SPDISPINFO->p.tickunit;
		x -= SPDISPINFO->p.tickstep;
	}
}

void
draw_freq(void)
{
	char str[10];
	uint16_t bg = UISTAT->mode == FREQ ? BG_ACTIVE : BG_NORMAL;
	int i;
	const uint16_t xsim[] = { 0, 16, 0, 0, 16, 0, 0, 0 };
	uint16_t x = 0;
	sprintf(str, "%8d", UISTAT->freq);
	for (i = 0; i < 8; i++) {
		int8_t c = str[i] - '0';
		uint16_t fg = 0xffff;
		if (UISTAT->mode == FREQ && UISTAT->digit == 7-i)
			fg = 0xfe40;

		if (c >= 0 && c <= 9)
			ili9341_drawfont_dma(c, &NF32x48, x, 0, fg, bg);
		else
			ili9341_fill(x, 0, 32, 48, bg);
		x += 32;

		// fill gaps
		if (xsim[i] > 0) {
			ili9341_fill(x, 0, xsim[i], 48, bg);
			x += xsim[i];
		}
	}
	// draw Hz symbol
	ili9341_drawfont_dma(10, &NF32x48, x, 0, 0xffff, bg);
}

void
draw_info(void)
{
	char str[10];
	int x = 0;
	int y = 48;
	uint16_t bg = UISTAT->mode == GAIN ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont_dma(14, &NF20x24, x, y, 0xfffe, bg);
	x += 20;
	if (UISTAT->gain != -7)
		sprintf(str, "%2d", UISTAT->gain);
	else
		// -infinity
		sprintf(str, "-\003", UISTAT->gain);
	ili9341_drawfont_string(str, &NF20x24, x, y, 0xfffe, bg);
	x += 40;
	ili9341_drawfont_dma(13, &NF20x24, x, y, 0xfffe, bg);
	x += 20;

	bg = UISTAT->mode == MOD ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont_dma(UISTAT->modulation, &ICON48x20, x+2, y+2, 0xffe0, bg);
	x += 48+4;

	bg = UISTAT->mode == AGCMODE ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont_dma(UISTAT->agcmode + 2, &ICON48x20, x+2, y+2, 0xffff, bg);
	x += 48+4;

	bg = UISTAT->mode == RFGAIN ? BG_ACTIVE : BG_NORMAL;
	ili9341_drawfont_dma(15, &NF20x24, x, y, 0x07ff, bg);
	x += 20;
	sprintf(str, "%3d ", -6 * (int32_t)UISTAT->rfgain);
	ili9341_drawfont_string(str, &NF20x24, x, y, 0x07ff, bg);
	x += 60;
	ili9341_drawfont_dma(13, &NF20x24, x, y, 0x07ff, bg);
	x += 20;
}

void
clear_background(void)
{
	int i = 0;
	for (i = 0; i < 12; i++) {
		ili9341_fill(0, i*10, 320, 10, 0x0000);
	}
}


//volatile int count;

// event handler sent from M4 core
void M0_M4CORE_IRQHandler(void) {
	LPC_CREG->M4TXEVENT = 0;
	//ili9341_drawfont_dma(count++ % 10, &NF32x48, 192, 0, 0xffC0, 0x0000);

	if (SPDISPINFO->update_flag & FLAG_SPDISP) {
		draw_spectrogram();
		draw_waterfall();
		SPDISPINFO->update_flag &= ~FLAG_SPDISP;
	}
	if (SPDISPINFO->update_flag & FLAG_UI) {
		draw_tick();
		draw_freq();
		draw_info();
		SPDISPINFO->update_flag &= ~FLAG_UI;
	}
}

//volatile int8_t pending_launch = 0;

int main(void) {
	//char buf[16];
	//int i;

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
	ili9341_init();
	waterfall_init();
	clear_background();

	//spi_test();
#if 0
	ili9341_drawfont_dma(0, &ICON48x20, 0, 48, 0xffe0, 0x0000);		// LSB
	ili9341_drawfont_dma(1, &ICON48x20, 48, 48, 0x07ff, 0x0000);	// USB
	ili9341_drawfont_dma(2, &ICON48x20, 96, 48, 0xffff, 0x0000);	// OFF
	ili9341_drawfont_dma(3, &ICON48x20, 48*3, 48, 0xffff, 0x0000);	// FAST
	ili9341_drawfont_dma(4, &ICON48x20, 48*4, 48, 0xffff, 0x0000);	// SLOW
	ili9341_drawfont_dma(5, &ICON48x20, 48*5, 48, 0xffff, 0x0000);	// MID
#endif
#if 0
	//ili9341_test();
	//ili9341_dma_test();
	//ili9341_bulk_test();
	//ili9341_drawstring_dma("Hello", 100, 100, 0xffff, 0x0000);
	for (i = 0; i < 10; i++)
		ili9341_drawfont_dma(i, &NF20x24, i*20, 48, 0xffe0, 0x0000);
	for (i = 0; i < 5; i++)
		ili9341_drawfont_dma(i, &NF32x48, i*32, 0, 0xffff, 0x0000);
	//for (i = 0; i < 5; i++)
	//	ili9341_drawfont_dma(i+5, &NF32x48, i*32, 92, 0xffff, 0x0000);
#endif

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
