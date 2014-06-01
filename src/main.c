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

#include <lpc43xx.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_gpdma.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_rgu.h>

#include <cr_section_macros.h>

#include <stdio.h>
#include <limits.h>
#include <arm_math.h>

#include "hsadctest.h"
#include "meas.h"


// TODO: insert other include files here

#define VADC_DMA_WRITE  7
#define VADC_DMA_READ   8
#define VADC_DMA_READ_SRC  (LPC_VADC_BASE + 512)  /* VADC FIFO */
#define FIFO_SIZE       8

#define DMA_NUM_LLI_TO_USE    4
static GPDMA_LLI_Type DMA_Stuff[DMA_NUM_LLI_TO_USE];

// ADC sampling rate: 20MHz
#define PLL0_MSEL	400
#define PLL0_NSEL	15
#define PLL0_PSEL	8
#define ADCCLK_MATCHVALUE	(8 - 1)  // 80MHz / 4 = 20MHz
#define ADCCLK_DGECI 0

#define SETTINGS_GPIO_IN    (PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW | INBUF_ENABLE  | FILTER_ENABLE)
#define SETTINGS_GPIO_OUT   (PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW |                 FILTER_ENABLE)
#define SETTINGS_SGPIO      (PDN_DISABLE | PUP_DISABLE |                 INBUF_ENABLE                 )
#define SETTINGS_SPIFI      (PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW | INBUF_ENABLE  | FILTER_ENABLE)
#define SETTINGS_SSP        (PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW | INBUF_ENABLE  | FILTER_ENABLE)

// ------------------------------------------------------------------------------------------------
// -----                                         VADC                                         -----
// ------------------------------------------------------------------------------------------------
/**
  * @brief Product name title=UM????? Chapter title=?????? Modification date=12/11/2012 Major revision=? Minor revision=?  (VADC)
    0x400F0000
  */
typedef struct {                            /*!< (@ 0x400F0000) VADC Structure         */
  __O  uint32_t FLUSH;                      /*!< (@ 0x400F0000) Flushes FIFO */
  __IO uint32_t DMA_REQ;                    /*!< (@ 0x400F0004) Set or clear DMA write request */
  __I  uint32_t FIFO_STS;                   /*!< (@ 0x400F0008) Indicates FIFO fullness status */
  __IO uint32_t FIFO_CFG;                   /*!< (@ 0x400F000C) Configures FIFO fullness level that triggers interrupt and packing 1 or 2 samples per word. */
  __O  uint32_t TRIGGER;                    /*!< (@ 0x400F0010) Enable software trigger to start descriptor processing */
  __IO uint32_t DSCR_STS;                   /*!< (@ 0x400F0014) Indicates active descriptor table and descriptor entry */
  __IO uint32_t POWER_DOWN;                 /*!< (@ 0x400F0018) Set or clear power down mode */
  __IO uint32_t CONFIG;                     /*!< (@ 0x400F001C) Configures external trigger mode, store channel ID in FIFO and wakeup recovery time from power down. */
  __IO uint32_t THR_A;                      /*!< (@ 0x400F0020) Configures window comparator A levels. */
  __IO uint32_t THR_B;                      /*!< (@ 0x400F0024) Configures window comparator B levels. */
  __I  uint32_t LAST_SAMPLE[6];             /*!< (@ 0x400F0028)	Contains last converted sample of input M [M=0..5) and result of window comparator. */
  __I  uint32_t RESERVED0[48];
  __IO uint32_t ADC_DEBUG;                  /*!< (@ 0x400F0100) Reserved  (ADC Debug pin inputs) */
  __IO uint32_t ADC_SPEED;                  /*!< (@ 0x400F0104) ADC speed control */
  __IO uint32_t POWER_CONTROL;              /*!< (@ 0x400F0108) Configures ADC power vs. speed, DC-in biasing, output format and power gating. */
  __I  uint32_t RESERVED1[61];
  __I  uint32_t FIFO_OUTPUT[16];            /*!< (@ 0x400F0200 - 0x400F023C) FIFO output mapped to 16 consecutive address locations. An output contains the value and input channel ID of one or two converted samples  */
  __I  uint32_t RESERVED2[48];
  __IO uint32_t DESCRIPTOR_0[8];            /*!< (@ 0x400F0300) Table0  descriptor n, n= 0 to 7  */
  __IO uint32_t DESCRIPTOR_1[8];            /*!< (@ 0x400F0320) Table1  descriptor n, n= 0 to 7  */
  __I  uint32_t RESERVED3[752];
  __O  uint32_t CLR_EN0;                    /*!< (@ 0x400F0F00) Interrupt0 clear mask */
  __O  uint32_t SET_EN0;                    /*!< (@ 0x400F0F04) Interrupt0 set mask */
  __I  uint32_t MASK0;                      /*!< (@ 0x400F0F08) Interrupt0 mask */
  __I  uint32_t STATUS0;                    /*!< (@ 0x400F0F0C) Interrupt0 status. Interrupt0 contains FIFO fullness, descriptor status and ADC range under/overflow */
  __O  uint32_t CLR_STAT0;                  /*!< (@ 0x400F0F10) Interrupt0 clear status  */
  __O  uint32_t SET_STAT0;                  /*!< (@ 0x400F0F14) Interrupt0 set status  */
  __I  uint32_t RESERVED4[2];
  __O  uint32_t CLR_EN1;                    /*!< (@ 0x400F0F20) Interrupt1 mask clear enable.  */
  __O  uint32_t SET_EN1;                    /*!< (@ 0x400F0F24) Interrupt1 mask set enable  */
  __I  uint32_t MASK1;                      /*!< (@ 0x400F0F28) Interrupt1 mask */
  __I  uint32_t STATUS1;                    /*!< (@ 0x400F0F2C) Interrupt1 status. Interrupt1 contains window comparator results and register last LAST_SAMPLE[M] overrun. */
  __O  uint32_t CLR_STAT1;                  /*!< (@ 0x400F0F30) Interrupt1 clear status  */
  __O  uint32_t SET_STAT1;                  /*!< (@ 0x400F0F34) Interrupt1 set status  */
} LPC_VADC_Type;

#define LPC_VADC_BASE             0x400F0000
#define LPC_VADC                  ((LPC_VADC_Type           *) LPC_VADC_BASE)

#define STATUS0_FIFO_FULL_MASK      (1<<0)
#define STATUS0_FIFO_EMPTY_MASK     (1<<1)
#define STATUS0_FIFO_OVERFLOW_MASK  (1<<2)
#define STATUS0_DESCR_DONE_MASK     (1<<3)
#define STATUS0_DESCR_ERROR_MASK    (1<<4)
#define STATUS0_ADC_OVF_MASK        (1<<5)
#define STATUS0_ADC_UNF_MASK        (1<<6)

#define STATUS0_CLEAR_MASK          0x7f

#define STATUS1_THCMP_BRANGE(__ch)  ((1<<0) << (5 * (__ch)))
#define STATUS1_THCMP_ARANGE(__ch)  ((1<<1) << (5 * (__ch)))
#define STATUS1_THCMP_DCROSS(__ch)  ((1<<2) << (5 * (__ch)))
#define STATUS1_THCMP_UCROSS(__ch)  ((1<<3) << (5 * (__ch)))
#define STATUS1_THCMP_OVERRUN(__ch) ((1<<4) << (5 * (__ch)))

#define STATUS1_CLEAR_MASK          0x1fffffff

#define CGU_BASE_VADC CGU_BASE_ENET_CSR
#define VADC_IRQn RESERVED7_IRQn

int32_t capture_count;

#define CAPTUREBUFFER		((uint8_t*)0x20000000)
#define CAPTUREBUFFER_SIZE	0x8000

#define DEST_BUFFER			((uint8_t*)0x20008000)
#define DEST_BUFFER_SIZE		0x4000

#define NCO_BUFFER			((uint8_t*)0x2000C000)
#define NCO_BUFFER_SIZE		0x800
#define NCO_SAMPLES			1024

/*
 * DSP Processing
 */

#define NCO_CYCLE 1000
#define NCO_SAMPLES 1024
#define NCO_COS_OFFSET (NCO_CYCLE/4)

static void ConfigureNCOTable(int freq)
{
	int i;
	int16_t *tbl = (int16_t*)NCO_BUFFER;
	for (i = 0; i < NCO_SAMPLES; i++) {
		tbl[i] = (int16_t)(arm_cos_f32(2*PI*freq*(i+0.5)/NCO_CYCLE) * SHRT_MAX / 16);
		//tbl[i] = 1;
	}
}


typedef struct {
	int32_t s0;
	int32_t s1;
	int32_t s2;
	int32_t d0;
	int32_t d1;
	int32_t d2;
	int32_t dest;
} CICState;

static void cic_decimate(CICState *cic, uint8_t *buf, int len)
{
	uint32_t offset = 0x08000800;
	uint32_t *capture = (uint32_t*)buf;
	uint32_t *nco_base = (uint32_t*)NCO_BUFFER;
	int16_t *result = (int16_t*)DEST_BUFFER;

	int32_t s0 = cic->s0;
	int32_t s1 = cic->s1;
	int32_t s2 = cic->s2;
	int32_t d0 = cic->d0;
	int32_t d1 = cic->d1;
	int32_t d2 = cic->d2;
	int32_t e0, e1, e2;
	uint32_t f;
	uint32_t x;
	int i, j, k, l;

	l = cic->dest;
	for (i = 0; i < len / 4; ) {
		for (j = 0; j < NCO_SAMPLES/2; ) {
			for (k = 0; k < 8; k++) {
				x = capture[i++];
				x = __SSUB16(x, offset);
				f = nco_base[j++];
				s0 = __SMLAD(x, f, s0);
				s1 += s0;
				s2 += s1;
				/*d00 = d0[k] - s2;
				d0[k] = s2;
				d11 = d1[k] - d00;
				d1[k] = d00;
				d22 = d2[k] - d11;
				d2[k] = d11;*/
			}
			e0 = d0 - s2;
			d0 = s2;
			e1 = d1 - e0;
			d1 = e0;
			e2 = d2 - e1;
			d2 = e1;
			result[l++] = e2 >> 16;
			l %=  DEST_BUFFER_SIZE/2;
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

typedef struct {
	int64_t s0;
	int64_t s1;
	int64_t s2;
	int64_t d0;
	int64_t d1;
	int64_t d2;
	int32_t dest;
} CICState64;

static void cic_decimate64(CICState64 *cic, uint8_t *buf, int len)
{
	uint32_t offset = 0x08000800;
	uint32_t *capture = (uint32_t*)buf;
	uint32_t *nco_base = (uint32_t*)NCO_BUFFER;
	int16_t *result = (int16_t*)DEST_BUFFER;

	int64_t s0 = cic->s0;
	int64_t s1 = cic->s1;
	int64_t s2 = cic->s2;
	int64_t d0 = cic->d0;
	int64_t d1 = cic->d1;
	int64_t d2 = cic->d2;
	int64_t e0, e1, e2;
	uint32_t f;
	uint32_t x;
	int i, j, k, l;

	l = cic->dest;
	for (i = 0; i < len / 4; ) {
		for (j = 0; j < NCO_SAMPLES/2; ) {
			for (k = 0; k < 8; k++) {
				x = capture[i++];
				x = __SSUB16(x, offset);
				f = nco_base[j++];
				s0 = __SMLALD(x, f, s0);
				s1 += s0;
				s2 += s1;
			}
			e0 = d0 - s2;
			d0 = s2;
			e1 = d1 - e0;
			d1 = e0;
			e2 = d2 - e1;
			d2 = e1;
			result[l++] = e2 >> 16;
			l %=  DEST_BUFFER_SIZE/2;
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

static CICState cic1;

void DMA_IRQHandler (void)
{
  if (LPC_GPDMA->INTERRSTAT & 1)
  {
    LPC_GPDMA->INTERRCLR = 1;
  }

#define MEMCPY __aeabi_memcpy8

  if (LPC_GPDMA->INTTCSTAT & 1)
  {
	LPC_GPDMA->INTTCCLEAR = 1;
	//LPC_GPDMA->C0CONFIG |= (1 << 18); //halt further requests

	//TOGGLE_MEAS_PIN_3();
    int length = CAPTUREBUFFER_SIZE / 2;
	SET_MEAS_PIN_3();
    if ((capture_count & 1) == 0)
    	//MEMCPY(DEST_BUFFER, CAPTUREBUFFER, length);
    	cic_decimate(&cic1, CAPTUREBUFFER, length);
    else
    	//MEMCPY(DEST_BUFFER + length, CAPTUREBUFFER + length, length);
    	//MEMCPY(DEST_BUFFER, CAPTUREBUFFER + length, length);
    	cic_decimate(&cic1, CAPTUREBUFFER + length, length);
    CLR_MEAS_PIN_3();
    capture_count ++;
  }
}

static void VADC_SetupDMA(void)
{
  int i;
  uint32_t transSize;

  NVIC_DisableIRQ(DMA_IRQn);
  LPC_GPDMA->C0CONFIG = 0;

  /* clear all interrupts on channel 0 */
  LPC_GPDMA->INTTCCLEAR = 0x01;
  LPC_GPDMA->INTERRCLR = 0x01;

  /* Setup the DMAMUX */
  LPC_CREG->DMAMUX &= ~(0x3<<(VADC_DMA_WRITE*2));
  LPC_CREG->DMAMUX |= 0x3<<(VADC_DMA_WRITE*2);  /* peripheral 7 vADC Write(0x3) */
  LPC_CREG->DMAMUX &= ~(0x3<<(VADC_DMA_READ*2));
  LPC_CREG->DMAMUX |= 0x3<<(VADC_DMA_READ*2);  /* peripheral 8 vADC read(0x3) */

  LPC_GPDMA->CONFIG = 0x01;  /* Enable DMA channels, little endian */
  while ( !(LPC_GPDMA->CONFIG & 0x01) );

  // The size of the transfer is in multiples of 32bit copies (hence the /4)
  // and must be even multiples of FIFO_SIZE.
  transSize = CAPTUREBUFFER_SIZE / DMA_NUM_LLI_TO_USE / 4;

  for (i = 0; i < DMA_NUM_LLI_TO_USE; i++)
  {
    DMA_Stuff[i].SrcAddr = VADC_DMA_READ_SRC;
    DMA_Stuff[i].DstAddr = ((uint32_t)CAPTUREBUFFER) + transSize*4*i;
    DMA_Stuff[i].NextLLI = (uint32_t)(&DMA_Stuff[(i+1)%DMA_NUM_LLI_TO_USE]);
    DMA_Stuff[i].Control = (transSize << 0) |      // Transfersize (does not matter when flow control is handled by peripheral)
                           (0x2 << 12)  |          // Source Burst Size
                           (0x2 << 15)  |          // Destination Burst Size
                           (0x2 << 18)  |          // Source width // 32 bit width
                           (0x2 << 21)  |          // Destination width   // 32 bits
                           (0x1 << 24)  |          // Source AHB master 0 / 1
                           (0x0 << 25)  |          // Dest AHB master 0 / 1
                           (0x0 << 26)  |          // Source increment(LAST Sample)
                           (0x1 << 27)  |          // Destination increment
                           (0x0UL << 31);          // Terminal count interrupt disabled
    //log_i("DMA_Stuff[%d] on address %#x, destination %#x, transfer size %#x (%d)\r\n", i, (uint32_t)&DMA_Stuff[i], DMA_Stuff[i].DstAddr, transSize, transSize);
  }

  // Let the last LLI in the chain cause a terminal count interrupt to
  // notify when the capture buffer is completely filled
  DMA_Stuff[DMA_NUM_LLI_TO_USE/2 - 1].Control |= (0x1UL << 31); // Terminal count interrupt enabled
  DMA_Stuff[DMA_NUM_LLI_TO_USE - 1].Control |= (0x1UL << 31); // Terminal count interrupt enabled

  LPC_GPDMA->C0SRCADDR = DMA_Stuff[0].SrcAddr;
  LPC_GPDMA->C0DESTADDR = DMA_Stuff[0].DstAddr;
  LPC_GPDMA->C0CONTROL = DMA_Stuff[0].Control;
  LPC_GPDMA->C0LLI     = (uint32_t)(&DMA_Stuff[1]); // must be pointing to the second LLI as the first is used when initializing
  LPC_GPDMA->C0CONFIG  =  (0x1)        |          // Enable bit
                          (VADC_DMA_READ << 1) |  // SRCPERIPHERAL - set to 8 - VADC
                          (0x0 << 6)   |          // Destination peripheral - memory - no setting
                          (0x2 << 11)  |          // Flow control - peripheral to memory - DMA control
                          (0x1 << 14)  |          // Int error mask
                          (0x1 << 15);            // ITC - term count error mask

  NVIC_EnableIRQ(DMA_IRQn);
}

#define RGU_SIG_VADC 60

static void VADC_Init(void)
{
  CGU_EntityConnect(CGU_CLKSRC_PLL0_AUDIO, CGU_BASE_VADC);
  CGU_EnableEntity(CGU_BASE_VADC, ENABLE);

  // Reset the VADC block
  RGU_SoftReset(RGU_SIG_VADC);
  while(RGU_GetSignalStatus(RGU_SIG_VADC));

  // Clear FIFO
  LPC_VADC->FLUSH = 1;

  // Disable the VADC interrupt
  NVIC_DisableIRQ(VADC_IRQn);
  LPC_VADC->CLR_EN0 = STATUS0_CLEAR_MASK;         // disable interrupt0
  LPC_VADC->CLR_STAT0 = STATUS0_CLEAR_MASK;       // clear interrupt status
  while(LPC_VADC->STATUS0 & 0x7d);  // wait for status to clear, have to exclude FIFO_EMPTY (bit 1)
  LPC_VADC->CLR_EN1 = STATUS1_CLEAR_MASK;          // disable interrupt1
  LPC_VADC->CLR_STAT1 = STATUS1_CLEAR_MASK;  // clear interrupt status
  while(LPC_VADC->STATUS1);         // wait for status to clear

  // Make sure the VADC is not powered down
  LPC_VADC->POWER_DOWN =
    (0<<0);        /* PD_CTRL:      0=disable power down, 1=enable power down */

  // Clear FIFO
  LPC_VADC->FLUSH = 1;

  // FIFO Settings
  LPC_VADC->FIFO_CFG =
    (1<<0) |         /* PACKED_READ:      0= 1 sample packed into 32 bit, 1= 2 samples packed into 32 bit */
    (FIFO_SIZE<<1);  /* FIFO_LEVEL:       When FIFO contains this or more samples raise FIFO_FULL irq and DMA_Read_Req, default is 8 */

  // Descriptors:
  if (ADCCLK_MATCHVALUE == 0)
  {
    // A matchValue of 0 requires special handling to prevent a automatic start.
    // For more information see the "Appendix A Errata" of the VADC manual.
    LPC_VADC->DSCR_STS =
      (1<<0) |       /* ACT_TABLE:        0=table 0 is active, 1=table 1 is active */
      (0<<1);        /* ACT_DESCRIPTOR:   ID of the descriptor that is active */

    LPC_VADC->DESCRIPTOR_1[0] =
      (0<<0) |       /* CHANNEL_NR:    0=convert input 0, 1=convert input 1, ..., 5=convert input 5 */
      (0<<3) |       /* HALT:          0=continue with next descriptor after this one, 1=halt after this and restart at a new trigger */
      (0<<4) |       /* INTERRUPT:     1=raise interrupt when ADC result is available */
      (0<<5) |       /* POWER_DOWN:    1=power down after this conversion */
      (2<<6) |       /* BRANCH:        0=continue with next descriptor (wraps around after top) */
                     /*                1=branch to the first descriptor in this table */
                     /*                2=swap tables and branch to the first descriptor of the new table */
                     /*                3=reserved (do not store sample). continue with next descriptor (wraps around the top) */
      (1<<8)  |      /* MATCH_VALUE:   Evaluate this desciptor when descriptor timer value is equal to match value */
      (0<<22) |      /* THRESHOLD_SEL: 0=no comparison, 1=THR_A, 2=THR_B */
      (1<<24) |      /* RESET_TIME:    1=reset descriptor timer */
      (1UL<<31);     /* UPDATE_TABLE:  1=update table with all 8 descriptors of this table */
  }
  else
  {
    LPC_VADC->DSCR_STS =
      (0<<0) |       /* ACT_TABLE:        0=table 0 is active, 1=table 1 is active */
      (0<<1);        /* ACT_DESCRIPTOR:   ID of the descriptor that is active */
  }

  LPC_VADC->CONFIG = /* configuration register */
    (1<<0) |        /* TRIGGER_MASK:     0=triggers off, 1=SW trigger, 2=EXT trigger, 3=both triggers */
    (0<<2) |        /* TRIGGER_MODE:     0=rising, 1=falling, 2=low, 3=high external trigger */
    (0<<4) |        /* TRIGGER_SYNC:     0=no sync, 1=sync external trigger input */
    (0<<5) |        /* CHANNEL_ID_EN:    0=don't add, 1=add channel id to FIFO output data */
    (0x90<<6);      /* RECOVERY_TIME:    ADC recovery time from power down, default is 0x90 */

  {
    LPC_VADC->DESCRIPTOR_0[0] =
      (0<<0) |       /* CHANNEL_NR:    0=convert input 0, 1=convert input 1, ..., 5=convert input 5 */
      (0<<3) |       /* HALT:          0=continue with next descriptor after this one, 1=halt after this and restart at a new trigger */
      (0<<4) |       /* INTERRUPT:     1=raise interrupt when ADC result is available */
      (0<<5) |       /* POWER_DOWN:    1=power down after this conversion */
      (1<<6) |       /* BRANCH:        0=continue with next descriptor (wraps around after top) */
                     /*                1=branch to the first descriptor in this table */
                     /*                2=swap tables and branch to the first descriptor of the new table */
                     /*                3=reserved (do not store sample). continue with next descriptor (wraps around the top) */
      (ADCCLK_MATCHVALUE<<8)  |    /* MATCH_VALUE:   Evaluate this desciptor when descriptor timer value is equal to match value */
      (0<<22) |      /* THRESHOLD_SEL: 0=no comparison, 1=THR_A, 2=THR_B */
      (1<<24) |      /* RESET_TIME:    1=reset descriptor timer */
      (1UL<<31);       /* UPDATE_TABLE:  1=update table with all 8 descriptors of this table */
  }

  LPC_VADC->ADC_SPEED =
    ADCCLK_DGECI;   /* DGECx:      For CRS=3 all should be 0xF, for CRS=4 all should be 0xE, */
                       /*             for all other cases it should be 0 */

  LPC_VADC->POWER_CONTROL =
    (0 /*crs*/ << 0) |    /* CRS:          current setting for power versus speed programming */
    (1 << 4) |      /* DCINNEG:      0=no dc bias, 1=dc bias on vin_neg slide */
    (0 << 10) |     /* DCINPOS:      0=no dc bias, 1=dc bias on vin_pos slide */
    (0 << 16) |     /* TWOS:         0=offset binary, 1=two's complement */
    (1 << 17) |     /* POWER_SWITCH: 0=ADC is power gated, 1=ADC is active */
    (1 << 18);      /* BGAP_SWITCH:  0=ADC bandgap reg is power gated, 1=ADC bandgap is active */

//  LPC_VADC->SET_EN0 = STATUS0_FIFO_FULL_MASK;// only care about FIFO_FULL

  // Enable interrupts
  //NVIC_EnableIRQ(VADC_IRQn);

  VADC_SetupDMA();
}

static void VADC_Start(void)
{
	capture_count = 0;
	LPC_VADC->TRIGGER = 1;
}

static void VADC_Stop(void)
{
  NVIC_DisableIRQ(DMA_IRQn);
  NVIC_DisableIRQ(VADC_IRQn);

  // disable DMA
  LPC_GPDMA->C0CONFIG |= (1 << 18); //halt further requests
/*
  // power down VADC
  LPC_VADC->POWER_CONTROL = 0;

  // Clear FIFO
  LPC_VADC->FLUSH = 1;

  // Reset the VADC block
  RGU_SoftReset(RGU_SIG_VADC);
  while(RGU_GetSignalStatus(RGU_SIG_VADC));
*/
}

static void priorityConfig()
{
  // High - Copying of samples
  NVIC_SetPriority(DMA_IRQn,   ((0x01<<3)|0x01));

  // Low - Communication
  NVIC_SetPriority(USB0_IRQn, ((0x03<<3)|0x01));
  NVIC_SetPriority(USB1_IRQn, ((0x03<<3)|0x01));
  NVIC_SetPriority(I2C0_IRQn, ((0x03<<3)|0x01));
}

int main(void) {
	setup_systemclock();
    setup_pll0audio(PLL0_MSEL, PLL0_NSEL, PLL0_PSEL);
    priorityConfig();

    printf("Hello World\n");
    GPIO_SetDir(0,1<<8, 1);
	GPIO_ClearValue(0,1<<8);

	scu_pinmux(0x6, 11, SETTINGS_GPIO_OUT, FUNC0); //GPIO3[7], available on J7-14
	LPC_GPIO_PORT->DIR[3] |= (1UL << 7);
	LPC_GPIO_PORT->SET[3] |= (1UL << 7);

	//ConfigureNCOTable(2500000 / 20000); // 2.5MHz
	//ConfigureNCOTable(2500000 / 5000); // 2.5MHz
	ConfigureNCOTable(0); // 0MHz
	memset(&cic1, 0, sizeof cic1);

	VADC_Init();
	VADC_Start();
    //volatile static int i = 0 ;

    while(1) {
        //i++ ;
        if ((capture_count / 1024) % 2) {
        	LPC_GPDMA->C0CONFIG |= (1 << 18); //halt further requests
            //int length = CAPTUREBUFFER_SIZE / 2;
        	//memset(DEST_BUFFER, 0, DEST_BUFFER_SIZE);
            //cic_decimate(&cic1, CAPTUREBUFFER, length);

        	GPIO_SetValue(0,1<<8);
        } else
        	GPIO_ClearValue(0,1<<8);
        //if ((i & 0x1fffff) == 0x100000)
        //	printf("%d\n", i);
    }
	VADC_Stop();
    return 0 ;
}
