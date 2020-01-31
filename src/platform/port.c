/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port.h"
#include "deca_device_api.h"

/****************************************************************************//**
 *
 * 								APP global variables
 *
 *******************************************************************************/

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;

/****************************************************************************//**
 *
 * 					Port private variables and function prototypes
 *
 *******************************************************************************/

static volatile uint32_t signalResetDone;


static int SPI1_Configuration(void);
static int SPI2_Configuration(void);
static int GPIO_Configuration(void);
static int RCC_Configuration(void);
static int NVIC_Configuration(void);
static void SPI1_ConfigRate(uint16_t scalingfactor);

/****************************************************************************//**
 *
 * 								Time section
 *
 *******************************************************************************/
 
/* @fn 	  portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 * 		  CLOCKS_PER_SEC frequency.
 * 		  The resolution of time32_incr is usually 1/1000 sec.
 * */
 __INLINE unsigned long portGetTickCnt(void)
{
	return time32_incr;
}

 /* @fn    SysTick_Configuration
  * @brief SysTickTimer is configured to be clocked from HCLK (72MHz)
  * */
int SysTick_Configuration(void)
{
	if (SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC) !=0)
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);

	return 0;
}


/* @fn 	  usleep
 * @brief precise microseconds delay
 * */
#pragma GCC optimize ("O0")
int usleep(useconds_t usec)
{
	int i,j;

#pragma GCC ivdep
	for(i=0;i<usec;i++)
	{

#pragma GCC ivdep
		for(j=0;j<2;j++)
		{
			__NOP();
			__NOP();
		}
	}
	return 0;
}


/* @fn 	  Sleep
 * @brief Sleep delay in ms using SysTicktimer
 * */
void Sleep(__IO uint32_t Delay)
{
	uint32_t tickstart = 0;
	tickstart = portGetTickCnt();
	while((portGetTickCnt() - tickstart) < Delay)
	{
	}
}

/****************************************************************************//**
 *
 * 								END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 * 								Configuration section
 *
 *******************************************************************************/

/* @fn 	  peripherals_init
 * */
int peripherals_init (void)
{
	RCC_Configuration();
	GPIO_Configuration();

	SysTick_Configuration();
	NVIC_Configuration();
	
	return 0;
}


/* @fn 	  spi_peripheral_init
 * */
void spi_peripheral_init()
{
	SPI1_Configuration();	//initialise SPI1 peripheral for DW1000 control
	SPI2_Configuration();	//initialise SPI2 peripheral for LCD control

	port_LCD_RS_clear();
	port_LCD_RW_clear();
}


/* @fn 	  NVIC_Configuration
 * */
int NVIC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable GPIO used as DECA IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = DECAIRQ;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPD;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
	GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);

	/* Connect EXTI Line to GPIO Pin */
	GPIO_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
	EXTI_Init(&EXTI_InitStructure);

	/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DECAIRQ_EXTI_USEIRQ;

	NVIC_Init(&NVIC_InitStructure);

	return 0;
}


/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
	/* Check the parameters */
	assert_param(IS_GET_EXTI_LINE(EXTI_Line));

	return ((EXTI->IMR & EXTI_Line) == (uint32_t)RESET)?(RESET):(SET);
}

/* @fn 	  RCC_Configuration
 * */
int RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_ClocksTypeDef RCC_ClockFreq;

	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus != ERROR)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/****************************************************************/
		/* HSE= up to 25MHz (on EVB1000 is 12MHz),
		 * HCLK=72MHz, PCLK2=72MHz, PCLK1=36MHz 						*/
		/****************************************************************/
		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);
		/*  ADCCLK = PCLK2/4 */
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);

		/* Configure PLLs *********************************************************/
		/* PLL2 configuration: PLL2CLK = (HSE / 4) * 8 = 24 MHz */
		RCC_PREDIV2Config(RCC_PREDIV2_Div4);
		RCC_PLL2Config(RCC_PLL2Mul_8);

		/* Enable PLL2 */
		RCC_PLL2Cmd(ENABLE);

		/* Wait till PLL2 is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET){}

		/* PLL1 configuration: PLLCLK = (PLL2 / 3) * 9 = 72 MHz */
		RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div3);

		RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08){}
	}

	RCC_GetClocksFreq(&RCC_ClockFreq);

	/* Enable SPI1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable SPI2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Enable GPIOs clocks */
	RCC_APB2PeriphClockCmd(
						RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
						RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
						ENABLE);

	return 0;
}


/* @fn 	SPI1_ConfigRate
 * 		sets the SPI rate of DW1000 SPI
 * */
void SPI1_ConfigRate(uint16_t scalingfactor)
{
	SPI_InitTypeDef SPI_InitStructure;

	SPI_I2S_DeInit(SPIx);

	// SPIx Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = scalingfactor;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIx, &SPI_InitStructure);

	// Enable SPIx
	SPI_Cmd(SPIx, ENABLE);
}


/* @fn 	SPI1_Configuration
 * 		connects GPIO pins to SPIx
 * 		and configures SPIx
 *
 * */
int SPI1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// SPIx SCK and MOSI pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_SCK | SPIx_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	// SPIx MISO pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

	GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	// SPIx CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);

	// Disable SPIx SS Output
	SPI_SSOutputCmd(SPIx, DISABLE);

	// Set CS high
	GPIO_SetBits(SPIx_CS_GPIO, SPIx_CS);

	// Configure and Enable SPIx
	SPI1_ConfigRate(SPI_BaudRatePrescaler_32);

    return 0;
}


int SPI2_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	SPI_I2S_DeInit(SPIy);

	// SPIy Mode setup
//	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIy, &SPI_InitStructure);

	// SPIy SCK and MOSI pin setup
	GPIO_InitStructure.GPIO_Pin = SPIy_SCK | SPIy_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

	// SPIy MISO pin setup
	GPIO_InitStructure.GPIO_Pin = SPIy_MISO;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

	// SPIy CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPIy_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(SPIy_CS_GPIO, &GPIO_InitStructure);

	// Disable SPIy SS Output
	SPI_SSOutputCmd(SPIy, DISABLE);

	// Enable SPIy
	SPI_Cmd(SPIy, ENABLE);

	// Set CS high
	GPIO_SetBits(SPIy_CS_GPIO, SPIy_CS);

	// LCD_RS pin setup
	GPIO_InitStructure.GPIO_Pin = LCD_RS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

	// LCD_RW pin setup
	GPIO_InitStructure.GPIO_Pin = LCD_RW;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    return 0;
}

int GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure all unused GPIO port pins in Analog Input mode (floating input
	* trigger OFF), this will reduce the power consumption and increase the device
	* immunity against EMI/EMC */

	// Enable GPIOs clocks
	RCC_APB2PeriphClockCmd(
						RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
						RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
						ENABLE);

	// Set all GPIO pins as analog inputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//Enable GPIO used for User button
	GPIO_InitStructure.GPIO_Pin = TA_BOOT1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(TA_BOOT1_GPIO, &GPIO_InitStructure);

	//Enable GPIO used for Response Delay setting
	GPIO_InitStructure.GPIO_Pin = TA_RESP_DLY | TA_SW1_3 | TA_SW1_4 | TA_SW1_5 | TA_SW1_6 | TA_SW1_7 | TA_SW1_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(TA_RESP_DLY_GPIO, &GPIO_InitStructure);

	//Enable GPIO used for SW1 switch setting
	GPIO_InitStructure.GPIO_Pin = TA_SW1_3 | TA_SW1_4 | TA_SW1_5 | TA_SW1_6 | TA_SW1_7 | TA_SW1_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(TA_SW1_GPIO, &GPIO_InitStructure);

	// Enable GPIO used for LEDs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_Remap_SPI1, DISABLE);

    return 0;
}

/****************************************************************************//**
 *
 * 							End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 * 							DW1000 port section
 *
 *******************************************************************************/

/* @fn		reset_DW1000
 * @brief	DW_RESET pin on DW1000 has 2 functions
 * 			In general it is output, but it also can be used to reset the digital
 * 			part of DW1000 by driving this pin low.
 * 			Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

	usleep(1);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	Sleep(2);
}

/* @fn		setup_DW1000RSTnIRQ
 * @brief	setup the DW_RESET pin mode
 * 			0 - output Open collector mode
 * 			!0 - input mode with connected EXTI0 IRQ
 * */
void setup_DW1000RSTnIRQ(int enable)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	if(enable)
	{
		// Enable GPIO used as DECA IRQ for interrupt
		GPIO_InitStructure.GPIO_Pin = DECARSTIRQ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(DECARSTIRQ_GPIO, &GPIO_InitStructure);

		/* Connect EXTI Line to GPIO Pin */
		GPIO_EXTILineConfig(DECARSTIRQ_EXTI_PORT, DECARSTIRQ_EXTI_PIN);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MP IRQ polarity is high by default
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

		/* Enable and set EXTI Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = DECARSTIRQ_EXTI_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		NVIC_Init(&NVIC_InitStructure);
	}
	else
	{
		//put the pin back to tri-state ... as analog input
		GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MP IRQ polarity is high by default
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;
		EXTI_Init(&EXTI_InitStructure);
	}
}


/* @fn		port_is_boot1_low
 * @brief	check the BOOT1 pin status.
 * @return  1 if ON and 0 for OFF
 * */
int port_is_boot1_low(void)
{
	return ((GPIO_ReadInputDataBit(TA_BOOT1_GPIO, TA_BOOT1))?(0):(1));
}

/* @fn		port_is_boot1_low
 * @brief	check the BOOT1 pin status.
 * @return  1 if ON and 0 for OFF
 * */
int port_is_boot1_on(uint16_t x)
{
	return ((GPIO_ReadInputDataBit(TA_BOOT1_GPIO, TA_BOOT1))?(0):(1));
}

/* @fn		port_is_switch_on
 * @brief	check the switch status.
 * 			when switch (S1) is 'on' the pin is low
 * @return  1 if ON and 0 for OFF
 * */
int port_is_switch_on(uint16_t GPIOpin)
{
	return ((GPIO_ReadInputDataBit(TA_SW1_GPIO, GPIOpin))?(0):(1));
}


/* @fn		led_off
 * @brief	switch off the led from led_t enumeration
 * */
void led_off (led_t led)
{
	switch (led)
	{
	case LED_PC6:
		GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		break;
	case LED_PC7:
		GPIO_ResetBits(GPIOC, GPIO_Pin_7);
		break;
	case LED_PC8:
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		break;
	case LED_PC9:
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		break;
	case LED_ALL:
		GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}

/* @fn		led_on
 * @brief	switch on the led from led_t enumeration
 * */
void led_on (led_t led)
{
	switch (led)
	{
	case LED_PC6:
		GPIO_SetBits(GPIOC, GPIO_Pin_6);
		break;
	case LED_PC7:
		GPIO_SetBits(GPIOC, GPIO_Pin_7);
		break;
	case LED_PC8:
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		break;
	case LED_PC9:
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		break;
	case LED_ALL:
		GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}


/* @fn		port_wakeup_dw1000
 * @brief	"slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
    port_SPIx_clear_chip_select();
    Sleep(1);
    port_SPIx_set_chip_select();
    Sleep(7);						//wait 7ms for DW1000 XTAL to stabilise
}

/* @fn		port_wakeup_dw1000_fast
 * @brief	waking up of DW1000 using DW_CS and DW_RESET pins.
 * 			The DW_RESET signalling that the DW1000 is in the INIT state.
 * 			the total fast wakeup takes ~2.2ms and depends on crystal startup time
 * */
void port_wakeup_dw1000_fast(void)
{
	#define WAKEUP_TMR_MS	(10)

	uint32_t x = 0;
	uint32_t timestamp = portGetTickCnt();	//protection

	setup_DW1000RSTnIRQ(0); 		//disable RSTn IRQ
	signalResetDone = 0;			//signalResetDone connected to the process_dwRSTn_irq() callback
	setup_DW1000RSTnIRQ(1); 		//enable RSTn Rising IRQ
	port_SPIx_clear_chip_select();  //CS low

	//need to poll to check when the DW1000 is in the IDLE, the CPLL interrupt is not reliable
	//when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL lock (in 5 us)
	while((signalResetDone == 0) && \
		  ((portGetTickCnt() - timestamp) < WAKEUP_TMR_MS))
	{
		x++;	 //when DW1000 will switch to an IDLE state RSTn pin will high
	}
	setup_DW1000RSTnIRQ(0); 		//disable RSTn IRQ
	port_SPIx_set_chip_select();  	//CS high

	//it takes ~35us in total for the DW1000 to lock the PLL, download AON and go to IDLE state
	usleep(35);
}



/* @fn		port_set_dw1000_slowrate
 * @brief	wrapper to set SPI_CLK for DW1000 to 2.25MHz
 * 			note: SPI1 is clocked from 72MHz
 * */
void port_set_dw1000_slowrate(void)
{
    SPI1_ConfigRate(SPI_BaudRatePrescaler_32);
}

/* @fn		port_set_dw1000_fastrate
 * @brief	wrapper to set SPI_CLK for DW1000 to 18MHz
 * 			note: SPI1 is clocked from 72MHz
 * */
void port_set_dw1000_fastrate(void)
{
    SPI1_ConfigRate(SPI_BaudRatePrescaler_4);
}

/* @fn		port_LCD_RS_set
 * @brief	wrapper to set LCD_RS pin
 * */
void port_LCD_RS_set(void)
{
	GPIO_SetBits(SPIy_GPIO,LCD_RS);
}

/* @fn		port_LCD_RS_clear
 * @brief	wrapper to clear LCD_RS pin
 * */
void port_LCD_RS_clear(void)
{
	GPIO_ResetBits(SPIy_GPIO,LCD_RS);
}

/* @fn		port_LCD_RW_clear
 * @brief	wrapper to set LCD_RW pin
 * */
void port_LCD_RW_set(void)
{
	GPIO_SetBits(SPIy_GPIO,LCD_RW);
}

/* @fn		port_LCD_RW_clear
 * @brief	wrapper to clear LCD_RW pin
 * */
void port_LCD_RW_clear(void)
{
	GPIO_ResetBits(SPIy_GPIO,LCD_RW);
}


/****************************************************************************//**
 *
 * 							End APP port section
 *
 *******************************************************************************/



/****************************************************************************//**
 *
 * 								IRQ section
 *
 *******************************************************************************/

/* @fn		process_dwRSTn_irq
 * @brief	call-back to signal to the APP that the DW_RESET pin is went high
 *
 * */
__INLINE void process_dwRSTn_irq(void)
{
	signalResetDone = 1;
}

/* @fn		process_deca_irq
 * @brief	main call-back for processing of DW1000 IRQ
 * 			it re-enters the IRQ routing and processes all events.
 * 			After processing of all events, DW1000 will clear the IRQ line.
 * */
__INLINE void process_deca_irq(void)
{
	while(port_CheckEXT_IRQ() == 1)
	{

    	dwt_isr();

    } //while DW1000 IRQ line active
}


/* @fn		port_DisableEXT_IRQ
 * @brief	wrapper to disable DW_IRQ pin IRQ
 * 			in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
	NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn		port_EnableEXT_IRQ
 * @brief	wrapper to enable DW_IRQ pin IRQ
 * 			in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
	NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}


/* @fn		port_GetEXT_IRQStatus
 * @brief	wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
	return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}


/* @fn		port_CheckEXT_IRQ
 * @brief	wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
	return GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ);
}


/****************************************************************************//**
 *
 * 								END OF IRQ section
 *
 *******************************************************************************/

