/**************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo for EFM32WG_STK3800
 * @version 4.4.0
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_system.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "em_gpio.h"
#include "em_msc.h"
#include "em_lcd.h"
#include "segmentlcd.h"
//#include "em_timer.h"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_timer.c"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_msc.c"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/inc/em_msc.h"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_int.c"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emdrv/gpiointerrupt/src/gpiointerrupt.c"
#include "em_int.h"

//#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/an/an0019_efm32_eeprom_emulation/eeprom_emulation.h"

//#define DUTY_CYCLE                  45
#define FPWM	                    40000
#define TIMER_TOP                   (uint32_t)((14000000/FPWM)-1)
//#define TIMBUF		                (uint32_t)((TIMER_TOP*DUTY_CYCLE)/100)
#define TIMER_CHANNEL				2
#define TIMER_NUMBER				TIMER1
#define PWMLOCATION 				TIMER_ROUTE_LOCATION_LOC3
#define CCchanel					TIMER_ROUTE_CC2PEN
volatile uint32_t msTicks; /* counts 1ms timeTicks */

#define EM_MSC_RUN_FROM_FLASH

void Delay(uint32_t dlyTicks);
void moveInterruptVectorToRam(void);

uint8_t 	counter		= 0;
uint32_t 	DUTY_CYCLE	= 10;
uint32_t 	TIMBUF;

volatile uint8_t PB_0;
volatile uint8_t PB_1;


uint32_t Address;
uint32_t *Data_Ptr;
uint8_t str;

#define PageSize	((uint32_t)0x00000800)
#define PageAddress ((uint32_t)0x00004000)
#define IDaddress 	((uint32_t)0x00004000)
#define IDlenght  	((uint32_t)0x00000008)
#define KEYaddress	((uint32_t)IDaddress+IDlenght)
#define KEYlenght  	((uint32_t)0x00000008)
#define reserved_areaAddress ((uint32_t)(PageAddress+IDlenght+KEYlenght))
#define reserved_areaLenght (PageAddress+PageSize-IDlenght-IDlenght)
const uint8_t __attribute__((at(IDaddress))) ID[]= {0x63,0x69,0x65,0x47,0x32,0x36,0x36,0x38};
const uint8_t __attribute__((at(IDaddress))) KEY[]={0x20,0x42,0xA5,0x17,0x63,0x42,0x25,0x98};
const uint8_t __attribute__((at(reserved_areaAddress))) reserved_area[reserved_areaLenght];

uint8_t NewID []={0x31,0x31,0x33,0x33,0x65,0x75,0x85,0x95};
uint8_t Newkey[]={0x61,0xEF,0xFD,0xCD,0xA4,0xA2,0xB3,0x01};
/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

void gpioCallback(uint8_t pin)
{
  if (pin == 9)
  {
    PB_0=1;
  }
  else if (pin == 10)
  {
    PB_1=1;
  }
}
//void TIMER1_IRQHandler(void)
//{
//  TIMER_IntClear(TIMER_NUMBER, TIMER_IF_CC2);      // Clear overflow flag
//  counter++;                             // Increment counter
//}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

//  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_HFPER) / 1000)) while (1) ;

  /* Initialize gpio pin */
  CMU_ClockEnable(cmuClock_GPIO, true);
//  GPIO_PinModeSet(gpioPortB, 11 , gpioModePushPull, 0);

  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortB,9,gpioModeInputPull,1);
  GPIO_PinModeSet(gpioPortB,10,gpioModeInputPull,1);

  GPIOINT_Init();

  /* Register callbacks before setting up and enabling pin interrupt. */
  GPIOINT_CallbackRegister(9, gpioCallback);
  GPIOINT_CallbackRegister(10, gpioCallback);

  /* Set falling edge interrupt for both ports */
  GPIO_IntConfig(gpioPortB, 9, true, false, true);
  GPIO_IntConfig(gpioPortB, 10, true, false, true);

  str=ID[0];
  str=KEY[0];
  str=reserved_area[0];


//  CMU_ClockEnable(cmuClock_TIMER1, true);
//
//  EFM_ASSERT(CMU->HFPERCLKDIV & _CMU_HFPERCLKEN0_MASK);
//  // Create the timer count control object initializer
//  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
//  timerCCInit.mode = timerCCModePWM;
//  timerCCInit.cmoa = timerOutputActionToggle;
//
//  // Configure CC channel 0
//  TIMER_InitCC(TIMER_NUMBER, TIMER_CHANNEL, &timerCCInit);
//
//  // Route CCx to location x  and enable pin for ccx
//  TIMER_NUMBER->ROUTE |= (CCchanel | PWMLOCATION);
//
//  // Set Top Value
//  TIMER_TopSet(TIMER_NUMBER, TIMER_TOP);
//
//  // Set the PWM duty cycle here
//  TIMER_CompareBufSet(TIMER_NUMBER, TIMER_CHANNEL, TIMBUF);
//
//  // Create a timerInit object, based on the API default
//  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
//  timerInit.prescale = timerPrescale1;
//  timerInit.debugRun = true;
//
//  TIMER_IntEnable(TIMER_NUMBER, TIMER_IF_CC2);
//  NVIC_EnableIRQ(TIMER1_IRQn);
//
//  TIMER_Init(TIMER_NUMBER, &timerInit);
//  TIMER_Enable(TIMER_NUMBER, true);

  while (1)
  {
//	    if(counter == 5)
//	    {
//	    	if(DUTY_CYCLE<100)
//	    	{
//		    	DUTY_CYCLE+=10;
//	    	}
//	    	TIMBUF=TIMER_TOP*DUTY_CYCLE;
//	    	TIMBUF=TIMBUF/100;
//
//	    	TIMER_CompareBufSet(TIMER_NUMBER, TIMER_CHANNEL, TIMBUF);
//	    	counter = 0;
//	    }
	    if( PB_0  )
		{
	    	/* Enables the flash controller for writing. */
	    	MSC_Init();

	    	/* Erase the FLASH pages */
	    	MSC_ErasePage(PageAddress);

	    	/* Program Flash Bank1 */
	    	Address  = IDaddress;
	    	Data_Ptr = (uint32_t *)NewID;
	    	MSC_Init();

	    	while(Address < (IDaddress+IDlenght))
	    	  {
	    	    MSC_WriteWord(Address, (void *)Data_Ptr,4);
	    	    Address = Address +4;
	    	    Data_Ptr = Data_Ptr ++;
	    	  }
	    	Address  = KEYaddress;
	    	Data_Ptr = (uint32_t *)Newkey;

	    	while(Address < (KEYaddress+KEYlenght))
	    	  {
	    	    MSC_WriteWord(Address, (void *)Data_Ptr,4);
	    	    Address = Address +4;
	    	    Data_Ptr = Data_Ptr ++;
	    	  }
	    		MSC_Deinit();
	    		PB_0=0;
	    }
  }
}

