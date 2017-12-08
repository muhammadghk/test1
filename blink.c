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
//#include "em_timer.h"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_timer.c"

//#define DUTY_CYCLE                  45
#define FPWM	                    40000
#define TIMER_TOP                   (uint32_t)((14000000/FPWM)-1)
//#define TIMBUF		                (uint32_t)((TIMER_TOP*DUTY_CYCLE)/100)
#define TIMER_CHANNEL				2
#define TIMER_NUMBER				TIMER1
#define PWMLOCATION 				TIMER_ROUTE_LOCATION_LOC3
#define CCchanel					TIMER_ROUTE_CC2PEN
volatile uint32_t msTicks; /* counts 1ms timeTicks */

void Delay(uint32_t dlyTicks);

uint8_t 	counter		= 0;
uint32_t 	DUTY_CYCLE	= 10;
uint32_t 	TIMBUF;

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

void TIMER1_IRQHandler(void)
{
  TIMER_IntClear(TIMER_NUMBER, TIMER_IF_CC2);      // Clear overflow flag
  counter++;                             // Increment counter
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_HFPER) / 1000)) while (1) ;

  /* Initialize gpio pin */
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortB, 11 , gpioModePushPull, 0);



  CMU_ClockEnable(cmuClock_TIMER1, true);

  // Create the timer count control object initializer
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  timerCCInit.mode = timerCCModePWM;
  timerCCInit.cmoa = timerOutputActionToggle;

  // Configure CC channel 0
  TIMER_InitCC(TIMER_NUMBER, TIMER_CHANNEL, &timerCCInit);

  // Route CCx to location x  and enable pin for ccx
  TIMER_NUMBER->ROUTE |= (CCchanel | PWMLOCATION);

  // Set Top Value
  TIMER_TopSet(TIMER_NUMBER, TIMER_TOP);

  // Set the PWM duty cycle here
  TIMER_CompareBufSet(TIMER_NUMBER, TIMER_CHANNEL, TIMBUF);

  // Create a timerInit object, based on the API default
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.prescale = timerPrescale1;
  timerInit.debugRun = true;

  TIMER_IntEnable(TIMER_NUMBER, TIMER_IF_CC2);
  NVIC_EnableIRQ(TIMER1_IRQn);

  TIMER_Init(TIMER_NUMBER, &timerInit);
  TIMER_Enable(TIMER_NUMBER, true);



  while (1)
  {
	    if(counter == 5)
	    {
	    	if(DUTY_CYCLE<100)
	    	{
		    	DUTY_CYCLE+=10;
	    	}
	    	TIMBUF=TIMER_TOP*DUTY_CYCLE;
	    	TIMBUF=TIMBUF/100;

	    	TIMER_CompareBufSet(TIMER_NUMBER, TIMER_CHANNEL, TIMBUF);
	    	counter = 0;
	    }
  }
}

