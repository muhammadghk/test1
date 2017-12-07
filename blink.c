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

#define DUTY_CYCLE                  6		// for 30%
#define TIMER_TOP                   20
#define TIMER_CHANNEL				0
#define TIMER_NUMBER				TIMER0

volatile uint32_t msTicks; /* counts 1ms timeTicks */

void Delay(uint32_t dlyTicks);

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

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  CMU_ClockFreqGet(cmuClock_HFPER);

  /* Initialize gpio pin */
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);


  CMU_ClockEnable(cmuClock_TIMER0, true);

  // Create the timer count control object initializer
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  timerCCInit.mode = timerCCModePWM;
  timerCCInit.cmoa = timerOutputActionToggle;

  // Configure CC channel 0
  TIMER_InitCC(TIMER_NUMBER, TIMER_CHANNEL, &timerCCInit);

  // Route CC0 to location 3 (PD1) and enable pin for cc0
  TIMER3->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3);

  // Set Top Value
  TIMER_TopSet(TIMER_NUMBER, TIMER_TOP);

  // Set the PWM duty cycle here
  TIMER_CompareBufSet(TIMER_NUMBER, TIMER_CHANNEL, DUTY_CYCLE);

  // Create a timerInit object, based on the API default
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.prescale = timerPrescale256;
  timerInit.debugRun = true;

  TIMER_Init(TIMER_NUMBER, &timerInit);

  while (1)
  {

  }
}
