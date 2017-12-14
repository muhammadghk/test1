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
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_timer.c"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_prs.c"

#define TOP 0xFFFF

long count = 0, totalTime;
char Capdone=0;

void TIMER0_IRQHandler(void)
{
  uint16_t intFlags = TIMER_IntGet(TIMER0);

  TIMER_IntClear(TIMER0, TIMER_IF_OF | TIMER_IF_CC0);

  if(intFlags & TIMER_IF_OF)
  {
    count += TOP;
  }

  if(intFlags & TIMER_IF_CC0)
  {
    totalTime = count + TIMER_CaptureGet(TIMER0, 0);

    totalTime = totalTime / 14; // time in us

    Capdone=1;
    /* Clear counter */
    count = 0;
   }
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_HFPER) / 1000)) while (1) ;


  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInputPullFilter, 1);
  GPIO_IntConfig(gpioPortB, 9, false, false, false);

  /* Enable PRS sense on GPIO and disable interrupt sense */
  GPIO_InputSenseSet(GPIO_INSENSE_PRS, _GPIO_INSENSE_RESETVALUE);

  CMU_ClockEnable(cmuClock_PRS, true);

  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN9, prsEdgeOff);


  /* timer init*/
  CMU_ClockEnable(cmuClock_TIMER0, true);

  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  timerCCInit.prsSel     = timerPRSSELCh0,
  timerCCInit.mode       = timerCCModeCapture,
  timerCCInit.filter     = true,
  timerCCInit.prsInput   = true,
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.enable     = false,
  timerInit.debugRun   = true,
  timerInit.prescale   = timerPrescale1,
  timerInit.clkSel     = timerClkSelHFPerClk,
  timerInit.fallAction = timerInputActionReloadStart,
  timerInit.riseAction = timerInputActionStop,
  timerInit .mode      = timerModeUp,
  TIMER_Init(TIMER0, &timerInit);


  TIMER_IntEnable(TIMER0, TIMER_IF_OF | TIMER_IF_CC0);
  NVIC_EnableIRQ(TIMER0_IRQn);


  while (1)
  {
	  if(Capdone)
	  {
		  Capdone=0;
	  }
	  EMU_EnterEM1();

  }
}

