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
//#include "em_dac.h"
//#include "em_acmp.h"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_dac.c"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_acmp.c"


uint8_t 	counter		= 0;
uint32_t 	DUTY_CYCLE	= 0;
uint32_t 	TIMBUF;
float 		voltage1=0, voltage0=0;
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
 static void ACMPConfig(void)
 {
   ACMP_Init_TypeDef acmp_init    = ACMP_INIT_DEFAULT;

   CMU_ClockEnable(cmuClock_ACMP0, true);

   ACMP_Init(ACMP0, &acmp_init);
   ACMP_ChannelSet(ACMP0, acmpChannelDAC0Ch1, acmpChannel6);
 }

 static void DACConfig(void)
 {
	  /* Use default settings */
	  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
	  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

	  CMU_ClockEnable(cmuClock_DAC0, true);


	  init.prescale = DAC_PrescaleCalc(500000, 0);
	  init.reference = dacRef1V25;
	  init.outMode = dacOutputPinADC;
	  /* Initialize the DAC and DAC channel. */
	  DAC_Init(DAC0, &init);

	  DAC_InitChannel(DAC0, &initChannel, 0);
	  DAC_InitChannel(DAC0, &initChannel, 1);
 }


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{


  uint32_t DAC_Value;
  /* Chip errata */
  CHIP_Init();

  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_HFPER) / 1000)) while (1) ;

  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortB,9,gpioModeInputPull,1);
  GPIO_PinModeSet(gpioPortB,10,gpioModeInputPull,1);

  /* Initialize LED driver */
  BSP_LedsInit();

  DACConfig();
  ACMPConfig();
  ACMP_Enable(ACMP0);
  DAC_Enable(DAC0,0,1);
  DAC_Enable(DAC0,1,1);

  while (1)
  {

	    if( !(GPIO_PinInGet(gpioPortB, 9)) )
		{
	    	Delay(100);
		    if( !(GPIO_PinInGet(gpioPortB, 9)) )
		    {
				voltage0+=0.1;
				if(voltage0>1.25) voltage0=0;
				DAC_Value = (uint32_t)((voltage0 * 4096) / 1.25);
				DAC_ChannelOutputSet(DAC0,0,DAC_Value);
	//			BSP_LedSet(1);
	//			BSP_LedClear(0);
		    }
	    }
	    if( !(GPIO_PinInGet(gpioPortB, 10)) )
		{
	    	Delay(100);
		    if( !(GPIO_PinInGet(gpioPortB, 10)) )
		    {
				voltage1+=0.1;
				if(voltage1>1.25) voltage1=0;
				DAC_Value = (uint32_t)((voltage1 * 4096) / 1.25);
				DAC_ChannelOutputSet(DAC0,1,DAC_Value);
	//			BSP_LedClear(1);
	//			BSP_LedSet(0);
		    }
		}

		if(ACMP0->STATUS & ACMP_STATUS_ACMPOUT)
		{
			BSP_LedSet(1);
			BSP_LedClear(0);
		}
		else
		{
			BSP_LedClear(1);
			BSP_LedSet(0);
		}
	}

}

