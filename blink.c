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
//#include "em_adc.h"
//#include "em_timer.h"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_timer.c"
#include "/home/taimoor/SimplicityStudio_v4/developer/sdks/exx32/v4.4.1/emlib/src/em_adc.c"


uint8_t 	counter		= 0;
uint32_t 	DUTY_CYCLE	= 0;
uint32_t 	TIMBUF;
float 		voltage=0;
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
 static void ADCConfig(void)
 {
   ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
   ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

   init.timebase = ADC_TimebaseCalc(0);
   init.prescale = ADC_PrescaleCalc(7000000, 0);
   ADC_Init(ADC0, &init);

   /* Init for single conversion use, measure VDD/3 with 1.25 reference. */
   singleInit.reference  = adcRef5VDIFF;
   singleInit.input      = adcSingleInpCh0;
   singleInit.resolution = adcRes12Bit;
   //singleInit.rep		= true;
   singleInit.acqTime = adcAcqTime64;

   ADC_InitSingle(ADC0, &singleInit);
 }

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{


  uint32_t sample;
  /* Chip errata */
  CHIP_Init();

  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_HFPER) / 1000)) while (1) ;

  /* Initialize peripherals clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  CMU_ClockEnable(cmuClock_ADC0, true);


//  GPIO_PinModeSet(gpioPortD, 0 , gpioModeInput, 0);


  /* Initialize LED driver */
  BSP_LedsInit();
  BSP_LedSet(0);

  ADCConfig();


  while (1)
  {
	    ADC_Start(ADC0, adcStartSingle);

	    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

		sample = ADC_DataSingleGet(ADC0);
		voltage = sample * 5;
		voltage /= 4096;
		if(voltage <  2.6 && voltage > 0)
		{
			BSP_LedClear(1);
			BSP_LedSet(0);

		}
		if(voltage >= 2.6 && voltage < 3.4)
		{
			BSP_LedSet(1);
			BSP_LedClear(0);

		}
		if(voltage >= 3.4)
		{
			BSP_LedSet(0);
			BSP_LedSet(1);
		}
	}

}

