/* @Author     : Om Raheja
 * @Date       : 30th January 2019
 * @Filename   : letimer.c
 * @Description: This file contains a function to initialize the LETIMER0 for different energy modes.
 * 				 An Interrupt routine for LETIMER is written to schedule an event and perform the task.
 * @IoT Embedded Firmware(Spring 2019)
 * */

/*Header files*/
#include "letimer.h"
#include "em_core.h"

/*Global variables*/
uint32_t Ticks;

/*LETIMER initialization function*/
void letimer_init(void)
{
	/* IF ENERGY STATE EM 0/1/2 IS CHOSEN */
	#if ((ENERGY_STATE == 0) | (ENERGY_STATE == 1) | (ENERGY_STATE == 2))

		Ticks = TICKS;

		/* Enable/Disable Oscillator */
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

		/* Select LFXO as clock source for LFA */
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

		/* PreScaling by 4*/
		CMU_ClockDivSet(cmuClock_LETIMER0,cmuClkDiv_2);

	#endif

	/* IF ENERGY STATE EM3 IS CHOSEN */
	#if (ENERGY_STATE == 3)

		Ticks = TICKS_EM3;

		/* Frequency for ULFRCO oscillator */
		//#define FREQ         (1.000)

		/* Enable/Disable Oscillator */
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);

		/* Select ULFRCO as clock source for LFA */
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

	#endif


	/*Enable the desired clock*/
	CMU_ClockEnable(cmuClock_LETIMER0,true);

	/* This part of the code was taken from the below website
	 * (https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/08/17/using_low_energytim-9RYB).
	 */

	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
			.enable         = false,
			.comp0Top       = true                   	/* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	};

	//Initialize letimer0
	LETIMER_Init(LETIMER0, &letimerInit);

	//ADDED NOW
//	Ticks = (3000/1000) * (CMU_ClockFreqGet(cmuClock_LETIMER0));

	//setting values for comp0 and comp1
	LETIMER_CompareSet(LETIMER0, 0 , Ticks);

	//Enable interrupt flags
	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);

	//Enable NVIC
	// LETIMER0_IRQn= 27, /*!< 16+27 EFR32 LETIMER0 Interrupt */
	NVIC_EnableIRQ(LETIMER0_IRQn);
}

/* LETIMER0 IRQ Handler */
void LETIMER0_IRQHandler(void)
{

	//Get pending LETIMER interrupt flags
	uint32_t reason = LETIMER_IntGet(LETIMER0);

	//Clear one or more pending LETIMER interrupts
	LETIMER_IntClear(LETIMER0,reason);

	//Check for underflow condition
	if((reason & LETIMER_IF_UF))
	{
//		timer_events.power_on_setup_timer_event = true;
//		timer_events.def_event = false;
//		gecko_external_signal(timer_events.power_on_setup_timer_event);
		gecko_external_signal(UF_FLAG);
	}

	//Check for comp1 condition
//	if((reason & LETIMER_IF_COMP1))
//	{
////		timer_events.timer_expired = true;
////		timer_events.def_event = false;
////		gecko_external_signal(timer_events.timer_expired);
//		gecko_external_signal(COMP1_FLAG);
//		LETIMER_IntDisable(LETIMER0,LETIMER_IFC_COMP1);
//	}
}
