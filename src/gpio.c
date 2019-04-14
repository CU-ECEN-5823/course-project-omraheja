/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "display.h"
#include "log.h"

#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5

void gpioInit()
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	/* Set Push button pins */
	GPIO_PinModeSet(__PB0_BUTTON_PORT, __PB0_BUTTON_PIN, gpioModeInputPull, true);
	GPIO_PinModeSet(__PB1_BUTTON_PORT, __PB1_BUTTON_PIN, gpioModeInputPull, true);

}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

/* Enable LCD */
void gpioEnableDisplay()
{
	GPIO_PinOutSet(gpioPortD, 15);
}

void gpioSetDisplayExtcomin(bool high)
{
	if (high == false)
	{
		GPIO_PinOutClear(gpioPortD, 13);
	}

	if (high == true)
	{
		GPIO_PinOutSet(gpioPortD, 13);
	}
}
