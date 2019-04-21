/*@Filename	 : gpio.c
 *@Author	 : Om Raheja
 *@Course	 : IoT Embedded Firmware [Spring 2019]
 *@Project   : MINE SAFETY USING BLUETOOTH MESH
 *@References: Bluetooth Mesh sample example codes [Light and Switch] have been used as references.
 * */

/* Includes */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "display.h"
#include "log.h"

/* Defines */
#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5


/* Initialize GPIO pins */
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

	/* Initialize GPIO for Buzzer */
	GPIO_PinModeSet(BUZZ_PORT, BUZZ_PIN, gpioModePushPull, false);

	/* Initialize GPIO for Noise Sensor */
	GPIO_PinModeSet(NOISE_PORT, NOISE_PIN, gpioModeInputPull, true);
}

/* Initialize Interrupt for GPIO pins */
void gpioIntEnable()
{
	/* Noise sensor interrupt enable at falling edge */
	GPIO_IntConfig(NOISE_PORT, NOISE_PIN, 0, true, true);

	/* PB0 interrupt configuration for falling edge */
	GPIO_IntConfig(__PB0_BUTTON_PORT, __PB0_BUTTON_PIN, 0, true, true);

	/* NVIC Enable */
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/* even gpio pin interrupts */
void GPIO_EVEN_IRQHandler(void)
{
	LOG_INFO("IN EVEN IRQ HANDLER\n\r");

	uint32_t flag;
	flag = GPIO_IntGet();
	GPIO_IntClear(flag);

	// pb0 interrupt
	if(flag & 0x40)
		gecko_external_signal(PB0_FLAG);
}

/* odd gpio pin interrupts */
void GPIO_ODD_IRQHandler(void)
{
	LOG_INFO("IN ODD IRQ HANDLER\n\r");

	uint32_t flag;
	flag = GPIO_IntGet();
	GPIO_IntClear(flag);

	// noise interrupt
	if(flag & 0x800)
		gecko_external_signal(NOISE_FLAG);
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

/* To Enable LCD */
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
