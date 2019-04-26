/*@Filename	 : gpio.c
 *@Author	 : Om Raheja
 *@Course	 : IoT Embedded Firmware [Spring 2019]
 *@Project   : MINE SAFETY USING BLUETOOTH MESH
 *@References: Bluetooth Mesh sample example codes [Light and Switch] have been used as references.
 * */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

/* Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "main.h"

/* Function Prototypes */
void gpioInit();
void gpioIntEnable();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);

/* Defines */
#define __PB0_BUTTON_PORT	(gpioPortF)
#define __PB0_BUTTON_PIN 	(6)
#define __PB1_BUTTON_PORT	(gpioPortF)
#define __PB1_BUTTON_PIN	(7)
#define PUSH_BUTTON_STATUS		0x20

/* Defines for Buzzer */
#define BUZZ_PORT (gpioPortA)
#define BUZZ_PIN (2)

/* Defines for Noise sensor */
#define NOISE_PORT (gpioPortD)
#define NOISE_PIN (11)

#endif /* SRC_GPIO_H_ */
