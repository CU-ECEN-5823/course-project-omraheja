/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();


#define __PB0_BUTTON_PORT	(gpioPortF)
#define __PB0_BUTTON_PIN 	(6)
#define __PB1_BUTTON_PORT	(gpioPortF)
#define __PB1_BUTTON_PIN	(7)

#endif /* SRC_GPIO_H_ */
