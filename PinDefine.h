
#ifndef _PINDEFINE_H_
#define _PINDEFINE_H_

#include "Gpio.h"
#include "Gpio_map.h"

/* led pin */
#define PORT_PIN_LED1     1
#define OFFSET_PIN_LED1   6
#define PIN_LED1          MAKE_GPIO(PORT_PIN_LED1, OFFSET_PIN_LED1)

#define PORT_PIN_LED2     1
#define OFFSET_PIN_LED2   7
#define PIN_LED2          MAKE_GPIO(PORT_PIN_LED2, OFFSET_PIN_LED2)

#endif

