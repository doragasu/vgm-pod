/************************************************************************//**
 * \file   rgb_led.h
 * \brief  Simple module to drive a RGB LED. This driver uses no PWM
 *         control, so available colors are limited to 7 + black.
 * \author Jesús Alonso (doragasu)
 * \date   2013
 ****************************************************************************/
/* This file is part of vmg-pod source package.
 *
 * vgm-pod is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Some open source application is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vgm-pod.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _RGB_LED_H_
#define _RGB_LED_H_

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/gpio.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>

// Definitions relative to the board port configuration
/// GPIO port used for the LEDs
#define RGBLED_GPIO_PERIPH	SYSCTL_PERIPH_GPIOF
/// Base of the GPIO port used for the LEDs
#define RGBLED_GPIO_PORT	GPIO_PORTF_BASE
/// GPIO pin for the red color
#define RGBLED_PIN_RED		GPIO_PIN_1
/// GPIO pin for the green color
#define RGBLED_PIN_GREEN	GPIO_PIN_3
/// GPIO pin for the blue color
#define RGBLED_PIN_BLUE		GPIO_PIN_2

/// Macro to build 3-bit RGB colors. Each input parameter must be 0 or 1.
#define RGBLED_COLOR(red,green,blue)	(((red)<<1)|((green)<<3)|((blue)<<2))

/**
 *  \defgroup colors LED colors
 *  color definitions, used with the RGBLED_COLOR() macro.
 *  \{
 */
/// Red color
#define RGBLED_RED			RGBLED_COLOR(1, 0, 0)
/// Green color
#define RGBLED_GREEN		RGBLED_COLOR(0, 1, 0)
/// Blue color
#define RGBLED_BLUE			RGBLED_COLOR(0, 0, 1)
/// Cyan color
#define RGBLED_CYAN			RGBLED_COLOR(0, 1, 1)
/// Magenta color
#define RGBLED_MAGENTA		RGBLED_COLOR(1, 0, 1)
/// Yellow color
#define RGBLED_YELLOW		RGBLED_COLOR(1, 1, 0)
/// Black color (LED off)
#define RGBLED_BLACK		RGBLED_COLOR(0, 0, 0)
/// White color
#define RGBLED_WHITE		RGBLED_COLOR(1, 1, 1)
/** \} */

/************************************************************************//**
 * \brief Module initialization. Must be called once, before using other
 *        functions in this module.
 ****************************************************************************/
#define RgbLedInit()														\
{																			\
	MAP_SysCtlPeripheralEnable(RGBLED_GPIO_PERIPH);							\
    MAP_GPIOPinTypeGPIOOutput(RGBLED_GPIO_PORT, RGBLED_PIN_RED |            \
    		                  RGBLED_PIN_BLUE | RGBLED_PIN_GREEN);	        \
}

/************************************************************************//**
 * \brief Sets the color of the LED. The color can be constructed using
 *        the macro RGBLED_COLOR().
 * \param color Color to set the LED. Can be constructed using the macro
 *        RGBLED_COLOR().
 ****************************************************************************/
#define RgbLedSet(color)	(MAP_GPIOPinWrite(RGBLED_GPIO_PORT, \
		RGBLED_PIN_RED | RGBLED_PIN_GREEN | RGBLED_PIN_BLUE, color))

#endif /*_RGB_LED_H_*/
