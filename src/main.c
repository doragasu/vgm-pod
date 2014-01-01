/************************************************************************//**
 * \file   main.c
 * \brief  FatFs for Stellaris Launchpad simple test program.
 * \author Jesús Alonso (doragasu)
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

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_timer.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <stdio.h>
#include "types.h"
#include "fatfs/ff.h"
#include "rgb_led.h"
#include "uart.h"
#include "vgm.h"

#define HELLO_STR       "Hello World!\r\n"
#define HELLO_STR_LEN   (sizeof(HELLO_STR) - 1)

/// Disk process, defined inside mmc.c (fatfs)
extern void disk_timerproc(void);


void SysTimerHandler(void)
{
    /// Clear interrupt flag
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    /// Call disk process
    disk_timerproc();
}

void SysTimerConfig(void)
{
    /// Enable TIMER0
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    /// Configure TIMER0
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE,TIMER_A, MAP_SysCtlClockGet()/100);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
    /// \todo Set it statically?
    TimerIntRegister(TIMER0_BASE, TIMER_A, SysTimerHandler);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}


// Couldn't manage to build UTF version without warnings. Trying
// --fwide-exec-charset=UTF-16 build flag doesn't help.
int main(void)
{
    FATFS fs;
    FIL f;
    FRESULT returned;
    int i;

	/// Set clock to 66,67 MHz (200 MHz / 3)
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_3 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    /// LED initialization
    RgbLedInit();

    /// Initialize UART0 (used for stdout)
    UartInit(0, 115200);
    printf("Debug console\r\n");

    /// Color cycle the LED. Ends with red color.
    for (i = 7; i > 0; i--)
    {
    	MAP_SysCtlDelay(2500000);
    	RgbLedSet(i<<1);
    }

    /// Mount filesystem
    if ((returned = f_mount(&fs, _T("/"), 0)))
    {
        printf("ERROR: couldn't mount volume.\r\n");
        return 1;
    }

    f_sync(&f);
   return 0;
}
