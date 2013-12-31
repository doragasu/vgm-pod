/************************************************************************//**
 * \file syscalls.c
 * \brief System calls implementation.
 *
 * Most system calls are implemented as stubs, but some basic functionality
 * has been coded. Most notably _sbrk (for heap allocation), _write (to be
 * able to write to the UART0 using printf, etc.), and _read (to be able to
 * read from the UART0 using scanf, etc.).
 *
 * \note Current implementation includes a non-blocking _write and a blocking
 * _read function.
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

#include <sys/stat.h>
#include <sys/times.h>
#include <sys/types.h>
#include <errno.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/interrupt.h>
#include "uart.h"
#undef errno

/// Error holder variable
extern int errno;
/// A pointer to a list of environment variables and their values.
/// For a minimal environment, this empty list is adequate:
char *__env[1] = { 0 };
char **environ = __env;
/// End of heap
char *heap_end = 0;

int _close(int file)
{
    return -1;
}

/// Create a new process. Minimal stub (for system without processes):
int _execve(char *name, char **argv, char **env) {
  errno = ENOMEM;
  return -1;
}

/// Create a new process. Minimal stub (for system without processes):
int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

/// All files are regarded as character special devices
int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

/// Get process ID. Only default process
int _getpid(void)
{
	return 1;
}

/// Query whether output stream is a terminal. For consistency with the other
/// minimal implementations, which only support output to stdout, this minimal
/// implementation is suggested:
int _isatty(int file)
{
    return 1;
}

/// Kill implementation. No processes, so error
int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

/// Link files. Unsupported
int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

/// Set a position in a file
int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/// Open a file. No filesystem supported
int _open(const char *name, int flags, int mode)
{
    return -1;
}

/// Read from file. The only file supported is the UART
/// \warning Blocking implementation
int _read(int file, char *ptr, int len)
{
    int i;

    for(i = 0; i < len; i++) ptr[i] = (char)MAP_UARTCharGet(UART0_BASE);

    return i;
}

/// Heap management
caddr_t _sbrk(unsigned int incr)
{
    /// Start of heap
	extern char _heap_bottom;
	/// Start of stack (also end of heap)
    extern char _stack_bottom;
	/// Temporal heap pointer
    char *prev_heap_end;
    tBoolean intStat;

    /// Disable interrupts to avoid heap corruption if _sbrk() is called from
    /// inside an interrupt.
    intStat = MAP_IntMasterDisable();

    /// Heap initialization
    if (heap_end == 0)
    	heap_end = &_heap_bottom;

    /// Store current heap pointer
    prev_heap_end = heap_end;

    if ((heap_end + incr) > &_stack_bottom)
    {
    	/// Error: Not enough heap space available
    	errno = ENOMEM;
    	/// Enable interrupts if they were already enabled when entering
    	if (intStat == false) MAP_IntMasterEnable();
    	/// \todo Should we return -1?
    	return (caddr_t)0;
    }

    /// Allocate requested heap
    heap_end += incr;

	/// Enable interrupts if they were already enabled when entering
    if (intStat == false) MAP_IntMasterEnable();
    /// Return pointer to allocated data
    return (caddr_t)prev_heap_end;
}

/*
//static char *heap_end = 0;
extern unsigned long _heap_bottom;
extern unsigned long _stack_bottom;

caddr_t _sbrk(unsigned int incr)
{
    char *prev_heap_end;
    if (heap_end == 0) {
        heap_end = (caddr_t)&_heap_bottom;            //1
    }
    prev_heap_end = heap_end;                         //2
    if (heap_end + incr > (caddr_t)&_stack_bottom) {      //3
        return (caddr_t)0;                            //4
    }
    heap_end += incr;
    return (caddr_t) prev_heap_end;                   //5
}
*/
/// Status of a file (by name). Minimal implementation:
int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

/// Timing information for current process. Minimal implementation:
int _times(struct tms *buf)
{
  return -1;
}

/// Remove a file's directory entry. Minimal implementation:
int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

/// Wait for a child process. Minimal implementation:
int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

/// Write to file. The only file supported is the UART.
/// \warning The UART uses interrupts for sending data, and to avoid blocking,
/// it is not checked whether the send process has finished before sending
/// another packet of data. If a send process is initiated before the previous
/// one has finished, unsent data from the first send process is lost. Also
/// the sent buffer MUST BE UNALTERED during the entire send process.
/// The function can be easily made blocking using Uart module callbacks.
int _write(int file, char *ptr, unsigned int len)
{
/*    unsigned int i;
    for(i = 0; i < len; i++) MAP_UARTCharPut(UART0_BASE, ptr[i]);
    return i;*/

	if (UartSend(0, ptr, len) == UART_BUSY) return -1;
	return len;
}
