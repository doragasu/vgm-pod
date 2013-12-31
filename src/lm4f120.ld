/******************************************************************************
 *
 * hello.ld - Linker configuration file for hello.
 *
 * Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
 *
 *****************************************************************************/

/************************************************************************//**
 * Defined symbols (accessible from code space):
 * _text:         Start of code and read only data.
 * _etext:        End of code and read only data.
 * _data:         Start of initialized variables and vector table.
 * _edata:        End of initialized variables and vector table.
 * _bss:          Start of uninitialized variable space.
 * _ebss:         End of uninitialized variable space.
 * _heap_botton:  Start of heap.
 * _stack_bottom: Start of stack/End of heap.
 * _stack_top:    End of stack.
 ****************************************************************************/

/************************************************************************//**
 * Available memory in the device:
 * - 256K Flash
 * - 32K RAM, partitioned as:
 *   + 28K work RAM (for variables + heap)
 *   + 4K stack
 ****************************************************************************/
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 32K - 4K
	STACK (rwx) : ORIGIN = 0x20007000, LENGTH = 4K
}

/************************************************************************//**
 * Sections:
 * - .text: For code and read only data. Stored at Flash memory.
 * - .data: Vector table and initialized variables. Stored at the end of the
 *          .text section. Copied to SRAM by the startup code.
 * - .bss : For uninitialized variables. Allocated at SRAM.
 ****************************************************************************/ 
SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > FLASH

	.ARM.exidx :
	{
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} > FLASH
    _etext = .;


    .data : AT(ADDR(.ARM.exidx) + SIZEOF(.ARM.exidx))
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

	/* Beginning of heap, just where used RAM ends */ 
	. = ALIGN(8);
    _heap_bottom = .;

	/* Beginning of stack */
    _stack_bottom = ORIGIN(STACK);
    /* End of stack (default value of the stack pointer) */
    _stack_top = ORIGIN(STACK) + LENGTH(STACK);
}
