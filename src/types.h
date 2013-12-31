/************************************************************************//**
 * \file	types.h
 * \brief	The usual datatype definitions, used for portability.
 * \author	Jesús Alonso (doragasu)
 * \date	2013
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
#ifndef _TYPES_H_
#define _TYPES_H_

/// Unsigned 8-bit datatype
typedef unsigned char u8;
/// Signed 8-bit datatype
typedef signed char i8;
/// Unsigned 16-bit datatype
typedef unsigned short u16;
/// Signed 16-bit datatype
typedef signed short i16;
/// Unsigned 32-bit datatype
typedef unsigned int u32;
/// Signed 32-bit datatype
typedef signed int i32;
/// Unsigned 64-bit datatype
typedef unsigned long long u64;
/// Signed 64-bit datatype
typedef long long i64;

#ifndef TRUE
/// Value for TRUE comparisons
#define TRUE 1
#endif
#ifndef FALSE
/// Value for FALSE comparisons
#define FALSE 0
#endif

typedef union
{
	u32 dw;
	u8 b[4];
} QBYTE;

/// NULL definition. Should work for both C and C++
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif /*NULL*/

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif /*TRUE*/

#endif /*_TYPES_H_*/
