/************************************************************************//**
 * \file    ym2612.h
 * \brief   Handles Yamaha YM2612 synthesizer chip
 *
 * YM2612 chip must be wired using PB and PE as follows:
 * D0~D7 (2~9)  PB0~PB7
 * /IC   (11)   PE0
 * A0    (17)   PE1
 * A1    (18)   PE2
 * /RD   (16)   PE3
 * /WR   (15)   PE4
 * /CS   (14)   PE5
 *
 * \author  Jesús Alonso (doragasu)
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

/** \addtogroup ym2612_api
 *  \brief Module for controlling the YM2612 FM syntesizer. This module
 *  allows to use the YM2612 and write to its registers.
 *  \{ */


#ifndef _YM2612_H_
#define _YM2612_H_

#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************************//**
 * \brief Initializes the module, including hardware ports and YM2612 chip
 * itself. Must be called before using any other function in this module.
 ****************************************************************************/
void Ym2612Init(void);

/************************************************************************//**
 * \brief Writes a value to the specified port and register of the YM2612.
 *
 * \param[in] port Port number to write to. Can be 0 or 1.
 * \param[in] reg  YM2612 register to write to.
 * \param[in] val  Value to write to the YM2612.
 ****************************************************************************/
void Ym2612RegWrite(u8 port, u8 reg, u8 val);

#ifdef __cplusplus
}
#endif

#endif // _YM2612_H_

/** \} */
