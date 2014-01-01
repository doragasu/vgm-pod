/************************************************************************//**
 * \file   vgm.c
 * \brief  Parses VGM files (only Genesis/Megadrive and Master System ones)
 *         and sends commands to YM2612 and SN76489 chips.
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
#include <string.h>
#include "vgm.h"
#include "fatfs/ff.h"
#include "ym2612.h"

/** \addtogroup vgm_api
 *  \brief Parses VGM files (only Genesis/Megadrive and Master System ones)
 *         and sends commands to YM2612 and SN76489 chips.
 *  \{ */

typedef struct
{
    VgmHead h;
    FIL f;
    VgmStat s;
} VgmData;

/// VGM module datam
static VgmData vd;

void VgmTimerHandler(void)
{
    /// Clear interrupt flag
#if (VGM_TIM_HALF == TIMER_A)
    MAP_TimerIntClear(VGM_TIM_BASE, TIMER_TIMA_TIMEOUT);
#else
    MAP_TimerIntClear(VGM_TIM_BASE, TIMER_TIMB_TIMEOUT);
#endif
}

/************************************************************************//**
 * \brief Module initialization. Must be called once before any other
 * function.
 ****************************************************************************/
void VgmInit(void)
{
    /// Initialize submodules
    Ym2612Init();

    /// Configure 48 kHz timer
#ifdef VGM_TIM_ENABLE_PERIPH
    MAP_SysCtlPeripheralEnable(VGM_TIM_PERIPH);
#endif // VGM_TIM_ENABLE_PERIPH

#if (VGM_TIM_HALF == TIMER_A)
    MAP_TimerConfigure(VGM_TIM_BASE, TIMER_CFG_A_PERIODIC);
#elif (VGM_TIM_HALF == TIMER_B)
    MAP_TimerConfigure(VGM_TIM_BASE, TIMER_CFG_B_PERIODIC);
#else
#error "Timer half not properly defined"
#endif
    MAP_TimerLoadSet(VGM_TIM_BASE, VGM_TIM_HALF, MAP_SysCtlClockGet()/48000);
//    MAP_TimerEnable(VGM_TIM_BASE, VGM_TIM_HALF);
    /// \todo Set it statically?
    TimerIntRegister(VGM_TIM_BASE, VGM_TIM_HALF, VgmTimerHandler);
#if (VGM_TIM_HALF == TIMER_A)
    MAP_TimerIntEnable(VGM_TIM_BASE, TIMER_TIMA_TIMEOUT);
#else
    MAP_TimerIntEnable(VGM_TIM_BASE, TIMER_TIMB_TIMEOUT);
#endif

}

/************************************************************************//**
 * \brief Opens a VGM file and parses its header, to get ready to play it.
 *
 * \param[in] fileName Name of the file to open.
 * \return
 * - VGM_OK File has been opened and parsed correctly.
 * - VGM_FILE_ERR File couldn't be opened and read correctly.
 * - VGM_HEAD_ERR VGM header checks failed.
 * - VGM_STREAM_ERR Stream format is not correct.
 * - VGM_NOT_SUPPORTED VGM header looks correct but file is not supported.
 ****************************************************************************/
int VgmOpen(char fileName[])
{
    UINT readed;
    /// Set some default values
    memset(&vd.h, 0, VGM_MAX_HEADLEN);
    vd.h.snFeedback = 0x0009;
    vd.h.snNfsrLen = 16;
    /// Open the file
    vd.s = VGM_CLOSE;
    if (f_open(&vd.f, fileName, FA_READ | FA_OPEN_EXISTING))
        return VGM_FILE_ERR;
    /// Read fields of 1.00 VGM header, checking for errors
    if (f_read(&vd.f, &vd.h, VGM_MIN_HEADLEN, &readed))
    {
        f_close(&vd.f);
        return VGM_FILE_ERR;
    }
    if (readed < VGM_MIN_HEADLEN)
    {
        f_close(&vd.f);
        return VGM_HEAD_ERR;
    }
    /// Check for file identification "VGM " string
    if (0x206D6756 != vd.h.ident)
    {
        f_close(&vd.f);
        return VGM_HEAD_ERR;
    }
    /// Read extra fields if header version is 1.51 or greater
    if (1.51 <= vd.h.version)
    {
        if (f_read(&vd.f, &vd.h.rf5c68Clk, VGM_MAX_HEADLEN - VGM_MIN_HEADLEN,
                &readed) || ((VGM_MAX_HEADLEN - VGM_MIN_HEADLEN) != readed))
        {
            f_close(&vd.f);
            return VGM_HEAD_ERR;
        }
    }
    /// Check this is a Master System or Megadrive/Genesis VGM file
    if (!vd.h.ym2612Clk && !vd.h.sn76489Clk)
    {
        f_close(&vd.f);
        return VGM_HEAD_ERR;
    }
    /// File OK, go to stop state
    vd.s = VGM_STOP;
    return VGM_OK;
}

/************************************************************************//**
 * \brief Stars playing a previously opened VGM file.
 *
 * \return
 * - VGM_OK Playback has started
 * - VGM_BUSY Already busy playing a file
 * - VGM_ERROR Couldn't play file because it is not opened or has errors.
 ****************************************************************************/
int VgmPlay(void)
{
    switch (vd.s)
    {
        case VGM_ERROR_STOP:
        case VGM_CLOSE: return VGM_ERROR;
        case VGM_PLAY: return VGM_BUSY;
        case VGM_STOP:
            // Go to start of data, preload a data sector
            // Load samples if any
        case VGM_PAUSE:
            vd.s = VGM_PLAY;
            // Enable timer. It will do all the work
            return VGM_OK;
    }
    return VGM_OK;
}

/************************************************************************//**
 * \brief Advances the VGM file stream pointer
 *
 * \param[in] timeMs Time to advance in milliseconds.
 * \return
 * - VGM_OK Stream successfully advanced
 * - VGM_EOF Reached end of file
 * - VGM_ERROR Couldn't advance because file is not opened or has errors.
 ****************************************************************************/
int VgmFf(u32 timeMs)
{
    return VGM_OK;
}

/************************************************************************//**
 * \brief Rewinds the VGM file stream pointer
 *
 * \param[in] timeMs Time to rewind in milliseconds.
 * \return
 * - VGM_OK Stream successfully rewinded
 * - VGM_SOF Reached start of file
 * - VGM_ERROR Couldn't rewind because file is not opened or has errors.
 ****************************************************************************/
int VgmRew(u32 timeMs)
{
    return VGM_OK;
}

/************************************************************************//**
 * \brief Pauses playback
 *
 * \return
 * - VGM_OK Playback paused
 * - VGM_ERROR Couldn't pause because file is not opened/playing.
 ****************************************************************************/
int VgmPause(void)
{
    return VGM_OK;
}

/************************************************************************//**
 * \brief Stops playback
 *
 * \return
 * - VGM_OK Playback stopped
 * - VGM_ERROR Couldn't stop because file is not opened/playing.
 ****************************************************************************/
int VgmStop(void)
{
    return VGM_OK;
}

/************************************************************************//**
 * \brief Closes VGM file
 *
 * \return
 * - VGM_OK VGM File closed
 * - VGM_BUSY Couldn't close file because it is being played.
 * - VGM_ERROR Couldn't close file because it is not opened and stopped.
 ****************************************************************************/
int VgmClose(void)
{
    return VGM_OK;
}

/************************************************************************//**
 * \brief Returns VGM file header.
 *
 * \return The VGM header of the opened file, or NULL if the file is not
 * properly opened.
 ****************************************************************************/
VgmHead *VgmGetHead(void)
{
    return &vd.h;
}

/************************************************************************//**
 * \brief Returns playback cursor
 *
 * \return The playback cursor
 ****************************************************************************/
u32 VgmGetCursor(void)
{
    return 0;
}

/************************************************************************//**
 * \brief Returns the playback/module status
 *
 * \return The playback/module status
 ****************************************************************************/
VgmStat VgmGetStat(void)
{
    return vd.s;
}

/** \} */
