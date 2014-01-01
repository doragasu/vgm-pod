/************************************************************************//**
 * \file   vgm.h
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

#ifndef _VGM_H_
#define _VGM_H_

#include "types.h"

/** \addtogroup vgm_api
 *  \brief Parses VGM files (only Genesis/Megadrive and Master System ones)
 *         and sends commands to YM2612 and SN76489 chips.
 *  \{ */

/// Dirty trick to check things at compile time and error if check fails
#define COMPILE_TIME_ASSERT(expr) typedef u8 COMP_TIME_ASSERT[((!!(expr))*2-1)]

#define VGM_MAX_HEADLEN     192

#define VGM_MIN_HEADLEN     64

/// Function completed without error
#define VGM_OK               0
/// Function failed
#define VGM_ERROR           -1
/// An error opening/reading a file occurred
#define VGM_FILE_ERR        -2
/// The VGM header was not correct
#define VGM_HEAD_ERR        -3
/// The VGM stream was not correct
#define VGM_STREAM_ERR      -4
/// Unsupported VGM file
#define VGM_NOT_SUPPORTED   -5
/// Reached end of VGM stream
#define VGM_EOF             -6
/// Reached start of VGM stream
#define VGM_SOF             -7
/// Cannot complete request because module is busy
#define VGM_BUSY            -8

#define VGM_TIM_PERIPH      SYSCTL_PERIPH_TIMER0
#define VGM_TIM_BASE        TIMER0_BASE
#define VGM_TIM_HALF        TIMER_B
#define VGM_TIM_ENABLE_PERIPH

/// VGM header version 1.61
typedef struct
{
    u32 ident;
    u32 eofOffset;
    u32 version;
    u32 sn76489Clk;
    u32 ym2413Clk;
    u32 gd3Offset;
    u32 totalSamples;
    u32 loopOffset;
    u32 loopNSamples;
    // VGM 1.01 additions
    u32 rate;
    u16 snFeedback; // 0x0009 for versions prior to 1.01
    u8 snNfsrLen;   // 16 for versions prior to 1.01
    // VGM 1.51 additions
    union
    {
        u8 snFlags;
        struct
        {
            u8 freq400:1;
            u8 outNeg:1;
            u8 stereo:1;
            u8 div8:1;
        } sn;
    };
    // VGM 1.10 additions
    u32 ym2612Clk;
    u32 ym2151Clk;
    // VGM 1.50 additions
    u32 VgmStreamOff;   // 0x40 for versions prior to 1.50
    // VGM 1.51 additions
    u32 SegaPcmClk;
    u32 SegaPcmIfReg;
    u32 rf5c68Clk;
    u32 ym2203Clk;
    u32 ym2608Clk;
    u32 ym2610Clk;
    u32 ym3812Clk;
    u32 ym3526Clk;
    u32 y8950Clk;
    u32 ymf262Clk;
    u32 ymf278bClk;
    u32 ymf271Clk;
    u32 ymf280bClk;
    u32 rf5c164Clk;
    u32 pwmClk;
    u32 ay8910Clk;
    u8 ay8910Chip;
    u8 ay8910Flags;
    u8 ym2203Flags;
    u8 ym2608Flags;
    // VGM 1.60 additions
    u8 volModifier;
    u8 reserved1;
    u8 loopBase;
    // VGM 1.51 additions
    u8 loopModif;
    u32 GmbDmgClk;
    u32 NesApuClk;
    u32 MultiPcmClk;
    u32 upd7759Clk;
    u32 okim6258Clk;
    union
    {
        u8 okim6258Flags;
        struct
        {
            u8 clkDiv:2;
            u8 adpcm34bit:1;
            u8 out1012bit:1;
        } okim6258;
    };
    union
    {
        u8 k054539Flags;
        struct
        {
            u8 revStereo:1;
            u8 disReverb:1;
            u8 keyOnUpdate:1;
        } k054539;
    };
    u8 c140Type;
    u8 reserved2;
    u32 okim6295Clk;
    u32 k051649Clk;
    u32 k054539Clk;
    u32 huc6280Clk;
    u32 c140Clk;
    u32 k053260Clk;
    u32 pokeyClk;
    u32 qSoundClk;
    u32 reserved3;
    u32 reserved4;
} VgmHead;

typedef enum
{
    VGM_CLOSE,      ///< No VGM file is opened
    VGM_STOP,       ///< VGM file is opened, playback is stopped
    VGM_PLAY,       ///< VGM file is being played
    VGM_PAUSE,      ///< VGM playback is paused
    VGM_ERROR_STOP  ///< An error occurred while playing the file
} VgmStat;

/// Check header length is correct
COMPILE_TIME_ASSERT(sizeof(VgmHead) == VGM_MAX_HEADLEN);

#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************************//**
 * \brief Module initialization. Must be called once before any other
 * function.
 ****************************************************************************/
void VgmInit(void);

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
int VgmOpen(char fileName[]);

/************************************************************************//**
 * \brief Stars playing a previously opened VGM file.
 *
 * \return
 * - VGM_OK Playback has started
 * - VGM_BUSY Already busy playing a file
 * - VGM_ERROR Couldn't play file because it is not opened or has errors.
 ****************************************************************************/
int VgmPlay(void);

/************************************************************************//**
 * \brief Advances the VGM file stream pointer
 *
 * \param[in] timeMs Time to advance in milliseconds.
 * \return
 * - VGM_OK Stream successfully advanced
 * - VGM_EOF Reached end of file
 * - VGM_ERROR Couldn't advance because file is not opened or has errors.
 ****************************************************************************/
int VgmFf(u32 timeMs);

/************************************************************************//**
 * \brief Rewinds the VGM file stream pointer
 *
 * \param[in] timeMs Time to rewind in milliseconds.
 * \return
 * - VGM_OK Stream successfully rewinded
 * - VGM_SOF Reached start of file
 * - VGM_ERROR Couldn't rewind because file is not opened or has errors.
 ****************************************************************************/
int VgmRew(u32 timeMs);

/************************************************************************//**
 * \brief Pauses playback
 *
 * \return
 * - VGM_OK Playback paused
 * - VGM_ERROR Couldn't pause because file is not opened/playing.
 ****************************************************************************/
int VgmPause(void);

/************************************************************************//**
 * \brief Stops playback
 *
 * \return
 * - VGM_OK Playback stopped
 * - VGM_ERROR Couldn't stop because file is not opened/playing.
 ****************************************************************************/
int VgmStop(void);

/************************************************************************//**
 * \brief Closes VGM file
 *
 * \return
 * - VGM_OK VGM File closed
 * - VGM_BUSY Couldn't close file because it is being played.
 * - VGM_ERROR Couldn't close file because it is not opened and stopped.
 ****************************************************************************/
int VgmClose(void);

/************************************************************************//**
 * \brief Returns VGM file header.
 *
 * \return The VGM header of the opened file, or NULL if the file is not
 * properly opened.
 ****************************************************************************/
VgmHead *VgmGetHead(void);

/************************************************************************//**
 * \brief Returns playback cursor
 *
 * \return The playback cursor
 ****************************************************************************/
u32 VgmGetCursor(void);

/************************************************************************//**
 * \brief Returns the playback/module status
 *
 * \return The playback/module status
 ****************************************************************************/
VgmStat VgmGetStat(void);

#ifdef __cplusplus
}
#endif

/** \} */

#endif // _VGM_H_
