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

#include "vgm.h"

/** \addtogroup vgm_api
 *  \brief Parses VGM files (only Genesis/Megadrive and Master System ones)
 *         and sends commands to YM2612 and SN76489 chips.
 *  \{ */

/// VGM header of the file being processed
static VgmHead vh;

/// VGM module status
static VgmStat vs = VGM_CLOSE;

/************************************************************************//**
 * \brief Opens a VGM file and parses its header, to get ready to play it.
 *
 * \param[in] fileName Name of the file to open.
 * \return
 * - VGM_OK If file has been opened and parsed correctly.
 * - VGM_FILE_ERR if file couldn't be opened and read correctly.
 * - VGM_HEAD_ERR if VGM header checks failed.
 * - VGM_STREAM_ERR if stream format was not correct.
 ****************************************************************************/
int VgmOpen(char fileName[])
{
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
    return &vh;
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
    return vs;
}

/** \} */
