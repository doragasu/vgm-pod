/************************************************************************//**
 * \file   uart.h
 * \brief  Module for controlling the microcontroller UARTs. The module uses
 *  UART FIFOs, and allows externally allocated buffers to be used.
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

/** \addtogroup uart_api
 *  \brief Module for controlling the microcontroller UARTs. The module uses
 *  UART FIFOs, and allows externally allocated buffers to be used.
 *  \{ */

#ifndef _UART_H_
#define _UART_H_

#include "types.h"

/// Number of UARTs available in the system/chip
#define UART_NUM_UARTS		8

/// OK return code
#define UART_OK		0
/// Busy return code. Indicates the UART is busy doing Tx or Rx
#define UART_BUSY	1

#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************************//**
 * \brief UART initialization.
 *
 * Sets the specified UART to the selected baudrate, 8 bits, no parity and 1
 * stop bit. This function must be called before the other functions in the
 * module.
 *
 * \param[in] uartNum Number of the UART to configure, from 0 to
 *            UART_NUM_UARTS.
 * \param[in] baudRate UART baud rate in bps.
 ****************************************************************************/
void UartInit(u8 uartNum, u32 baudRate);

/************************************************************************//**
 * \brief Sends a data buffer using the specified UART.
 *
 * Prepares a data buffer for sending it with the UART using interrupts. If
 * a callback function has been set with UartSendCompleteCallbackSet()
 * function, it will be called when the send process is finished (i.e., when
 * all the data has been copied to the TX FIFO.
 *
 * \param[in] uartNum Initialized UART to use for sending the data.
 * \param[in] data Pointer to the data buffer containing the data to send.
 * \param[in] bLen Number of bytes to send.
 * \return UART_OK if the send process has been properly started, UART_BUSY
 *         if the UART was already sending data and cannot start sending
 *         a new buffer.
 ****************************************************************************/
u8 UartSend(u8 uartNum, void *data, u32 bLen);

/************************************************************************//**
 * \brief Receives data from the specified UART.
 *
 * Prepares a data buffer for receiving data using the specified UART and
 * interrupts. Reception process ends when there is a UART Timeout interrupt,
 * or if recvFullBuff is TRUE, when maxLen bytes are received. If a callback
 * is configured with UartRecvCompleteCallbackSet(), it is called when the
 * reception process ends.
 *
 * \param[in] uartNum UART to use for receiving data.
 * \param[in] recvBuf Pointer to the buffer to deposit the received data in.
 * \param[in] maxLen Maximum data length to be received (in bytes).
 * \param[in] recvFullBuf If TRUE, receiving process will end only when
 *            maxLen bytes are received. If FALSE, an UART Timeout interrupt
 *            will also end the receiving process.
 * \warning Care must be taken when calling this function, because if it is
 *          called when it is already receiving a data buffer, previously
 *          received data will be discarded and a new receive process will be
 *          started.
 ****************************************************************************/
void UartRecv(u8 uartNum, void *recvBuf, u32 maxLen, u8 recvFullBuf);

/************************************************************************//**
 * \brief Checks if the specified UART is sending data.
 *
 * \param[in] uartNum Number of the UART to check.
 * \return UART_BUSY if the specified UART is sending data.
 *         UART_OK otherwise.
 ****************************************************************************/
u8 UartSendBusy(u8 uartNum);

/************************************************************************//**
 * \brief Checks if the specified UART is receiving data.
 *
 * \param[in] uartNum Number of the UART to check.
 * \return UART_BUSY if the specified UART is receiving data.
 *         UART_OK otherwise.
 ****************************************************************************/
u8 UartRecvBusy(u8 uartNum);

/************************************************************************//**
 * \brief Sets a callback function that will be called when a receive
 *        process is finished. The called callback must take an input
 *        parameter with the number of received bytes.
 *
 * \param[in] uartNum Number of the UART to set the callback for.
 * \param[in] RxCallback Callback function. It must accept an input
 *            parameter with the number of received bytes.
 * \note To remove a previously set callback, call this function setting to
 *       NULL RxCallback parameter.
 ****************************************************************************/
void UartRecvCompleteCallbackSet(u8 uartNum, void (*RxCallback)(int recvd));

/************************************************************************//**
 * \brief Sets a callback function that will be called when a send
 *        process is finished.
 *
 * \param[in] uartNum Number of the UART to set the callback for.
 * \param[in] TxCallback Callback function.
 * \note To remove a previously set callback, call this function setting to
 *       NULL TxCallback parameter.
 ****************************************************************************/
void UartSendCompleteCallbackSet(u8 uartNum, void (*TxCallback)(void));

/************************************************************************//**
 * \brief Flushes the reception FIFO buffer.
 *
 * \param[in] uartNum The UART whose FIFO RX buffer will be flushed.
 ****************************************************************************/
void UartRxFifoFlush(u8 uartNum);

/// \warning must be called before initialization. Use with extreme caution!
void UartCustomIntRegister(u8 uartNum, void (*IntCallback)(void));
/// \warning Use with extreme caution!
void UartCustomIntEnable(u8 uartNum, unsigned long interrupts);

unsigned long UartGetBase(u8 uartNum);

#ifdef __cplusplus
}
#endif
#endif /*_UART_H_*/

/** \} */
