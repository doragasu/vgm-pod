/************************************************************************//**
 * \file   uart.c
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

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_uart.h>
#include <inc/hw_gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include "uart.h"

/// Defines a data buffer for the Uart
typedef struct
{
	u8 *data;		///< Pointer to the data buffer
	u32 maxLen;		///< Maximum lenght of the buffer
	u32 idx;		///< Current index in the data buffer
} UartBuf;

/// Data required to manage a UART
typedef struct
{
	unsigned long base;				///< Base dir for the UART
	unsigned long periph;			///< Peripheral for the UART
	UartBuf txBuf;					///< Buffer for sending data
	UartBuf rxBuf;					///< Buffer for receiving data
	void (*TxCompleteCallback)(void);		///< Reception callback
	void (*RxCompleteCallback)(int recvd);	///< Send callback
	u8 intNum;						///< Interrupt number for the UART
} UartData;

/// Data structures for all the available UARTs
static UartData uart[UART_NUM_UARTS];

/// Allowed base addresses for the UARTs
static const unsigned long uartBase[UART_NUM_UARTS] =
{
    UART0_BASE, UART1_BASE, UART2_BASE, UART3_BASE,
    UART4_BASE, UART5_BASE, UART6_BASE, UART7_BASE
};

/// Allowed peripherals used by the UARTs
static const unsigned long uartPeriph[UART_NUM_UARTS] =
{
    SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_UART2,
    SYSCTL_PERIPH_UART3, SYSCTL_PERIPH_UART4, SYSCTL_PERIPH_UART5,
    SYSCTL_PERIPH_UART6, SYSCTL_PERIPH_UART7
};

/// Allowed interrupt numbers for the UARTs
static const u8 uartInt[UART_NUM_UARTS] =
{
    INT_UART0, INT_UART1, INT_UART2, INT_UART3,
    INT_UART4, INT_UART5, INT_UART6, INT_UART7
};

/// Peripheral corresponding to each UART
static const unsigned long uartGpioPeriph[UART_NUM_UARTS] =
{
	SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOD,
	SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOE,
	SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOE
};

/// Base for each UART
static const unsigned long uartGpioBase[UART_NUM_UARTS] =
{
	GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE, GPIO_PORTC_BASE,
	GPIO_PORTC_BASE, GPIO_PORTE_BASE, GPIO_PORTD_BASE, GPIO_PORTE_BASE
};

/// Pins used for Rx by each UART
static const u8 uartRxGpioPin[UART_NUM_UARTS] =
{
	GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_6, GPIO_PIN_6,
	GPIO_PIN_4, GPIO_PIN_4, GPIO_PIN_4, GPIO_PIN_0
};

/// Pins used for Tx by each UART
static const u8 uartTxGpioPin[UART_NUM_UARTS] =
{
	GPIO_PIN_1, GPIO_PIN_1, GPIO_PIN_7, GPIO_PIN_7,
	GPIO_PIN_5, GPIO_PIN_5, GPIO_PIN_5, GPIO_PIN_1
};

/// Mux values for each UART and for the pins defined in uartRxGpioPin
static const unsigned long uartRxMuxPin[UART_NUM_UARTS] =
{
	GPIO_PA0_U0RX, GPIO_PB0_U1RX, GPIO_PD6_U2RX, GPIO_PC6_U3RX,
	GPIO_PC4_U4RX, GPIO_PE4_U5RX, GPIO_PD4_U6RX, GPIO_PE0_U7RX
};

/// Mux values for each UART and for the pins defined in uartTxGpioPin
static const unsigned long uartTxMuxPin[UART_NUM_UARTS] =
{
	GPIO_PA1_U0TX, GPIO_PB1_U1TX, GPIO_PD7_U2TX, GPIO_PC7_U3TX,
	GPIO_PC5_U4TX, GPIO_PE5_U5TX, GPIO_PD5_U6TX, GPIO_PE1_U7TX
};

/************************************************************************//**
 * \brief Handles RX and RT interrupts for all the UARTs.
 *
 * \param[in] ud Ponter to the UartData structure of the UART originating the
 *            interrupt.
 * \param[in] forceEnd If TRUE, received process will be finished
 *            independently of the amount of bytes received.
 ****************************************************************************/
void UartIntRx(UartData *ud, int forceEnd)
{
	/// Extract the contents of the RX FIFO
	while ((ud->rxBuf.idx < ud->rxBuf.maxLen) && MAP_UARTCharsAvail(ud->base))
	{
		ud->rxBuf.data[ud->rxBuf.idx++] =
			MAP_UARTCharGetNonBlocking(ud->base);
	}
	/// Check if we have finished receiving data
	if (forceEnd || (ud->rxBuf.idx >= ud->rxBuf.maxLen))
	{
		/// When finished, disable RX and RT interrupts, mark the end of
		/// reception, and call the callback if requested.
		ud->rxBuf.maxLen = 0;
		MAP_UARTIntDisable(ud->base, UART_INT_RX | UART_INT_RT);
		if (ud->RxCompleteCallback) ud->RxCompleteCallback(ud->rxBuf.idx);
	}
}

/************************************************************************//**
 * \brief Handles TX interrupts for all the UARTs.
 *
 * \param[in] ud Ponter to the UartData structure of the UART originating the
 *            interrupt.
 ****************************************************************************/
void UartIntTx(UartData *ud)
{
	UartBuf *txb = &(ud->txBuf);

	/// Extract the data with the UART interrupts disabled, to avoid
	/// race conditions
	MAP_IntDisable(ud->intNum);
	while ((txb->idx < txb->maxLen) && MAP_UARTSpaceAvail(ud->base))
		MAP_UARTCharPutNonBlocking(ud->base, txb->data[txb->idx++]);
	MAP_IntEnable(ud->intNum);

	/// If all data sent...
	if (txb->idx >= txb->maxLen)
	{
		/// Disable TX Interrupts, mark the end of reception
		/// and call the callback if requested
		MAP_UARTIntDisable(ud->base, UART_INT_TX);
		ud->txBuf.maxLen = 0;
		if (ud->TxCompleteCallback)
			ud->TxCompleteCallback();
	}
}

/************************************************************************//**
 * \brief Handles interrupts for all the UARTs in the device. Must be called
 *        by the interrupt handlers of every UART.
 * \param[in] uartNum Number of the UART that generated the interrupt.
 ****************************************************************************/
void UartInterrupt(unsigned uartNum)
{
	/// Interrupt source
	unsigned long source;
	/// Pointer to the UartData to use
	UartData *ud = &uart[uartNum];

	/// Get and clear interrupt flags
    source = MAP_UARTIntStatus(ud->base, true);
    MAP_UARTIntClear(ud->base, source);

    if (source & (UART_INT_RX | UART_INT_RT))
    	UartIntRx(ud, (int)(source & UART_INT_RT));
    if (source & UART_INT_TX) UartIntTx(ud);
}

/// Handler of the UART0 interrupt
void UartInterrupt0(void)
{
	UartInterrupt(0);
}

/// Handler of the UART1 interrupt
void UartInterrupt1(void)
{
	UartInterrupt(1);
}

/// Handler of the UART2 interrupt
void UartInterrupt2(void)
{
	UartInterrupt(2);
}

/// Handler of the UART3 interrupt
void UartInterrupt3(void)
{
	UartInterrupt(3);
}

/// Handler of the UART4 interrupt
void UartInterrupt4(void)
{
	UartInterrupt(4);
}

/// Handler of the UART5 interrupt
void UartInterrupt5(void)
{
	UartInterrupt(5);
}

/// Handler of the UART6 interrupt
void UartInterrupt6(void)
{
	UartInterrupt(6);
}

/// Handler of the UART7 interrupt
void UartInterrupt7(void)
{
	UartInterrupt(7);
}

/// Allowed interrupt handlers for the UARTs
static const void *uartHandlers[UART_NUM_UARTS] =
{
	UartInterrupt0, UartInterrupt1, UartInterrupt2, UartInterrupt3,
	UartInterrupt4, UartInterrupt5, UartInterrupt6, UartInterrupt7
};

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
void UartInit(u8 uartNum, u32 baudRate)
{
	/// Pointer to the UartData to use
	UartData *ud = &uart[uartNum];

	/// Set the base address, peripheral and interrupt number used
	ud->intNum = uartInt[uartNum];
	ud->base = uartBase[uartNum];
	ud->periph = uartPeriph[uartNum];
	/// Set the "complete" callbacks to NULL
	ud->RxCompleteCallback = NULL;
	ud->TxCompleteCallback = NULL;

	/// Initialize Tx and Rx buffers
	ud->rxBuf.data = NULL;
	ud->rxBuf.maxLen = 0;
	ud->txBuf.data = NULL;
	ud->txBuf.maxLen = 0;

    /// Enable the UART peripheral
	MAP_SysCtlPeripheralEnable(uartGpioPeriph[uartNum]);
	MAP_GPIOPinTypeUART(uartGpioBase[uartNum],
		uartRxGpioPin[uartNum] | uartTxGpioPin[uartNum]);
	MAP_GPIOPinConfigure(uartRxMuxPin[uartNum]);
	MAP_GPIOPinConfigure(uartTxMuxPin[uartNum]);
    MAP_SysCtlPeripheralEnable(ud->periph);
    /// Configure baudrate, 8 bits, no parity, 1 start and 1 stop bits.
    MAP_UARTConfigSetExpClk(ud->base, MAP_SysCtlClockGet(), baudRate,
    	UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE);

    /// Set FIFO TX FIFO threshold to almost empty, and RX FIFO threshold to
    /// almost full.
    /// \note Received data must be extracted from the FIFO in two scenarios:
    /// -# When the RX FIFO threshold is reached and Receive interrupt occurs.
    /// -# When the Receive Time-Out interrupt is triggered
    UartRxFifoFlush(uartNum);
    MAP_UARTFIFOLevelSet(ud->base, UART_FIFO_TX1_8, UART_FIFO_RX7_8);

    /// Register UART interrupt
	UARTIntRegister(ud->base, uartHandlers[uartNum]);
	/// Disable all UART interrupt flags
	MAP_UARTIntDisable(ud->base, 0xFFFFFFFF);
    /// Enable UART interrupts. It will be necessary to individually enable
	/// TX/RX/RT interrupt flags.
    MAP_IntEnable(ud->intNum);

    // Start the UART
    MAP_UARTEnable(ud->base);
}

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
u8 UartSend(u8 uartNum, void *data, u32 bLen)
{
	UartData *ud = &uart[uartNum];
	UartBuf *txb = &(ud->txBuf);

	/// Check we are not already sending data
	if (ud->txBuf.maxLen) return UART_BUSY;

	/// Set up TX buffer
	ud->txBuf.data = data;
	ud->txBuf.maxLen = bLen;
	ud->txBuf.idx = 0;

	while ((txb->idx < txb->maxLen) && MAP_UARTSpaceAvail(ud->base))
		MAP_UARTCharPutNonBlocking(ud->base, txb->data[txb->idx++]);

	/// Enable TX interrupts
	MAP_UARTIntEnable(ud->base, UART_INT_TX);

	return UART_OK;
}

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
void UartRecv(u8 uartNum, void *recvBuf, u32 maxLen, u8 recvFullBuf)
{
	UartData *ud = &uart[uartNum];

	/// Make sure RX interrupts are disabled while manipulating RX buffer
	MAP_UARTIntDisable(ud->base, UART_INT_RX | UART_INT_RT);
	/// If we were already receiving, received data is discarded
	ud->rxBuf.data = recvBuf;
	ud->rxBuf.maxLen = maxLen;
	ud->rxBuf.idx = 0;

	/// Enable RX interrupt. RT interrupt is also enabled unless user
	/// requests to receive only full buffers
	MAP_UARTIntEnable(ud->base, UART_INT_RX | (recvFullBuf?0:UART_INT_RT));
}

/************************************************************************//**
 * \brief Checks if the specified UART is sending data.
 *
 * \param[in] uartNum Number of the UART to check.
 * \return UART_BUSY if the specified UART is sending data.
 *         UART_OK otherwise.
 ****************************************************************************/
u8 UartSendBusy(u8 uartNum)
{
	return uart[uartNum].txBuf.maxLen?UART_BUSY:UART_OK;
}

/************************************************************************//**
 * \brief Checks if the specified UART is receiving data.
 *
 * \param[in] uartNum Number of the UART to check.
 * \return UART_BUSY if the specified UART is receiving data.
 *         UART_OK otherwise.
 ****************************************************************************/
u8 UartRecvBusy(u8 uartNum)
{
	return uart[uartNum].rxBuf.maxLen?UART_BUSY:UART_OK;
}

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
void UartRecvCompleteCallbackSet(u8 uartNum, void (*RxCallback)(int recvd))
{
	uart[uartNum].RxCompleteCallback = RxCallback;
}

/************************************************************************//**
 * \brief Sets a callback function that will be called when a send
 *        process is finished.
 *
 * \param[in] uartNum Number of the UART to set the callback for.
 * \param[in] TxCallback Callback function.
 * \note To remove a previously set callback, call this function setting to
 *       NULL TxCallback parameter.
 ****************************************************************************/
void UartSendCompleteCallbackSet(u8 uartNum, void (*TxCallback)(void))
{
	uart[uartNum].TxCompleteCallback = TxCallback;
}

/************************************************************************//**
 * \brief Flushes the reception FIFO buffer.
 *
 * \param[in] uartNum The UART whose FIFO RX buffer will be flushed.
 ****************************************************************************/
void UartRxFifoFlush(u8 uartNum)
{
	UartData *ud = &uart[uartNum];
	while (MAP_UARTCharsAvail(ud->base)) MAP_UARTCharGetNonBlocking(ud->base);
}

/// Function for custom interrupt handling. Registers an interrupt handler.
/// \warning must be called before initialization. Use with extreme caution!
void UartCustomIntRegister(u8 uartNum, void (*IntCallback)(void))
{
    UARTIntUnregister(uartBase[uartNum]);
    UARTIntRegister(uartBase[uartNum], IntCallback);
}

/// Function for custom interrupt handling. Enables interrupts.
/// \warning Use with extreme caution!
void UartCustomIntEnable(u8 uartNum, unsigned long interrupts)
{
	MAP_UARTIntEnable(uartBase[uartNum], interrupts);
}

unsigned long UartGetBase(u8 uartNum)
{
    return uartBase[uartNum];
}

/** \} */
