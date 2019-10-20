/*
 * Copyright 2013, 2017, Jernej Kovacic
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file
 * Implementation of functions that handle data receiving via a UART.
 *
 * @author Jernej Kovacic
 */

#include <string.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "app_config.h"
#include "bsp.h"
#include "uart.h"
#include "interrupt.h"

#include "print.h"

#include "echo.h"

/* Allocated echo buffer */
static struct echo echo;

/* UART number: */
static uint8_t recvUartNr = ( uint8_t ) -1;

/* A queue for received characters, not processed yet */
static QueueHandle_t recvQueue;


/* forward declaration of an ISR handler: */
static void recvIsrHandler(void);

/**
 * Initializes all receive related tasks and synchronization primitives.
 * This function must be called before anything is attempted to be received!
 *
 * @param uart_nr - number of the UART
 *
 * @return pdPASS if initialization is successful, pdFAIL otherwise
 */
int16_t recvInit(uint8_t uart_nr)
{
    /* Obtain the UART's IRQ from BSP */
    const uint8_t uartIrqs[BSP_NR_UARTS] = BSP_UART_IRQS;
    const uint8_t irq = ( uart_nr<BSP_NR_UARTS ?
                             uartIrqs[uart_nr] :
                             (uint8_t) -1 );
    if (echo_init(&echo, vPrintMsg) != 0)
    {
        return pdFAIL;
    }

    /* Check if UART number is valid */
    if ( uart_nr >= BSP_NR_UARTS )
    {
        return pdFAIL;
    }

    recvUartNr = uart_nr;

    /* Create and assert a queue for received characters */
    recvQueue = xQueueCreate(RECV_QUEUE_SIZE, sizeof(portCHAR));
    if ( 0 == recvQueue )
    {
        return pdFAIL;
    }

    /* Attempt to register UART's IRQ on VIC */
    if ( pic_registerIrq(irq, &recvIsrHandler, 50) < 0 )
    {
        return pdFAIL;
    }

    /* Enable the UART's IRQ on VIC */
    pic_enableInterrupt(irq);

    /* Configure the UART to receive data and trigger interrupts on receive */
    uart_enableRx(recvUartNr);
    uart_enableRxInterrupt(recvUartNr);

    return pdPASS;
}


/*
 * ISR handler, triggered by IRQ 12.
 * It reads a character from the UART and pushes it into the queue.
 */
static void recvIsrHandler(void)
{
    int ret;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Acknowledge the interrupt on the UART controller */
    uart_clearRxInterrupt(recvUartNr);

    /* Get all received characters from the UART */
    while ((ret = uart_readChar(recvUartNr)) >= 0) {
        portCHAR ch = ret;
        /*
         * Push it to the queue.
         * Note, since this is not a FreeRTOS task,
         * a *FromISR implementation of the command must be called!
         */
        xQueueSendToBackFromISR(recvQueue, (void*) &ch, &xHigherPriorityTaskWoken);
    }

    /* Now the buffer is empty we can switch context if necessary. */
    if( xHigherPriorityTaskWoken )
    {
        /* Actual macro used here is port specific. */
        // FIXME: not included in this port (yet)
        //taskYIELD_FROM_ISR ();
    }
}


/**
 * A FreeRTOS task that processes received characters.
 * The task is waiting in blocked state until the ISR handler pushes something
 * into the queue. If the received character is valid, it will be appended to a
 * string buffer. When 'Enter' is pressed, the entire string will be sent to UART0.
 *
 * @param params - ignored
 */
void recvTask(void* params)
{
    portCHAR ch;

    for ( ; ; )
    {
        /* The task is blocked until something appears in the queue */
        xQueueReceive(recvQueue, (void*) &ch, portMAX_DELAY);

        echo_putchar(&echo, ch);

    }  /* for */

    /* if it ever breaks out of the infinite loop... */
    vTaskDelete(NULL);

    /* suppress a warning since 'params' is ignored */
    (void) params;
}
