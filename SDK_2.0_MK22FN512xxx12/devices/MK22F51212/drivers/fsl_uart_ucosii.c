/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_uart_ucosii.h"
#include <ucos_ii.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void UART_RTOS_Callback(UART_Type *base, uart_handle_t *state, status_t status, void *param)
{
    uint8_t err;
    uart_rtos_handle_t *handle = (uart_rtos_handle_t *)param;

    if (status == kStatus_UART_RxIdle)
    {
        OSFlagPost(handle->rx_event, RTOS_UART_COMPLETE, OS_FLAG_SET, &err);
    }
    else if (status == kStatus_UART_TxIdle)
    {
        OSFlagPost(handle->tx_event, RTOS_UART_COMPLETE, OS_FLAG_SET, &err);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_RTOS_Init
 * Description   : Initializes the UART instance for application
 *
 *END**************************************************************************/
int UART_RTOS_Init(uart_rtos_handle_t *handle, uart_handle_t *t_handle, const struct rtos_uart_config *cfg)
{
    uart_config_t defcfg;
    uint8_t err;

    if (NULL == handle)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == t_handle)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == cfg)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == cfg->base)
    {
        return kStatus_InvalidArgument;
    }
    if (0 == cfg->srcclk)
    {
        return kStatus_InvalidArgument;
    }
    if (0 == cfg->baudrate)
    {
        return kStatus_InvalidArgument;
    }

    handle->base = cfg->base;
    handle->t_state = t_handle;

    handle->tx_sem = OSSemCreate(1);
    if (NULL == handle->tx_sem)
    {
        return kStatus_Fail;
    }
    handle->rx_sem = OSSemCreate(1);
    if (NULL == handle->rx_sem)
    {
        OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }
    handle->tx_event = OSFlagCreate(0, &err);
    if (OS_ERR_NONE != err)
    {
        OSSemDel(handle->rx_sem, OS_DEL_ALWAYS, &err);
        OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }
    handle->rx_event = OSFlagCreate(0, &err);
    if (OS_ERR_NONE != err)
    {
        OSFlagDel(handle->rx_event, OS_DEL_ALWAYS, &err);
        OSSemDel(handle->rx_sem, OS_DEL_ALWAYS, &err);
        OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }

    UART_GetDefaultConfig(&defcfg);

    defcfg.baudRate_Bps = cfg->baudrate;
    defcfg.parityMode = cfg->parity;
#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
    defcfg.stopBitCount = cfg->stopbits;
#endif

    UART_Init(handle->base, &defcfg, cfg->srcclk);
    UART_TransferCreateHandle(handle->base, handle->t_state, UART_RTOS_Callback, handle);
    UART_TransferStartRingBuffer(handle->base, handle->t_state, cfg->buffer, cfg->buffer_size);

    UART_EnableTx(handle->base, true);
    UART_EnableRx(handle->base, true);

    return 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_RTOS_Deinit
 * Description   : Deinitializes the UART instance and frees resources
 *
 *END**************************************************************************/
int UART_RTOS_Deinit(uart_rtos_handle_t *handle)
{
    uint8_t err;

    UART_Deinit(handle->base);

    OSFlagDel(handle->tx_event, OS_DEL_ALWAYS, &err);
    OSFlagDel(handle->rx_event, OS_DEL_ALWAYS, &err);

    /* Give the semaphore. This is for functional safety */
    OSSemPost(handle->tx_sem);
    OSSemPost(handle->rx_sem);

    OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
    OSSemDel(handle->rx_sem, OS_DEL_ALWAYS, &err);

    /* Invalidate the handle */
    handle->t_state = NULL;
    handle->base = NULL;

    return 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_RTOS_Send
 * Description   : Initializes the UART instance for application
 *
 *END**************************************************************************/
int UART_RTOS_Send(uart_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length)
{
    OS_FLAGS ev;
    uint8_t err;
    int retval = kStatus_Success;

    if (NULL == handle->base)
    {
        /* Invalid handle. */
        return kStatus_Fail;
    }
    if (0 == length)
    {
        return 0;
    }
    if (NULL == buffer)
    {
        return kStatus_InvalidArgument;
    }

    OSSemPend(handle->tx_sem, 0, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

    handle->tx_xfer.data = (uint8_t *)buffer;
    handle->tx_xfer.dataSize = (uint32_t)length;

    ev = OSFlagPost(handle->tx_event, RTOS_UART_COMPLETE, OS_FLAG_CLR, &err);
    assert((ev & RTOS_UART_COMPLETE) == 0);

    /* Non-blocking call */
    UART_TransferSendNonBlocking(handle->base, handle->t_state, &handle->tx_xfer);

    ev = OSFlagPend(handle->tx_event, RTOS_UART_COMPLETE, OS_FLAG_WAIT_SET_ALL, 0, &err);
    if (!(ev & RTOS_UART_COMPLETE))
    {
        retval = kStatus_Fail;
    }

    if (OS_ERR_NONE != OSSemPost(handle->tx_sem))
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }

    return retval;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_RTOS_Recv
 * Description   : Receives chars for the application
 *
 *END**************************************************************************/
int UART_RTOS_Receive(uart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received)
{
    uint8_t err;
    OS_FLAGS ev;
    size_t n = 0;
    int retval = kStatus_Success;

    if (NULL == handle->base)
    {
        /* Invalid handle. */
        return kStatus_Fail;
    }
    if (0 == length)
    {
        if (received != NULL)
        {
            *received = 0;
        }
        return 0;
    }
    if (NULL == buffer)
    {
        return kStatus_InvalidArgument;
    }

    OSSemPend(handle->rx_sem, 0, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

    handle->rx_xfer.data = buffer;
    handle->rx_xfer.dataSize = (uint32_t)length;

    ev = OSFlagPost(handle->rx_event, RTOS_UART_COMPLETE, OS_FLAG_CLR, &err);
    assert((ev & RTOS_UART_COMPLETE) == 0);

    /* Non-blocking call */
    UART_TransferReceiveNonBlocking(handle->base, handle->t_state, &handle->rx_xfer, &n);

    if (n < length)
    {
        ev = OSFlagPend(handle->rx_event, RTOS_UART_COMPLETE, OS_FLAG_WAIT_SET_ALL, 0, &err);
        if (ev & RTOS_UART_COMPLETE)
        {
            n = length;
        }
        else
        {
            retval = kStatus_Fail;
        }
    }

    if (OS_ERR_NONE != OSSemPost(handle->rx_sem))
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }

    if (received != NULL)
    {
        *received = n;
    }

    return retval;
}
