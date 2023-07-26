/*
 * SPDX-FileCopyrightText: 2013 Armink
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SPDX-FileContributor: 2016-2021 Espressif Systems (Shanghai) CO LTD
 */
/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbfuncholding_m.c,v 1.60 2013/09/02 14:13:40 Armink Add Master Functions  Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
#include "mb_m.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
static const char* TAG = "MODBUS_FUNCBLOCK";

#define MB_PDU_REQ_WRITE_BLOCK_ADDR_OFF           ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_REQ_WRITE_BLOCK_BYTECNT_OFF        ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_REQ_WRITE_BLOCK_DATA_OFF           ( MB_PDU_DATA_OFF + 6 )
#define MB_PDU_FUNC_WRITE_BLOCK_BYTECNT_OFF        ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_WRITE_BLOCK_SIZE              ( 2 )


/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/
#if MB_MASTER_RTU_ENABLED || MB_MASTER_ASCII_ENABLED || MB_MASTER_TCP_ENABLED


/**
 * This function will write a data block intended for a flash memory
 *
 * @param ucSndAddr salve address
 * @param usRegAddr register start address
 * @param usNRegs register total number
 * @param pusDataBuffer data to be written
 * @param lTimeOut timeout (-1 will waiting forever)
 *
 * @return error code
 */
eMBMasterReqErrCode
eMBMasterReqWriteBlock( UCHAR ucSndAddr,
        uint32_t flash_addr, USHORT * data_ptr, uint16_t data_size, LONG lTimeOut )
{
    UCHAR *ucMBFrame;
    //USHORT usRegIndex = 0;
    eMBMasterReqErrCode eErrStatus = MB_MRE_NO_ERR;

    if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
    else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
    else
    {
        vMBMasterGetPDUSndBuf(&ucMBFrame);
        vMBMasterSetDestAddress(ucSndAddr);
        ucMBFrame[MB_PDU_FUNC_OFF]                     = MB_FUNC_WRITE_BLOCK;
        ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_ADDR_OFF]       = flash_addr >> 24;
        ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_ADDR_OFF + 1]   = flash_addr >> 16;
        ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_ADDR_OFF + 2]   = flash_addr >> 8;
        ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_ADDR_OFF + 3]   = flash_addr;
        ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_BYTECNT_OFF]    = data_size >> 8;
        ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_BYTECNT_OFF + 1]= data_size;
        ucMBFrame += MB_PDU_REQ_WRITE_BLOCK_DATA_OFF;
        memcpy(ucMBFrame, data_ptr, data_size);
        vMBMasterSetPDUSndLength( MB_PDU_REQ_WRITE_BLOCK_DATA_OFF + data_size);
        ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_TRANSMIT );
        eErrStatus = eMBMasterWaitRequestFinish( );
    }
    return eErrStatus;

}


eMBException
eMBMasterFuncWriteBlock( UCHAR * pucFrame, USHORT * usLen )
{
    UCHAR *ucMBFrame;
    eMBException eStatus = MB_EX_NONE;
    USHORT  usReqByteCount, usWrittenBytes;

    if( *usLen == ( MB_PDU_SIZE_MIN + MB_PDU_FUNC_WRITE_BLOCK_SIZE ) )
    {
        vMBMasterGetPDUSndBuf(&ucMBFrame);

        usReqByteCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_BYTECNT_OFF] << 8 );
        usReqByteCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_WRITE_BLOCK_BYTECNT_OFF + 1] );
        usWrittenBytes = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_BLOCK_BYTECNT_OFF] << 8 );
        usWrittenBytes |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_BLOCK_BYTECNT_OFF+1]);
        ESP_LOGI(TAG, "Written bytes (l=%d):", usWrittenBytes);
        if (usReqByteCount != usWrittenBytes) {
            ESP_LOGE(TAG, "Length mismatch (req=%d != wr=%d):", usReqByteCount, usWrittenBytes);
            // bytes written don't match request
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }

        //Check if the written bytes match the requested bytes
    } else {
        ESP_LOGE(TAG, "Length error (l=%d):", *usLen);
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif // #if MB_MASTER_RTU_ENABLED || MB_MASTER_ASCII_ENABLED || MB_MASTER_TCP_ENABLED
