/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include <sdkconfig.h>
#include "esp_log.h"
#include "port.h"
#include "mbport.h"
#include "mbcrc.h"
#include "mb_m.h"
#include "esp_modbus_master.h"
#include "port_tcp_master.h"    // required for modbus tcp port types (the header exposed by cmake file)
#include "lwip/err.h"           // needs just for exposing of the TCP stack port functions

#if CONFIG_MB_UTEST

#if CONFIG_MB_UTEST_DEBUG
#define TEST_LOG(tag, str, ...) do { \
    ESP_EARLY_LOGI(tag, str, __VA_ARGS__); \
} while(0)
#else
    #define TEST_LOG(tag, str, ...) void
#endif

void mb_master_serial_poll_cb(const UCHAR* pucPDUData, USHORT ucPDULength);

extern USHORT __real_usMBMasterPortSerialRxPoll(size_t xEventSize);

extern void __real_vMBMasterErrorCBRespondTimeout(UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength );

extern BOOL __real_xMBMasterTCPPortInit(USHORT usTCPPort);

extern BOOL __real_xMBMasterTCPPortSendRequest(UCHAR * pucMBTCPFrame, USHORT usTCPLength);

extern BOOL __real_xMBMasterTCPPortGetResponse(UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength) ;

extern void __real_vMBMasterTCPPortClose(void);

extern BOOL __real_xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);
                       
extern BOOL __real_xMBPortSerialPutByte(CHAR ucByte);

extern BOOL __real_xMBPortSerialGetByte(CHAR* pucByte);

extern BOOL __real_xMBSerialPortGetRequest(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength);

extern BOOL __real_xMBSerialPortSendResponse(UCHAR *pucMBSerialFrame, USHORT usSerialLength);

extern void __real_vMBPortSerialClose(void);

extern BOOL __real_xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);

extern BOOL __real_xMBMasterPortSerialPutByte(CHAR ucByte);

extern BOOL __real_xMBMasterPortSerialGetByte(CHAR* pucByte);

extern BOOL __real_xMBMasterSerialPortGetResponse(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength);

extern BOOL __real_xMBMasterSerialPortSendRequest(UCHAR *pucMBSerialFrame, USHORT usSerialLength);

extern void __real_vMBMasterPortSerialClose(void);

extern BOOL __real_xMBTCPPortInit(USHORT usTCPPort);

extern BOOL __real_xMBTCPPortSendResponse(UCHAR * pucMBTCPFrame, USHORT usTCPLength);

extern BOOL __real_xMBTCPPortGetRequest(UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength);

extern void __real_vMBTCPPortClose(void);

extern BOOL __real_xMBMasterPortEventGet(eMBMasterEventType* eEvent);

extern eMBMasterReqErrCode __real_eMBMasterWaitRequestFinish( void );

extern err_t __real_xMBTCPPortMasterConnect(MbSlaveInfo_t* pxInfo);

//extern esp_err_t __real_mbc_master_send_request(mb_param_request_t* request, void* data_ptr);

//extern eMBMasterReqErrCode __real_eMBMasterReqReadHoldingRegister(UCHAR ucSndAddr, USHORT usRegAddr, USHORT usNRegs, LONG lTimeOut);

#endif
