#warning "dependency injection for port_tcp_master is working"
#pragma once
#include "sdkconfig.h"


#if CONFIG_MB_UTEST

#include "mb_m.h"
#include "mbport_stubs.h"

#if CONFIG_MB_UTEST_OVERRIDE

#include "hw_subst.h"

#define uart_wait_tx_done __wrap_uart_wait_tx_done
#define uart_read_bytes __wrap_uart_read_bytes

#endif

extern BOOL __real_xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);

extern BOOL __real_xMBMasterPortSerialPutByte(CHAR ucByte);

extern BOOL __real_xMBMasterPortSerialGetByte(CHAR* pucByte);

extern BOOL __real_xMBMasterSerialPortGetResponse(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength);

extern BOOL __real_xMBMasterSerialPortSendRequest(UCHAR *pucMBSerialFrame, USHORT usSerialLength);

extern void __real_vMBMasterPortSerialClose(void);

extern BOOL __real_xMBMasterPortSerialWaitEvent(uart_event_t* pxEvent, ULONG xTimeout);

BOOL __wrap_xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);

BOOL __wrap_xMBMasterPortSerialPutByte(CHAR ucByte);

BOOL __wrap_xMBMasterPortSerialGetByte(CHAR* pucByte);

BOOL __wrap_xMBMasterSerialPortGetResponse(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength);

BOOL __wrap_xMBMasterSerialPortSendRequest(UCHAR *pucMBSerialFrame, USHORT usSerialLength);

void __wrap_vMBMasterPortSerialClose(void);

//BOOL __wrap_xMBMasterPortSerialWaitEvent(uart_event_t* pxEvent, ULONG xTimeout);

// eMBMasterReqErrCode __wrap_eMBMasterWaitRequestFinish( void );

#endif