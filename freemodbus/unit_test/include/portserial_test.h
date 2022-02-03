#warning"dependency injection for port_tcp_slave is working"
#pragma once
#include "sdkconfig.h"

#if CONFIG_MB_UTEST

#include "mbport_stubs.h"
#include "port.h"

#if CONFIG_MB_UTEST_OVERRIDE

#include "hw_subst.h"

#endif

BOOL __wrap_xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);
                       
BOOL __wrap_xMBPortSerialPutByte(CHAR ucByte);

BOOL __wrap_xMBPortSerialGetByte(CHAR* pucByte);

BOOL __wrap_xMBSerialPortGetRequest(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength);

BOOL __wrap_xMBSerialPortSendResponse(UCHAR *pucMBSerialFrame, USHORT usSerialLength);

void __wrap_vMBPortSerialClose(void);


extern BOOL __real_xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);
                       
extern BOOL __real_xMBPortSerialPutByte(CHAR ucByte);

extern BOOL __real_xMBPortSerialGetByte(CHAR* pucByte);

extern BOOL __real_xMBSerialPortGetRequest(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength);

extern BOOL __real_xMBSerialPortSendResponse(UCHAR *pucMBSerialFrame, USHORT usSerialLength);

extern void __real_vMBPortSerialClose(void);

#else
#warning "dependency injection for port_tcp_slave skipped"
#endif