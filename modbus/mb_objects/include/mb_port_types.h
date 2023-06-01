#pragma once

#include "driver/uart.h"
#include "mb_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _mb_comm_mode mb_mode_type_t;

struct _port_serial_opts
{
    mb_mode_type_t mode;            /*!< Modbus communication mode */
    uart_port_t port;               /*!< Modbus communication port (UART) number */
    uint8_t slave_addr;             /*!< Modbus slave address field (dummy for master) */
    uint32_t baudrate;              /*!< Modbus baudrate */
    uart_word_length_t data_bits;   /*!< Modbus number of data bits */
    uart_stop_bits_t stop_bits;     /*!< Modbus number of stop bits */
    uart_parity_t parity;           /*!< Modbus UART parity settings */
    uint32_t response_tout_ms;      /*!< Modbus slave response timeout */
};

typedef enum _mb_slave_state mb_slave_state_t;
typedef struct _slave_addr_info mb_slave_addr_info_t;
typedef struct _port_serial_opts mb_serial_opts_t;

typedef enum _addr_type_enum {
    MB_IPV4 = 0,                     /*!< TCP IPV4 addressing */
    MB_IPV6 = 1                      /*!< TCP IPV6 addressing */
} mb_addr_type_t;

struct _port_tcp_opts {
    mb_comm_mode_t mode;                    /*!< Modbus communication mode */
    uint16_t port;                          /*!< Modbus port */
    uint8_t slave_addr;                     /*!< Modbus slave address */
    mb_addr_type_t addr_type;               /*!< Modbus address type */
    void *ip_addr_table;                    /*!< Modbus address table for connection */
    void *ip_netif_ptr;                     /*!< Modbus network interface */
    uint32_t response_tout_ms;              /*!< Modbus slave response timeout */
};

typedef struct _port_tcp_opts mb_tcp_opts_t;

#ifdef __cplusplus
}
#endif