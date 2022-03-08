message(STATUS "mb_ut_lib: ${CMAKE_CURRENT_LIST_DIR}, ${CONFIG_MB_UTEST}")

add_library(mb_ut_lib "${CMAKE_CURRENT_LIST_DIR}/ut_io.c"
                        "${CMAKE_CURRENT_LIST_DIR}/mbport_stubs.c"
                        "${CMAKE_CURRENT_LIST_DIR}/hw_subst.c"
                        "${CMAKE_CURRENT_LIST_DIR}/pcap.c"
                        )

idf_component_get_property(dir esp-modbus COMPONENT_DIR)
target_include_directories(mb_ut_lib PUBLIC 
                            "${CMAKE_CURRENT_LIST_DIR}"
                            "${CMAKE_CURRENT_LIST_DIR}/../include"
                            "${dir}/freemodbus/port"
                            "${dir}/freemodbus/modbus/include"
                            "${dir}/freemodbus/common/include"
                            "${dir}/freemodbus/modbus/rtu"
                            "${dir}/freemodbus/tcp_master/port"
                            "${dir}/freemodbus/tcp_slave/port"
                            )

message(STATUS "library include: ${CMAKE_CURRENT_LIST_DIR}/../include") 

idf_component_get_property(spiffs_lib spiffs COMPONENT_LIB)
message(STATUS "The spiffs lib: ${spiffs_lib}") 
target_link_libraries(mb_ut_lib PUBLIC ${spiffs_lib})

set(WRAP_FUNCTIONS
    xMBMasterPortSerialInit
    xMBMasterPortSerialInputEvent
    xMBMasterPortSerialPutByte
    xMBMasterPortSerialGetByte
    xMBMasterSerialPortGetResponse
    xMBMasterSerialPortSendRequest
    vMBMasterPortSerialClose
    xMBPortEventGet
    xMBPortSerialInit
    xMBPortSerialPutByte
    xMBPortSerialGetByte
    xMBSerialPortGetRequest
    xMBSerialPortSendResponse
    vMBPortSerialClose
    xMBMasterPortEventGet
    xMBMasterTCPPortInit
    xMBMasterTCPPortSendRequest
    xMBMasterTCPPortGetResponse
    vMBMasterTCPPortClose
    xMBTCPPortInit
    xMBTCPPortSendResponse
    xMBTCPPortGetRequest
    vMBTCPPortClose
    xMBPortSerialWaitEvent
    #vMBMasterErrorCBRespondTimeout
    #usMBMasterPortSerialRxPoll
    #eMBMasterWaitRequestFinish
    #xMBTCPPortMasterConnect
    )

if(CONFIG_MB_UTEST_OVERRIDE AND CONFIG_FMB_COMM_MODE_TCP_EN)

list(APPEND WRAP_FUNCTIONS
    esp_netif_init
    esp_netif_deinit
    esp_wifi_set_ps
    get_example_netif
    example_connect
    example_disconnect
    init_services
    )

endif()
  
    # lwip_freeaddrinfo
    # lwip_getaddrinfo
    # lwip_connect
    # lwip_recv
    # lwip_send
    # lwip_socket
    # lwip_send
    # select
    # fcntl
    # lwip_getsockopt

foreach(wrap ${WRAP_FUNCTIONS})
    target_link_libraries(mb_ut_lib PUBLIC "-Wl,--undefined=${wrap}")
    target_link_libraries(mb_ut_lib PUBLIC "-Wl,--wrap=${wrap}")
endforeach()

# allow multiple symbol definitions
target_link_libraries(mb_ut_lib PUBLIC "-Wl,--allow-multiple-definition") #--allow-multiple-definition -z,multidefs