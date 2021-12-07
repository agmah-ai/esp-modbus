message(STATUS "mb_ut_lib: ${CMAKE_CURRENT_LIST_DIR}, ${CONFIG_MB_UTEST}")

add_library(mb_ut_lib "${CMAKE_CURRENT_LIST_DIR}/ut_io.c"
                        "${CMAKE_CURRENT_LIST_DIR}/mbport_stubs.c"
                        "${CMAKE_CURRENT_LIST_DIR}/hw_subst.c"
                        "${CMAKE_CURRENT_LIST_DIR}/pcap.c")

idf_component_get_property(dir esp-modbus COMPONENT_DIR)
target_include_directories(mb_ut_lib PUBLIC 
                            "${CMAKE_CURRENT_LIST_DIR}"
                            "${dir}/freemodbus/port"
                            "${dir}/freemodbus/modbus/include"
                            "${dir}/freemodbus/common/include"
                            "${dir}/freemodbus/modbus/rtu"
                            "${dir}/freemodbus/tcp_master/port"
                            )

idf_component_get_property(spiffs_lib spiffs COMPONENT_LIB)
message(STATUS "The spiffs lib: ${spiffs_lib}") 
target_link_libraries(mb_ut_lib PUBLIC ${spiffs_lib})

set(WRAP_FUNCTIONS
    xMBMasterPortSerialInit
    xMBMasterPortSerialPutByte
    xMBMasterPortSerialGetByte
    xMBMasterSerialPortGetResponse
    xMBMasterSerialPortSendRequest
    vMBMasterPortSerialClose
    xMBPortSerialInit
    xMBPortSerialPutByte
    xMBPortSerialGetByte
    xMBSerialPortGetRequest
    xMBSerialPortSendResponse
    vMBPortSerialClose
    xMBMasterTCPPortInit
    xMBMasterTCPPortSendRequest
    xMBMasterTCPPortGetResponse
    vMBMasterTCPPortClose
    xMBTCPPortInit
    xMBTCPPortSendResponse
    xMBTCPPortGetRequest
    vMBTCPPortClose
    xMBMasterPortEventGet
    vMBMasterErrorCBRespondTimeout
    usMBMasterPortSerialRxPoll
    eMBMasterWaitRequestFinish
    xMBTCPPortMasterConnect)

foreach(wrap ${WRAP_FUNCTIONS})
    target_link_libraries(mb_ut_lib PUBLIC "-Wl,--wrap=${wrap}")
endforeach()

# allow multiple symbol definitions
target_link_libraries(mb_ut_lib PUBLIC "-Wl,-zmuldefs" "-no-pie")