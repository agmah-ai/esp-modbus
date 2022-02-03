# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(srcs
    "common/esp_modbus_master.c"
    "common/esp_modbus_slave.c"
    "modbus/mb.c"
    "modbus/mb_m.c"
    "modbus/ascii/mbascii.c"
    "modbus/ascii/mbascii_m.c"
    "modbus/rtu/mbrtu_m.c"
    "modbus/rtu/mbrtu.c"
    "modbus/rtu/mbcrc.c"
    "modbus/tcp/mbtcp.c"
    "modbus/tcp/mbtcp_m.c"
    "port/port.c"
    "port/portevent.c"
    "port/portevent_m.c"
    "port/portother.c"
    "port/portother_m.c"
    "port/portserial.c"
    "port/portserial_m.c"
    "port/porttimer.c"
    "port/porttimer_m.c"
    "modbus/functions/mbfunccoils.c"
    "modbus/functions/mbfunccoils_m.c"
    "modbus/functions/mbfuncdiag.c"
    "modbus/functions/mbfuncdisc.c"
    "modbus/functions/mbfuncdisc_m.c"
    "modbus/functions/mbfuncholding.c"
    "modbus/functions/mbfuncholding_m.c"
    "modbus/functions/mbfuncinput.c"
    "modbus/functions/mbfuncinput_m.c"
    "modbus/functions/mbfuncother.c"
    "modbus/functions/mbutils.c"
    "serial_slave/modbus_controller/mbc_serial_slave.c"
    "serial_master/modbus_controller/mbc_serial_master.c"
    "tcp_slave/port/port_tcp_slave.c"
    "tcp_slave/modbus_controller/mbc_tcp_slave.c"
    "tcp_master/modbus_controller/mbc_tcp_master.c"
    "tcp_master/port/port_tcp_master.c"
    "common/esp_modbus_master_tcp.c"
    "common/esp_modbus_slave_tcp.c"
    "common/esp_modbus_master_serial.c"
    "common/esp_modbus_slave_serial.c")

set(include_dirs common/include)

set(priv_include_dirs common port modbus modbus/ascii modbus/functions
                      modbus/rtu modbus/tcp modbus/include)

list(APPEND priv_include_dirs serial_slave/port serial_slave/modbus_controller
     serial_master/port serial_master/modbus_controller
     tcp_slave/port tcp_slave/modbus_controller
     tcp_master/port tcp_master/modbus_controller)

add_prefix(srcs "${CMAKE_CURRENT_LIST_DIR}/freemodbus/" ${srcs})
add_prefix(include_dirs "${CMAKE_CURRENT_LIST_DIR}/freemodbus/" ${include_dirs})
add_prefix(priv_include_dirs "${CMAKE_CURRENT_LIST_DIR}/freemodbus/" ${priv_include_dirs})

message(STATUS "DEBUG: Use esp-modbus component folder: ${CMAKE_CURRENT_LIST_DIR}.")

set(dep_inj_dir "${CMAKE_CURRENT_LIST_DIR}/freemodbus/unit_test")

if(CONFIG_MB_UTEST)
    set(dep_inj_dir "${CMAKE_CURRENT_LIST_DIR}/freemodbus/unit_test")
    list(APPEND priv_include_dirs "${dep_inj_dir}/include")
    foreach(src ${srcs})
        get_filename_component(src_wo_ext ${src} NAME_WE)
        message(STATUS "file without extention: ${src_wo_ext}")
        if(EXISTS "${dep_inj_dir}/src/${src_wo_ext}_test.c")
            message(STATUS "file without extention: ${src_wo_ext}")
            list(REMOVE_ITEM srcs ${src})
            list(APPEND srcs "${dep_inj_dir}/src/${src_wo_ext}_test.c")
            set_property(SOURCE ${src} APPEND_STRING PROPERTY COMPILE_FLAGS
                " -include ${dep_inj_dir}/include/${src_wo_ext}_test.h -include ${dep_inj_dir}/include/dep_inject.h")
            #get_property(SOURCE ${src} PROPERTY DIRECTORY)
        endif()
    endforeach()
endif()

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                    PRIV_INCLUDE_DIRS "${priv_include_dirs}"
                    REQUIRES driver lwip)

if(CONFIG_MB_UTEST)
    message(STATUS "SOURCES: ${srcs}")
    add_subdirectory(freemodbus/unit_test/mb_utest_lib)
    target_link_libraries(${COMPONENT_LIB} PUBLIC mb_ut_lib)
endif()