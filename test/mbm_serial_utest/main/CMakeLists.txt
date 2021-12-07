idf_component_register(SRCS "master.c"
                    INCLUDE_DIRS ".")

if(CONFIG_MB_UTEST)
    set(UT_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../fs")
    if(CONFIG_MB_UTEST_INIT_FS)
        if(EXISTS ${UT_SRC_DIR})
            spiffs_create_partition_image(utfs ${UT_SRC_DIR} FLASH_IN_PROJECT)
            #create_fat_spiflash(utfs ${UT_SRC_DIR} [FLASH_IN_PROJECT])
            message(STATUS "${UT_SRC_DIR} is included into project.")
        else()
            message(FATAL_ERROR "${UT_SRC_DIR} doesn't exist.")
        endif()
    else()
        message(STATUS "${UT_SRC_DIR} not included into image.")
    endif()

endif()