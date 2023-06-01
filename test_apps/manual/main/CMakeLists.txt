set(srcs "test_common.c" "test_multi_master_serial.c" "test_multi_master_tcp.c" 
            "test_master_slave.c" "test_app_main.c")

# In order for the cases defined by `TEST_CASE` to be linked into the final elf,
idf_component_register(SRCS ${srcs} WHOLE_ARCHIVE)

