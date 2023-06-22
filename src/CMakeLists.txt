
set(srcs "main.c" "network.c")

if(SDKCONFIG MATCHES "LORA")
    list(APPEND srcs "radio.c")
else()
    list(APPEND srcs "NBIoT.c")
endif()

if(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND srcs "powerC3.c")
endif()

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ".")

