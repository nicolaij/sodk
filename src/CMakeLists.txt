
set(srcs "main.c" "network.c")

if(SDKCONFIG MATCHES "NB-IoT")
    list(APPEND srcs "NBIoT.c")
else()
    list(APPEND srcs "radio.c")
endif()

if(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND srcs "powerC3.c")
endif()

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ".")

