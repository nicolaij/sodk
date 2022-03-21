
set(srcs "main.c" "radio.c" "network.c" "clock.c")

if(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND srcs "powerC3.c")
endif()
if(IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs "power.c")
endif()

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ".")

