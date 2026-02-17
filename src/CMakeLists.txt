
set(srcs "main.c" "network.c")
list(APPEND srcs "NBIoT.c" "NBIoTlib.c")
list(APPEND srcs "terminal.c")
list(APPEND srcs "powerC3.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ".")


