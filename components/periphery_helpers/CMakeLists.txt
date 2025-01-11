idf_component_register(SRCS 
                    include/ph_board_led.hpp
                    include/ph_uart.hpp 
                    include/ph_uart_primitives.hpp 
                    src/board_led.cpp
                    src/uart.cpp 
                    INCLUDE_DIRS "include"
                    REQUIRES generic_lib esp_driver_uart
)

#for being able to compile with clang
target_compile_options(${COMPONENT_LIB} PUBLIC -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)

