idf_component_register(SRCS 
                    main.cpp 
                    device_common.hpp
                    device_config.hpp
                    device_config.cpp
                    #ZigBee main logic
                    zb/zb_main.cpp
                    zb/zb_main.hpp
                    zb/zb_binds.hpp
                    zb/zb_binds.cpp
                    zb/zb_dev_def_const.hpp
                    zb/zb_dev_def.hpp
                    zb/zb_dev_def.cpp
                    zb/zb_dev_def_attr.hpp
                    zb/zb_dev_def_attr.cpp
                    zb/zb_dev_def_cmd.hpp
                    zb/zb_dev_def_cmd.cpp
                    zb/zb_dev_def_report.cpp
                    zb/zb_external_trigger.cpp
                    zb/zb_presence_logic.cpp
                    zb/zb_reset_button.cpp
                    zb/zb_runtime_state.cpp
                    #Periphery
                    periph/ld2412.cpp 
                    periph/ld2412.hpp 
                    periph/ld2412_component.cpp
                    periph/ld2412_component.hpp
                    INCLUDE_DIRS ""
                    #REQUIRES nvs_flash
                    )

#for being able to compile with clang
target_compile_options(${COMPONENT_LIB} PUBLIC -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)

