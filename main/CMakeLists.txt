idf_component_register(SRCS 
                    main.cpp 
                    device_common.hpp
                    device_config.hpp
                    device_config.cpp
                    #Library stuff
                    lib/function.hpp
                    lib/array_count.hpp
                    lib/expected_results.hpp
                    lib/formatter.hpp
                    lib/misc_helpers.hpp
                    lib/object_pool.hpp
                    lib/thread.hpp
                    lib/thread_lock.hpp
                    lib/type_traits.hpp
                    lib/linked_list.hpp
                    lib/linked_list.cpp
                    #ZigBee helpers
                    zb_helpers/zbh_bind_table.hpp
                    zb_helpers/zbh_bind_table.cpp
                    zb_helpers/zbh_cmd_sender.hpp
                    zb_helpers/zbh_cmd_sender.cpp
                    zb_helpers/zbh_handlers_cmd.hpp
                    zb_helpers/zbh_handlers_cmd.cpp
                    zb_helpers/zbh_types.hpp
                    zb_helpers/zbh_types.cpp
                    zb_helpers/zbh_apsde.hpp
                    zb_helpers/zbh_apsde.cpp
                    zb_helpers/zbh_apsde_binds.hpp
                    zb_helpers/zbh_apsde_binds.cpp
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
                    periph/board_led.hpp
                    periph/board_led.cpp
                    periph/uart.cpp 
                    periph/uart.hpp 
                    periph/ld2412.cpp 
                    periph/ld2412.hpp 
                    periph/ld2412_component.cpp
                    periph/ld2412_component.hpp
                    INCLUDE_DIRS "")

#for being able to compile with clang
target_compile_options(${COMPONENT_LIB} PUBLIC -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)

