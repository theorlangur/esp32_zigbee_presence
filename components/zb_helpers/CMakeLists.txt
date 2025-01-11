idf_component_register(SRCS 
                    #Library stuff
                    include/zbh_bind_table.hpp
                    include/zbh_cmd_sender.hpp
                    include/zbh_handlers_cmd.hpp
                    include/zbh_types.hpp
                    include/zbh_apsde.hpp
                    include/zbh_apsde_binds.hpp
                    src/zbh_bind_table.cpp
                    src/zbh_cmd_sender.cpp
                    src/zbh_handlers_cmd.cpp
                    src/zbh_types.cpp
                    src/zbh_apsde.cpp
                    src/zbh_apsde_binds.cpp
                    INCLUDE_DIRS "include"
                    REQUIRES generic_lib
)

#for being able to compile with clang
target_compile_options(${COMPONENT_LIB} PUBLIC -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)

