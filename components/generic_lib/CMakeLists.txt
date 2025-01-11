idf_component_register(SRCS 
                    #Library stuff
                    include/lib_function.hpp
                    include/lib_array_count.hpp
                    include/lib_expected_results.hpp
                    include/lib_formatter.hpp
                    include/lib_misc_helpers.hpp
                    include/lib_object_pool.hpp
                    include/lib_thread.hpp
                    include/lib_thread_lock.hpp
                    include/lib_type_traits.hpp
                    include/lib_linked_list.hpp
                    src/lib_linked_list.cpp
                    INCLUDE_DIRS "include")

#for being able to compile with clang
target_compile_options(${COMPONENT_LIB} PUBLIC -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)

