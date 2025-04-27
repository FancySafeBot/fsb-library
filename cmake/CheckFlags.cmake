
function(get_supported_flags
    INPUT_COMPILE_FLAGS
    INPUT_LINK_FLAGS
    INPUT_SANITIZER_FLAGS
    VAR_C_FLAGS
    VAR_CXX_FLAGS
    VAR_LINK_FLAGS)

    # sanitizers
    if (INPUT_SANITIZER_FLAGS)
        check_sanitizers(SANITIZER_FLAGS ${INPUT_SANITIZER_FLAGS})
        list(APPEND INPUT_COMPILE_FLAGS
            ${SANITIZER_FLAGS})
        list(APPEND INPUT_LINK_FLAGS
            ${SANITIZER_FLAGS})
    endif ()

    # keep only supported flags
    check_c_compile_flags(SUPPORTED_C_COMPILE_FLAGS "${INPUT_COMPILE_FLAGS}")
    check_cxx_compile_flags(SUPPORTED_CXX_COMPILE_FLAGS "${INPUT_COMPILE_FLAGS}")
    check_link_flags(SUPPORTED_LINK_FLAGS "${INPUT_LINK_FLAGS}")

    # set function output
    set(${VAR_C_FLAGS} "${SUPPORTED_C_COMPILE_FLAGS}" PARENT_SCOPE)
    set(${VAR_CXX_FLAGS} "${SUPPORTED_CXX_COMPILE_FLAGS}" PARENT_SCOPE)
    set(${VAR_LINK_FLAGS} "${SUPPORTED_LINK_FLAGS}" PARENT_SCOPE)

endfunction()

function(check_c_compile_flags VAR_SUPPORTED_C_COMPILE_FLAGS COMPILE_FLAGS_LIST)
    # check flags
    set(SUPPORTED_C_COMPILE_FLAGS)
    set(BACKUP_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
    include(CheckCCompilerFlag)

    # apply custom compiler flags
    foreach (compiler_flag
        IN
        LISTS
        COMPILE_FLAGS_LIST)
        # remove problematic characters
        string(REGEX
            REPLACE "[^a-zA-Z0-9]"
            ""
            var_suffix
            ${compiler_flag})
        set(CMAKE_REQUIRED_FLAGS "-Werror ${compiler_flag}")
        check_c_compiler_flag(${compiler_flag} FLAG_C_SUPPORTED_${var_suffix})

        if (FLAG_C_SUPPORTED_${var_suffix})
            list(APPEND SUPPORTED_C_COMPILE_FLAGS ${compiler_flag})
        endif ()
    endforeach ()

    # Done testing flags
    set(CMAKE_REQUIRED_FLAGS ${BACKUP_REQUIRED_FLAGS})

    # Set variable in parent scope
    set(${VAR_SUPPORTED_C_COMPILE_FLAGS} "${SUPPORTED_C_COMPILE_FLAGS}" PARENT_SCOPE)
endfunction()

function(check_cxx_compile_flags VAR_SUPPORTED_CXX_COMPILE_FLAGS COMPILE_FLAGS_LIST)
    # check flags
    set(SUPPORTED_CXX_COMPILE_FLAGS)
    set(BACKUP_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
    include(CheckCXXCompilerFlag)

    # apply custom compiler flags
    foreach (compiler_flag
        IN
        LISTS
        COMPILE_FLAGS_LIST)
        # remove problematic characters
        string(REGEX
            REPLACE "[^a-zA-Z0-9]"
            ""
            var_suffix
            ${compiler_flag})
        set(CMAKE_REQUIRED_FLAGS "-Werror ${compiler_flag}")
        check_cxx_compiler_flag(${compiler_flag} FLAG_CXX_SUPPORTED_${var_suffix})

        if (FLAG_CXX_SUPPORTED_${var_suffix})
            list(APPEND SUPPORTED_CXX_COMPILE_FLAGS ${compiler_flag})
        endif ()
    endforeach ()

    # Done testing flags
    set(CMAKE_REQUIRED_FLAGS ${BACKUP_REQUIRED_FLAGS})

    # set variable in parent scope
    set(${VAR_SUPPORTED_CXX_COMPILE_FLAGS} "${SUPPORTED_CXX_COMPILE_FLAGS}" PARENT_SCOPE)
endfunction()

function(check_sanitizers VAR_SUPPORTED_SANITIZER_FLAGS SANITIZERS_LIST)
    # check flags
    set(SUPPORTED_SANITIZER_FLAGS)

    set(BACKUP_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    set(BACKUP_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
    set(COMPILE_C_PROGRAM_SRC "int main(void){ return 0; }")
    include(CheckCSourceCompiles)
    include(CheckCCompilerFlag)

    # apply custom compile and link flags
    foreach (sanitizer ${SANITIZERS_LIST})
        # remove problematic characters
        set(sanitize_flag "-fsanitize=${sanitizer}")
        string(REGEX
            REPLACE "[^a-zA-Z0-9]"
            ""
            sanitizer_suffix
            ${sanitize_flag})
        set(CMAKE_REQUIRED_LIBRARIES "-Werror ${sanitize_flag}")
        set(CMAKE_REQUIRED_FLAGS "-Werror ${sanitize_flag}")
        check_c_source_compiles("${COMPILE_C_PROGRAM_SRC}" FLAG_LINK_SUPPORTED_${sanitizer_suffix})

        if (FLAG_LINK_SUPPORTED_${sanitizer_suffix})
            check_c_compiler_flag(${sanitize_flag} FLAG_C_SUPPORTED_${sanitizer_suffix})

            if (FLAG_C_SUPPORTED_${sanitizer_suffix})
                list(APPEND SUPPORTED_SANITIZER_FLAGS ${sanitize_flag})
            endif ()
        endif ()
    endforeach ()

    # Done testing flags
    set(CMAKE_REQUIRED_FLAGS ${BACKUP_REQUIRED_FLAGS})
    set(CMAKE_REQUIRED_LIBRARIES ${BACKUP_REQUIRED_LIBRARIES})

    # set variable in parent scope
    set(${VAR_SUPPORTED_SANITIZER_FLAGS} "${SUPPORTED_SANITIZER_FLAGS}" PARENT_SCOPE)
endfunction()

function(check_link_flags VAR_SUPPORTED_LINK_FLAGS LINK_FLAGS_LIST)
    # check flags
    set(SUPPORTED_LINK_FLAGS)

    set(BACKUP_REQUIRED_LINK_OPTIONS ${CMAKE_REQUIRED_LINK_OPTIONS})
    set(COMPILE_C_PROGRAM_SRC "int main(void){ return 0; }")
    include(CheckCSourceCompiles)

    # apply custom link flags
    foreach (link_flag
        IN
        LISTS
        LINK_FLAGS_LIST)
        # remove problematic characters
        string(REGEX
            REPLACE "[^a-zA-Z0-9]"
            ""
            var_suffix
            ${link_flag})
        set(CMAKE_REQUIRED_LINK_OPTIONS "-Werror ${link_flag}")
        check_c_source_compiles("${COMPILE_C_PROGRAM_SRC}" FLAG_LINK_SUPPORTED_${var_suffix})

        if (FLAG_LINK_SUPPORTED_${var_suffix})
            list(APPEND SUPPORTED_LINK_FLAGS ${link_flag})
        endif ()
    endforeach ()

    # Done testing flags, reset backup
    set(CMAKE_REQUIRED_LINK_OPTIONS ${BACKUP_REQUIRED_LINK_OPTIONS})

    # set variable in parent scope
    set(${VAR_SUPPORTED_LINK_FLAGS} "${SUPPORTED_LINK_FLAGS}" PARENT_SCOPE)
endfunction()
