function(generate_documentation
        VAR_PROJECT_NAME
        VAR_PROJECT_VERSION
        VAR_PROJECT_DESCRIPTION
        VAR_INPUT_LIST)

    # check if Doxygen is installed
    find_package(Doxygen)
    if (DOXYGEN_FOUND)

        # look for dot (graphviz)
        find_program(DOT_EXECUTABLE NAMES dot)
        if (NOT DOT_EXECUTABLE STREQUAL "DOT_EXECUTABLE-NOTFOUND")
            set(DOT_FOUND TRUE)
            set(DOXYGEN_HAVE_DOT YES)
            get_filename_component(DOXYGEN_DOT_PATH ${DOT_EXECUTABLE} DIRECTORY)
        else ()
            unset(DOT_FOUND)
            set(DOXYGEN_HAVE_DOT NO)
        endif ()

        # doxygen assets
        set(DOXYGEN_CONTENT_DIR "${PROJECT_SOURCE_DIR}/docs")

        # configuration variables
        set(DOXYGEN_PROJECT_NAME "${VAR_PROJECT_NAME}")
        set(DOXYGEN_PROJECT_NUMBER "${VAR_PROJECT_VERSION}")
        set(DOXYGEN_PROJECT_BRIEF "${VAR_PROJECT_DESCRIPTION}")
        set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/doc/${VAR_PROJECT_NAME}")

        # Add relative path
        set(DOXYGEN_INPUT_LIST)
        foreach(INPUT_ITEM ${VAR_INPUT_LIST})
            file(REAL_PATH "${INPUT_ITEM}" INPUT_ITEM_FULL_PATH)
            file(RELATIVE_PATH DOXYGEN_INPUT_ITEM "${PROJECT_SOURCE_DIR}" "${INPUT_ITEM_FULL_PATH}")
            list(APPEND DOXYGEN_INPUT_LIST "${DOXYGEN_INPUT_ITEM}")
        endforeach()
        # replace list semicolons with spaces
        string(REPLACE ";"
                " "
                DOXYGEN_INPUT
                "${DOXYGEN_INPUT_LIST}")

        # configure
        set(DOXYFILE_IN ${PROJECT_SOURCE_DIR}/cmake/doxygen/Doxyfile.in)
        set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile)
        configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

        # note the option ALL which allows to build the docs together with the application
        set(DOCUMENTATION_TARGET_NAME Documentation-${VAR_PROJECT_NAME})
        add_custom_target(${DOCUMENTATION_TARGET_NAME} ALL
                COMMAND mkdir -p ${DOXYGEN_OUTPUT_DIRECTORY}
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM)
    endif ()

endfunction()
