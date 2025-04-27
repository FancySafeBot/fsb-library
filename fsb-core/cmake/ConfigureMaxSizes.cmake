
function(configure_max_sizes
    VAR_FSB_CONFIG_NAME)

    if (NOT DEFINED FSB_VERSION)
        message(ERROR "FSB_VERSION must be set\n")
    endif()
    if ((NOT DEFINED FSB_CONFIG) OR (FSB_CONFIG STREQUAL "default"))
        set(FSB_CONFIG "default")
        set(FSB_SIZE_BODIES 11)
        set(FSB_SIZE_JOINTS 10)
        set(FSB_SIZE_COORDINATES 15)
        set(FSB_SIZE_DOFS 12)
    elseif (
        (NOT DEFINED FSB_CONFIG) OR
        (NOT DEFINED FSB_SIZE_BODIES) OR
        (NOT DEFINED FSB_SIZE_JOINTS) OR
        (NOT DEFINED FSB_SIZE_COORDINATES) OR
        (NOT DEFINED FSB_SIZE_DOFS))
        message(ERROR "Not all custom configuration options were set\n"
            "  FSB_CONFIG=${FSB_CONFIG}\n"
            "  FSB_SIZE_BODIES=${FSB_SIZE_BODIES}\n"
            "  FSB_SIZE_JOINTS=${FSB_SIZE_JOINTS}\n"
            "  FSB_SIZE_COORDINATES=${FSB_SIZE_COORDINATES}\n"
            "  FSB_SIZE_DOFS=${FSB_SIZE_DOFS}\n")
    endif ()

    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fsb_configuration.h.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/fsb_configuration.h"
        @ONLY)

    # set function output
    set(${VAR_FSB_CONFIG_NAME} "${FSB_CONFIG}" PARENT_SCOPE)

endfunction()
