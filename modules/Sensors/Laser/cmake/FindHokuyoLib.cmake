# FindHokuyoLib.cmake

# HokuyoLib_FOUND:        TRUE if libary is found.
# HokuyoLib_LIBRARIES:    Libraries for all the hokuyo component libraries
#                         and dependencies.

macro(HokuyoLib_REPORT_NOT_FOUND REASON_MSG)
    unset(HokuyoLib_FOUND)
    unset(HokuyoLib_LIBRARIES)
    if(HokuyoLib_FIND_QUIETLY)
        message(STATUS "Failed to find HokuyoLib - " ${REASON_MSG} ${ARGN})
    elseif(HokuyoLib_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find HokuyoLib - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find HokuyoLib - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(HokuyoLib_REPORT_NOT_FOUND)

# first set, later unset if not found
set(HokuyoLib_FOUND TRUE)

FIND_PATH(HokuyoLib_DIR NAMES liburg_c.a
  PATHS
  /usr/local/lib
)
if(NOT HokuyoLib_DIR OR NOT EXISTS ${HokuyoLib_DIR})
    HokuyoLib_REPORT_NOT_FOUND(
        "Could not find HokuyoLib API directory -" ${HokuyoLib_DIR})
else()
    message(STATUS "HokuyoLib dir found: " ${HokuyoLib_DIR})
endif()

include(FindPackageHandleStandardArgs)
if(HokuyoLib_FOUND)
    SET(HokuyoLib_LIBRARIES ${HokuyoLib_DIR}/liburg_c.a)

    FIND_PACKAGE_HANDLE_STANDARD_ARGS(HokuyoLib DEFAULT_MSG
        HokuyoLib_LIBRARIES)
endif()
