# FindKinovaApi.cmake

# KinovaApi_FOUND:        TRUE if ebus is found.
# KinovaApi_INCLUDE_DIRS: Include directories for ebus.
# KinovaApi_LIBRARIES:    Libraries for all ebus component libraries
#                    and dependencies.

macro(KinovaApi_REPORT_NOT_FOUND REASON_MSG)
    unset(KinovaApi_FOUND)
    unset(KinovaApi_INCLUDE_DIRS)
    unset(KinovaApi_LIBRARIES)
    if(KinovaApi_FIND_QUIETLY)
        message(STATUS "Failed to find KinovaApi - " ${REASON_MSG} ${ARGN})
    elseif(KinovaApi_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find KinovaApi - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find KinovaApi - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(KinovaApi_REPORT_NOT_FOUND)

# first set, later unset if not found
set(KinovaApi_FOUND TRUE)

find_path(KinovaApi_API_DIR
    NAMES KinovaTypes.h
    PATHS /opt/JACO2SDK /opt/JACO-SDK
    PATH_SUFFIXES API
    NO_DEFAULT_PATH)
if(NOT KinovaApi_API_DIR OR NOT EXISTS ${KinovaApi_API_DIR})
    KinovaApi_REPORT_NOT_FOUND(
        "Could not find KinovaApi API directory")
else()
    message(STATUS "KinovaApi dir found: " ${KinovaApi_API_DIR})
endif()

include(FindPackageHandleStandardArgs)
if(KinovaApi_FOUND)
    set(KinovaApi_INCLUDE_DIRS ${KinovaApi_API_DIR})
    file(GLOB KinovaApi_LIBRARIES ${KinovaApi_API_DIR}/Kinova.API.*.so)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(KinovaApi DEFAULT_MSG
        KinovaApi_INCLUDE_DIRS KinovaApi_LIBRARIES)
endif()
