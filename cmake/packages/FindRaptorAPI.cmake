# FindRaptorAPI.cmake

# RaptorAPI_FOUND:        TRUE if ebus is found.
# RaptorAPI_INCLUDE_DIRS: Include directories for ebus.
# RaptorAPI_LIBRARIES:    Libraries for all ebus component libraries
#                    and dependencies.

macro(RaptorAPI_REPORT_NOT_FOUND REASON_MSG)
    unset(RaptorAPI_FOUND)
    unset(RaptorAPI_INCLUDE_DIRS)
    unset(RaptorAPI_LIBRARIES)
    if(RaptorAPI_FIND_QUIETLY)
        message(STATUS "Failed to find RaptorAPI - " ${REASON_MSG} ${ARGN})
    elseif(RaptorAPI_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find RaptorAPI - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find RaptorAPI - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(RaptorAPI_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling RaptorAPI_REPORT_NOT_FOUND
set(RaptorAPI_FOUND TRUE)

find_path(RaptorAPI_INCLUDE_DIRS NAMES RaptorAPI.hpp
                                 PATHS /usr/local/include/raptorapi-1.2
                                 NO_DEFAULT_PATH)                                 
if(NOT RaptorAPI_INCLUDE_DIRS OR NOT EXISTS ${RaptorAPI_INCLUDE_DIRS})
    RaptorAPI_REPORT_NOT_FOUND("Could not find RaptorAPI includes directory")
else()
    message(STATUS "RaptorAPI includes directory found: " ${RaptorAPI_INCLUDE_DIRS})
endif()

find_path(RaptorAPI_LIBRARIES_PATH NAMES libRaptorAPI.so
                                   PATHS /usr/local/lib/raptorapi-1.2
                                   NO_DEFAULT_PATH)
if(NOT RaptorAPI_LIBRARIES_PATH OR NOT EXISTS ${RaptorAPI_LIBRARIES_PATH})
    RaptorAPI_REPORT_NOT_FOUND("Could not find RaptorAPI libraries directory")
else()
    message(STATUS "RaptorAPI libraries directory found: " ${RaptorAPI_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(RaptorAPI_FOUND)
    file(GLOB RaptorAPI_LIBRARIES ${RaptorAPI_LIBRARIES_PATH}/libRaptorAPI.so
                                  ${RaptorAPI_LIBRARIES_PATH}/libSimpleChannelCIFX.so
                                  ${RaptorAPI_LIBRARIES_PATH}/libSimpleChannelUDP.so
                                  ${RaptorAPI_LIBRARIES_PATH}/libVirtuosePICV4.so
                                  ${RaptorAPI_LIBRARIES_PATH}/libAchillePICV4.so
                                  ${RaptorAPI_LIBRARIES_PATH}/libMAT6D.so
                                  ${RaptorAPI_LIBRARIES_PATH}/libAchilleARM.so)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(RaptorAPI DEFAULT_MSG RaptorAPI_INCLUDE_DIRS RaptorAPI_LIBRARIES)
endif()
