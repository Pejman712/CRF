# FindRPLiDARSDK.cmake
# The following standard variables get defined:
#  - RPLiDARSDK_FOUND:        TRUE if RPLiDARSDK is found.
#  - RPLiDARSDK_INCLUDE_DIRS: Include directories for RPLiDARSDK.
#  - RPLiDARSDK_LIBRARIES:    Libraries for all RPLiDARSDK component libraries and dependencies.

macro(RPLiDARSDK_REPORT_NOT_FOUND REASON_MSG)
    unset(RPLiDARSDK_FOUND)
    unset(RPLiDARSDK_INCLUDE_DIRS)
    unset(RPLiDARSDK_LIBRARIES)
    if(RPLiDARSDK_FIND_QUIETLY)
        message(STATUS "Failed to find RPLiDARSDK - " ${REASON_MSG} ${ARGN})
    elseif(RPLiDARSDK_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find RPLiDARSDK - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find RPLiDARSDK - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(RPLiDARSDK_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling RPLiDARSDK_REPORT_NOT_FOUND
set(RPLiDARSDK_FOUND TRUE)

find_path(RPLiDARSDK_INCLUDE_DIRS NAMES rplidar.h
                                  PATHS /usr/local/include/rplidar_sdk
                                  NO_DEFAULT_PATH)
if(NOT RPLiDARSDK_INCLUDE_DIRS OR NOT EXISTS ${RPLiDARSDK_INCLUDE_DIRS})
    RPLiDARSDK_REPORT_NOT_FOUND("Could not find RPLiDARSDK includes directory")
else()
    message(STATUS "RPLiDARSDK includes directory found: " ${RPLiDARSDK_INCLUDE_DIRS})
endif()

find_path(RPLiDARSDK_LIBRARIES_PATH NAMES libsl_lidar_sdk.a
                                    PATHS /usr/local/lib
                                    NO_DEFAULT_PATH)
if(NOT RPLiDARSDK_LIBRARIES_PATH OR NOT EXISTS ${RPLiDARSDK_LIBRARIES_PATH})
    RPLiDARSDK_REPORT_NOT_FOUND("Could not find RPLiDARSDK libraries directory")
else()
    message(STATUS "RPLiDARSDK libraries directory found: " ${RPLiDARSDK_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(RPLiDARSDK_FOUND)
    file(GLOB RPLiDARSDK_LIBRARIES ${RPLiDARSDK_LIBRARIES_PATH}/libsl_lidar_sdk.a)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(RPLiDARSDK DEFAULT_MSG RPLiDARSDK_INCLUDE_DIRS RPLiDARSDK_LIBRARIES)
endif()
