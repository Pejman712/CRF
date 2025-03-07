# FindDynamixelSDK.cmake
# The following standard variables get defined:
#  - DynamixelSDK_FOUND:        TRUE if DynamixelSDK is found.
#  - DynamixelSDK_INCLUDE_DIRS: Include directories for DynamixelSDK.
#  - DynamixelSDK_LIBRARIES:    Libraries for all DynamixelSDK component libraries and dependencies.

macro(DynamixelSDK_REPORT_NOT_FOUND REASON_MSG)
    unset(DynamixelSDK_FOUND)
    unset(DynamixelSDK_INCLUDE_DIRS)
    unset(DynamixelSDK_LIBRARIES)
    if(DynamixelSDK_FIND_QUIETLY)
        message(STATUS "Failed to find DynamixelSDK - " ${REASON_MSG} ${ARGN})
    elseif(DynamixelSDK_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find DynamixelSDK - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
    message("-- Failed to find DynamixelSDK - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(DynamixelSDK_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling DynamixelSDK_REPORT_NOT_FOUND
set(DynamixelSDK_FOUND TRUE)

find_path(DynamixelSDK_INCLUDE_DIRS NAMES dynamixel_sdk.h
    PATHS /usr/local/include/dynamixel_sdk
    NO_DEFAULT_PATH)

if(NOT DynamixelSDK_INCLUDE_DIRS OR NOT EXISTS ${DynamixelSDK_INCLUDE_DIRS})
    DynamixelSDK_REPORT_NOT_FOUND("Could not find DynamixelSDK includes directory")
else()
    message(STATUS "DynamixelSDK includes directory found: " ${DynamixelSDK_INCLUDE_DIRS})
endif()

find_path(DynamixelSDK_LIBRARIES_PATH NAMES libdxl_x64_cpp.so 
    PATHS /usr/local/lib
    NO_DEFAULT_PATH)
if(NOT DynamixelSDK_LIBRARIES_PATH OR NOT EXISTS ${DynamixelSDK_LIBRARIES_PATH})
    DynamixelSDK_REPORT_NOT_FOUND("Could not find DynamixelSDK libraries directory")
else()
    message(STATUS "DynamixelSDK libraries directory found: " ${DynamixelSDK_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(DynamixelSDK_FOUND)
    file(GLOB DynamixelSDK_LIBRARIES ${DynamixelSDK_LIBRARIES_PATH}/libdxl_x64_cpp.*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(DynamixelSDK DEFAULT_MSG DynamixelSDK_INCLUDE_DIRS DynamixelSDK_LIBRARIES)
endif()

