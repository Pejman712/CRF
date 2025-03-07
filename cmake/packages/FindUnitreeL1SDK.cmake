# FindUnitreeL1SDK.cmake
# The following standard variables get defined:
#  - UnitreeL1SDK_FOUND:        TRUE if UnitreeL1SDK is found.
#  - UnitreeL1SDK_INCLUDE_DIRS: Include directories for UnitreeL1SDK.
#  - UnitreeL1SDK_LIBRARIES:    Libraries for all UnitreeL1SDK component libraries and dependencies.



macro(UnitreeL1SDK_REPORT_NOT_FOUND REASON_MSG)
    unset(UnitreeL1SDK_FOUND)
    unset(UnitreeL1SDK_INCLUDE_DIRS)
    unset(UnitreeL1SDK_LIBRARIES)
    if(UnitreeL1SDK_FIND_QUIETLY)
        message(STATUS "Failed to find UnitreeL1SDK - " ${REASON_MSG} ${ARGN})
    elseif(UnitreeL1SDK_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find UnitreeL1SDK - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find UnitreeL1SDK - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(UnitreeL1SDK_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling UnitreeL1SDK_REPORT_NOT_FOUND
set(UnitreeL1SDK_FOUND TRUE)

find_path(UnitreeL1SDK_INCLUDE_DIRS NAMES unitree_lidar_sdk.h
                                  PATHS /usr/local/include/unilidar_sdk
                                  NO_DEFAULT_PATH)
if(NOT UnitreeL1SDK_INCLUDE_DIRS OR NOT EXISTS ${UnitreeL1SDK_INCLUDE_DIRS})
    UnitreeL1SDK_REPORT_NOT_FOUND("Could not find UnitreeL1SDK includes directory")
else()
    message(STATUS "UnitreeL1SDK includes directory found: " ${UnitreeL1SDK_INCLUDE_DIRS})
endif()

find_path(UnitreeL1SDK_LIBRARIES_PATH NAMES libunitree_lidar_sdk.a
                                    PATHS /usr/local/lib
                                    NO_DEFAULT_PATH)
if(NOT UnitreeL1SDK_LIBRARIES_PATH OR NOT EXISTS ${UnitreeL1SDK_LIBRARIES_PATH})
    UnitreeL1SDK_REPORT_NOT_FOUND("Could not find UnitreeL1SDK libraries directory")
else()
    message(STATUS "UnitreeL1SDK libraries directory found: " ${UnitreeL1SDK_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(UnitreeL1SDK_FOUND)
    file(GLOB UnitreeL1SDK_LIBRARIES ${UnitreeL1SDK_LIBRARIES_PATH}/libunitree_lidar_sdk.a)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(UnitreeL1SDK DEFAULT_MSG UnitreeL1SDK_INCLUDE_DIRS UnitreeL1SDK_LIBRARIES)
endif()
