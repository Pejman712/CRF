# FindZBar.cmake
# The following standard variables get defined:
#  - ZBar_FOUND:        TRUE if ZBar is found.
#  - ZBar_INCLUDE_DIRS: Include directories for ZBar.
#  - ZBar_LIBRARIES:    Libraries for all ZBar component libraries and dependencies.

macro(ZBar_REPORT_NOT_FOUND REASON_MSG)
    unset(ZBar_FOUND)
    unset(ZBar_INCLUDE_DIRS)
    unset(ZBar_LIBRARIES)
    if(ZBar_FIND_QUIETLY)
        message(STATUS "Failed to find ZBar - " ${REASON_MSG} ${ARGN})
    elseif(ZBar_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find ZBar - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find ZBar - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(ZBar_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling ZBar_REPORT_NOT_FOUND
set(ZBar_FOUND TRUE)

find_path(ZBar_INCLUDE_DIRS NAMES zbar.h
                            PATHS /usr/include
                                  /usr/local/include
                            NO_DEFAULT_PATH)
if(NOT ZBar_INCLUDE_DIRS OR NOT EXISTS ${ZBar_INCLUDE_DIRS})
    ZBar_REPORT_NOT_FOUND("Could not find ZBar includes directory")
else()
    message(STATUS "ZBar includes directory found: " ${ZBar_INCLUDE_DIRS})
endif()

find_path(ZBar_LIBRARIES_PATH NAMES libzbar.so
                              PATHS /usr/lib/x86_64-linux-gnu/
                                    /usr/local/lib
                              NO_DEFAULT_PATH)
if(NOT ZBar_LIBRARIES_PATH OR NOT EXISTS ${ZBar_LIBRARIES_PATH})
    ZBar_REPORT_NOT_FOUND("Could not find ZBar libraries directory")
else()
    message(STATUS "ZBar libraries directory found: " ${ZBar_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(ZBar_FOUND)
    set(ZBar_LIBRARIES zbar)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(ZBar DEFAULT_MSG ZBar_INCLUDE_DIRS ZBar_LIBRARIES)
endif()
