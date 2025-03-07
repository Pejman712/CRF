# Findlely.cmake
# The following standard variables get defined:
#  - lely_FOUND:        TRUE if lely is found.
#  - lely_INCLUDE_DIRS: Include directories for lely.
#  - lely_LIBRARIES:    Libraries for all lely component libraries and dependencies.

macro(lely_REPORT_NOT_FOUND REASON_MSG)
    unset(lely_FOUND)
    unset(lely_INCLUDE_DIRS)
    unset(lely_LIBRARIES)
    if(lely_FIND_QUIETLY)
        message(STATUS "Failed to find lely - " ${REASON_MSG} ${ARGN})
    elseif(lely_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find lely - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find lely - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(lely_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling lely_REPORT_NOT_FOUND
set(lely_FOUND TRUE)

find_path(lely_INCLUDE_DIRS NAMES master.hpp
                            PATHS /usr/include/lely/
                            PATH_SUFFIXES coapp
                            NO_DEFAULT_PATH)
if(NOT lely_INCLUDE_DIRS OR NOT EXISTS ${lely_INCLUDE_DIRS})
    lely_REPORT_NOT_FOUND("Could not find lely includes directory")
else()
    message(STATUS "lely includes directory found: " ${lely_INCLUDE_DIRS})
endif()

find_path(lely_LIBRARIES_PATH NAMES liblely-coapp.so
                              PATHS /usr/lib/x86_64-linux-gnu
                              NO_DEFAULT_PATH)
if(NOT lely_LIBRARIES_PATH OR NOT EXISTS ${lely_LIBRARIES_PATH})
    lely_REPORT_NOT_FOUND("Could not find lely libraries directory")
else()
    message(STATUS "lely libraries directory found: " ${lely_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(lely_FOUND)
    file(GLOB lely_LIBRARIES ${lely_LIBRARIES_PATH}/liblely-*.so)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(lely DEFAULT_MSG lely_INCLUDE_DIRS lely_LIBRARIES)
endif()
