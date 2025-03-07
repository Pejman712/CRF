# FindSOEM.cmake
# The following standard variables get defined:
#  - SOEM_FOUND:        TRUE if SOEM is found.
#  - SOEM_INCLUDE_DIRS: Include directories for SOEM.
#  - SOEM_LIBRARIES:    Libraries for all SOEM component libraries and dependencies.

macro(SOEM_REPORT_NOT_FOUND REASON_MSG)
    unset(SOEM_FOUND)
    unset(SOEM_INCLUDE_DIRS)
    unset(SOEM_LIBRARIES)
    if(SOEM_FIND_QUIETLY)
        message(STATUS "Failed to find SOEM - " ${REASON_MSG} ${ARGN})
    elseif(SOEM_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find SOEM - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find SOEM - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(SOEM_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling SOEM_REPORT_NOT_FOUND
set(SOEM_FOUND TRUE)

find_path(SOEM_INCLUDE_DIRS NAMES ethercat.h
                            PATHS /usr/local/include/soem
                            NO_DEFAULT_PATH)
if(NOT SOEM_INCLUDE_DIRS OR NOT EXISTS ${SOEM_INCLUDE_DIRS})
    SOEM_REPORT_NOT_FOUND("Could not find SOEM includes directory")
else()
    message(STATUS "SOEM includes directory found: " ${SOEM_INCLUDE_DIRS})
endif()

find_path(SOEM_LIBRARIES_PATH NAMES libsoem.a
                              PATHS /usr/local/lib
                              NO_DEFAULT_PATH)
if(NOT SOEM_LIBRARIES_PATH OR NOT EXISTS ${SOEM_LIBRARIES_PATH})
    SOEM_REPORT_NOT_FOUND("Could not find SOEM libraries directory")
else()
    message(STATUS "SOEM libraries directory found: " ${SOEM_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(SOEM_FOUND)
    file(GLOB SOEM_LIBRARIES ${SOEM_LIBRARIES_PATH}/libsoem.*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(SOEM DEFAULT_MSG SOEM_INCLUDE_DIRS SOEM_LIBRARIES)
endif()
