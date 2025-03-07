# Findurdfdom.cmake
# The following standard variables get defined:
#  - urdfdom_FOUND:        TRUE if urdfdom is found.
#  - urdfdom_INCLUDE_DIRS: Include directories for urdfdom.
#  - urdfdom_LIBRARIES:    Libraries for all urdfdom component libraries and dependencies.

macro(urdfdom_REPORT_NOT_FOUND REASON_MSG)
    unset(urdfdom_FOUND)
    unset(urdfdom_INCLUDE_DIRS)
    unset(urdfdom_LIBRARIES)
    if(urdfdom_FIND_QUIETLY)
        message(STATUS "Failed to find urdfdom - " ${REASON_MSG} ${ARGN})
    elseif(urdfdom_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find urdfdom - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find urdfdom - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(urdfdom_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling urdfdom_REPORT_NOT_FOUND
set(urdfdom_FOUND TRUE)

find_path(urdfdom_INCLUDE_DIRS NAMES model.h
                               PATHS /usr/include
                               PATH_SUFFIXES urdf_model
                               NO_DEFAULT_PATH)
if(NOT urdfdom_INCLUDE_DIRS OR NOT EXISTS ${urdfdom_INCLUDE_DIRS})
    urdfdom_REPORT_NOT_FOUND("Could not find urdfdom includes directory")
else()
    message(STATUS "urdfdom includes directory found: " ${urdfdom_INCLUDE_DIRS})
endif()

find_path(urdfdom_LIBRARIES_PATH NAMES liburdfdom_model.so
                              PATHS /usr/lib/x86_64-linux-gnu
                                    /usr/local/lib
                              NO_DEFAULT_PATH)
if(NOT urdfdom_LIBRARIES_PATH OR NOT EXISTS ${urdfdom_LIBRARIES_PATH})
    urdfdom_REPORT_NOT_FOUND("Could not find urdfdom libraries directory")
else()
    message(STATUS "urdfdom libraries directory found: " ${urdfdom_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(urdfdom_FOUND)
    file(GLOB urdfdom_LIBRARIES ${urdfdom_LIBRARIES_PATH}/liburdfdom_model.so*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(urdfdom DEFAULT_MSG urdfdom_INCLUDE_DIRS urdfdom_LIBRARIES)
endif()
