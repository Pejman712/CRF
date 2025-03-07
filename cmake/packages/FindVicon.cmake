# FindVicon.cmake
# The following standard variables get defined:
#  - Vicon_FOUND:        TRUE if Vicon is found.
#  - Vicon_INCLUDE_DIRS: Include directories for Vicon.
#  - Vicon_LIBRARIES:    Libraries for all Vicon component libraries and dependencies.

macro(Vicon_REPORT_NOT_FOUND REASON_MSG)
    unset(Vicon_FOUND)
    unset(Vicon_INCLUDE_DIRS)
    unset(Vicon_LIBRARIES)
    if(Vicon_FIND_QUIETLY)
        message(STATUS "Failed to find Vicon - " ${REASON_MSG} ${ARGN})
    elseif(Vicon_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find Vicon - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find Vicon - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(Vicon_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling Vicon_REPORT_NOT_FOUND
set(Vicon_FOUND TRUE)

find_path(Vicon_INCLUDE_DIRS NAMES vicon/DataStreamClient.h
                            PATHS /usr/local/include/vicondatastreamsdk-1.12/
                            NO_DEFAULT_PATH)
if(NOT Vicon_INCLUDE_DIRS OR NOT EXISTS ${Vicon_INCLUDE_DIRS})
    Vicon_REPORT_NOT_FOUND("Could not find Vicon includes directory")
else()
    message(STATUS "Vicon includes directory found: " ${Vicon_INCLUDE_DIRS})
endif()

find_path(Vicon_LIBRARIES_PATH NAMES libViconDataStreamSDK_CPP.so
                              PATHS /usr/local/lib/vicondatastreamsdk-1.12
                              NO_DEFAULT_PATH)
if(NOT Vicon_LIBRARIES_PATH OR NOT EXISTS ${Vicon_LIBRARIES_PATH})
    Vicon_REPORT_NOT_FOUND("Could not find Vicon libraries directory")
else()
    message(STATUS "Vicon libraries directory found: " ${Vicon_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(Vicon_FOUND)
    file(GLOB Vicon_LIBRARIES ${Vicon_LIBRARIES_PATH}/*.so*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Vicon DEFAULT_MSG Vicon_INCLUDE_DIRS Vicon_LIBRARIES)
endif()
