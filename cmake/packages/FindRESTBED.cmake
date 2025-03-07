# FindRESTBED.cmake
# The following standard variables get defined:
#  - RESTBED_FOUND:        TRUE if Restbed is found.
#  - RESTBED_INCLUDE_DIRS: Include directories for Restbed.
#  - RESTBED_LIBRARIES:    Libraries for all Restbed component libraries and dependencies.

macro(RESTBED_REPORT_NOT_FOUND REASON_MSG)
    unset(RESTBED_FOUND)
    unset(RESTBED_INCLUDE_DIRS)
    unset(RESTBED_LIBRARIES)
    if(RESTBED_FIND_QUIETLY)
        message(STATUS "Failed to find Restbed - " ${REASON_MSG} ${ARGN})
    elseif(RESTBED_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find Restbed - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find Restbed - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(RESTBED_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling RESTBED_REPORT_NOT_FOUND
set(RESTBED_FOUND TRUE)

find_path(RESTBED_INCLUDE_DIRS NAMES web_socket.hpp
                            PATHS /usr/local/include/corvusoft/restbed
                            NO_DEFAULT_PATH)
if(NOT RESTBED_INCLUDE_DIRS OR NOT EXISTS ${RESTBED_INCLUDE_DIRS})
    RESTBED_REPORT_NOT_FOUND("Could not find Restbed includes directory")
else()
    message(STATUS "Restbed includes directory found: " ${RESTBED_INCLUDE_DIRS})
endif()

find_path(RESTBED_LIBRARIES_PATH NAMES librestbed.so
                              PATHS /usr/local/library /usr/local/lib
                              NO_DEFAULT_PATH)
if(NOT RESTBED_LIBRARIES_PATH OR NOT EXISTS ${RESTBED_LIBRARIES_PATH})
    RESTBED_REPORT_NOT_FOUND("Could not find Restbed libraries directory")
else()
    message(STATUS "Restbed libraries directory found: " ${RESTBED_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(RESTBED_FOUND)
    file(GLOB RESTBED_LIBRARIES ${RESTBED_LIBRARIES_PATH}/librestbed.so*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(RESTBED DEFAULT_MSG RESTBED_INCLUDE_DIRS RESTBED_LIBRARIES)
endif()