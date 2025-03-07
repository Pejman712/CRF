# FindKortex.cmake
# The following standard variables get defined:
#  - Kortex_FOUND:        TRUE if Kortex is found.
#  - Kortex_INCLUDE_DIRS: Include directories for Kortex.
#  - Kortex_LIBRARIES:    Libraries for all Kortex component libraries and dependencies.

macro(Kortex_REPORT_NOT_FOUND REASON_MSG)
    unset(Kortex_FOUND)
    unset(Kortex_INCLUDE_DIRS)
    unset(Kortex_LIBRARIES)
    if(Kortex_FIND_QUIETLY)
        message(STATUS "Failed to find Kortex - " ${REASON_MSG} ${ARGN})
    elseif(Kortex_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find Kortex - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find Kortex - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(Kortex_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling Kortex_REPORT_NOT_FOUND
set(Kortex_FOUND TRUE)

find_path(Kortex_INCLUDE_DIRS_PATH NAMES FrameHandler.h
                            PATHS /opt/Kortex/include
                            PATH_SUFFIXES client
                            NO_DEFAULT_PATH)
if(NOT Kortex_INCLUDE_DIRS_PATH OR NOT EXISTS ${Kortex_INCLUDE_DIRS_PATH})
    Kortex_REPORT_NOT_FOUND("Could not find Kortex includes directory")
else()
    get_filename_component(Kortex_INCLUDE_DIRS_PATH ${Kortex_INCLUDE_DIRS_PATH} DIRECTORY)
    message(STATUS "Kortex includes directory found: " ${Kortex_INCLUDE_DIRS_PATH})
    set(Kortex_INCLUDE_DIRS ${Kortex_INCLUDE_DIRS_PATH}
                            ${Kortex_INCLUDE_DIRS_PATH}/client
                            ${Kortex_INCLUDE_DIRS_PATH}/client_stubs
                            ${Kortex_INCLUDE_DIRS_PATH}/common
                            ${Kortex_INCLUDE_DIRS_PATH}/google
                            ${Kortex_INCLUDE_DIRS_PATH}/messages
    )
endif()

find_path(Kortex_LIBRARIES_PATH NAMES libKortexApiCpp.a
                              PATHS /opt/Kortex/lib/release
                              NO_DEFAULT_PATH)
if(NOT Kortex_LIBRARIES_PATH OR NOT EXISTS ${Kortex_LIBRARIES_PATH})
    Kortex_REPORT_NOT_FOUND("Could not find Kortex libraries directory")
else()
    message(STATUS "Kortex libraries directory found: " ${Kortex_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(Kortex_FOUND)
    file(GLOB Kortex_LIBRARIES ${Kortex_LIBRARIES_PATH}/libKortex*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Kortex DEFAULT_MSG Kortex_INCLUDE_DIRS Kortex_LIBRARIES)
endif()

# Needed to compile for Linux
add_definitions(-D_OS_UNIX)
