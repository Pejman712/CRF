#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################


# This function adds a new module to the CERNRoboticFramework. It creates the variables:
#    - BUILD_MODULE_NAME: It says (ON/OFF) if the module will be reviewed for compilation.
#    - DISABLE_MODULE_NAME: It says if the module wont be compiled due to a lack of dependencies.
#    - CHECKED_MODULE_NAME: It says if the module dependencies were already checked.
#    - MODULE_NAME_LOCATION: Contains the name of the module location.
# It also adds the modules to the variable MODULES_LIST and the location to LOCATIONS_LIST.
#
# crf_module(
#     MODULE module_name -> Name of the Module. Ex: TemplateModule.
#     IN module_location -> Containing folder inside module. Ex: Utility.
#     IS boolean         -> Turn ON/OFF the module. When ON the module can be disabled if any
#                           dependency is missing.
# )
function(crf_module)
    set(options)
    set(oneValueArgs MODULE IN IS)
    set(multiValueArgs)
    cmake_parse_arguments(PARSED_ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(NOT DEFINED PARSED_ARGS_MODULE)
        message(FATAL_ERROR "You must provide a module")
    endif()
    if(NOT DEFINED PARSED_ARGS_IN)
        message(FATAL_ERROR "You must provide a location for the module")
    endif()
    if(NOT DEFINED PARSED_ARGS_IS)
        message(FATAL_ERROR "You must define if you want to build the module")
    endif()
    # First clear old variables
    unset(BUILD_${PARSED_ARGS_MODULE} CACHE)
    unset(DISABLE_${PARSED_ARGS_MODULE} CACHE)
    unset(CHECKED_${PARSED_ARGS_MODULE} CACHE)
    unset(${PARSED_ARGS_MODULE}_LOCATION CACHE)
    if(PARSED_ARGS_IS)
        # First the module is enabled and then disables if required
        set(BUILD_${PARSED_ARGS_MODULE} ON CACHE BOOL "" FORCE)
        set(DISABLE_${PARSED_ARGS_MODULE} FALSE CACHE INTERNAL "")
        set(CHECKED_${PARSED_ARGS_MODULE} FALSE CACHE INTERNAL "")
    else()
        set(BUILD_${PARSED_ARGS_MODULE} OFF CACHE BOOL "" FORCE)
        set(DISABLE_${PARSED_ARGS_MODULE} TRUE CACHE INTERNAL "")
        set(CHECKED_${PARSED_ARGS_MODULE} TRUE CACHE INTERNAL "")
    endif()
    set(${PARSED_ARGS_MODULE}_LOCATION ${PARSED_ARGS_IN} CACHE INTERNAL "")
    set(MODULES_LIST ${MODULES_LIST} ${PARSED_ARGS_MODULE} CACHE INTERNAL "")
    # Check if the location was already added to the list LOCATIONS_LIST and add it if not.
    set(LOCATION_ALREADY_ADDED FALSE)
    foreach(locations ${LOCATIONS_LIST})
        if (${locations} STREQUAL ${PARSED_ARGS_IN})
            set(LOCATION_ALREADY_ADDED TRUE)
        endif()
    endforeach(locations)
    if (NOT ${LOCATION_ALREADY_ADDED})
        set(LOCATIONS_LIST ${LOCATIONS_LIST} ${PARSED_ARGS_IN} CACHE INTERNAL "")
    endif()
endfunction(crf_module)


# This function adds a new external library to the CERNRoboticFramework. It creates the variables:
#    - BUILD_LIBRARY_NAME: It says (ON/OFF) if the library will be reviewed for compilation.
#    - DISABLE_LIBRARY_NAME: It says if the library wont be compiled due to a lack of dependencies.
#    - CHECKED_LIBRARY_NAME: It says if the library dependencies were already checked.
#    - LIBRARY_NAME_LOCATION: Contains the name of the library location, always ExternalLibraries.
# It also adds the library to the variable MODULES_LIST and the location to LOCATIONS_LIST. This is
# to treat it as another library.
#
# CRF_LIBRARY(
#     LIBRARY library_name -> Name of the Library. Ex: TemplateLibrary.
#     IS boolean         -> Turn ON/OFF the library. When ON the library can be disabled if any
#                           dependency is missing.
# )
function(crf_external_library)
    set(options)
    set(oneValueArgs LIBRARY IS)
    set(multiValueArgs)
    cmake_parse_arguments(PARSED_ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(NOT DEFINED PARSED_ARGS_LIBRARY)
        message(FATAL_ERROR "You must provide a library")
    endif()
    if(NOT DEFINED PARSED_ARGS_IS)
        message(FATAL_ERROR "You must define if you want to build the library")
    endif()
    crf_module(MODULE ${PARSED_ARGS_LIBRARY} IN ExternalLibraries IS ${PARSED_ARGS_IS})
endfunction(crf_external_library)


# This function adds a new implementation to a CERNRoboticFramework module. It creates the variables:
#    - BUILD_IMPLEMENTATION_NAME: It says (ON/OFF) if the implementation will be reviewed for
#                                 compilation.
#    - DISABLE_IMPLEMENTATION_NAME: It says if the implementation wont be compiled due to a lack of
#                                   dependencies.
#    - IMPLEMENTATION_NAME_LOCATION: Contains the name of the implementation location.
# It also adds the implementation to the variable ${MODULE}_IMPLEMENTATIONS_LIST and the module to
# the variable MODULES_WITH_IMPLEMENTATIONS.
#
# crf_implementation(
#     IMPLEMENTATION implementation_name -> Name of the Implementation. Ex: UVCCamera
#     OF module_name                     -> Name of the Module. Ex: Cameras
#     IS boolean                         -> Turn ON/OFF the implementation. When ON the module can
#                                           be disabled if any dependency is missing.
# )
function(crf_implementation)
    set(options)
    set(oneValueArgs IMPLEMENTATION OF IS)
    set(multiValueArgs)
    cmake_parse_arguments(PARSED_ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(NOT DEFINED PARSED_ARGS_IMPLEMENTATION)
        message(FATAL_ERROR "You must provide a module")
    endif()
    if(NOT DEFINED PARSED_ARGS_OF)
        message(FATAL_ERROR "You must provide a location for the module")
    endif()
    if(NOT DEFINED PARSED_ARGS_IS)
        message(FATAL_ERROR "You must define if you want to build the module")
    endif()
    # First clear old variables
    unset(BUILD_${PARSED_ARGS_IMPLEMENTATION} CACHE)
    unset(DISABLE_${PARSED_ARGS_IMPLEMENTATION} CACHE)
    unset(${PARSED_ARGS_IMPLEMENTATION}_LOCATION CACHE)
    if(PARSED_ARGS_IS)
        # First the implementation is enabled and then disables if required by directly checking
        # its dependency file
        set(BUILD_${PARSED_ARGS_IMPLEMENTATION} ON CACHE BOOL "" FORCE)
        set(DISABLE_${PARSED_ARGS_IMPLEMENTATION} FALSE CACHE INTERNAL "")
        include(${CMAKE_SOURCE_DIR}/modules/${${PARSED_ARGS_OF}_LOCATION}/${PARSED_ARGS_OF}/cmake/${PARSED_ARGS_IMPLEMENTATION}Dependencies.cmake)
    else()
        set(BUILD_${PARSED_ARGS_IMPLEMENTATION} OFF CACHE BOOL "" FORCE)
        set(DISABLE_${PARSED_ARGS_IMPLEMENTATION} TRUE CACHE INTERNAL "")
    endif()
    # Check if the Module was already added to the list MODULES_WITH_IMPLEMENTATIONS and add it if not.
    set(MODULE_ALREADY_IMPLEMENTED FALSE)
    foreach(moduleImplemented ${MODULES_WITH_IMPLEMENTATIONS})
        if (${moduleImplemented} STREQUAL ${PARSED_ARGS_OF})
            set(MODULE_ALREADY_IMPLEMENTED TRUE)
        endif()
    endforeach(moduleImplemented)
    if (${MODULE_ALREADY_IMPLEMENTED})
        set(${PARSED_ARGS_OF}_IMPLEMENTATIONS_LIST ${${PARSED_ARGS_OF}_IMPLEMENTATIONS_LIST} ${PARSED_ARGS_IMPLEMENTATION} CACHE INTERNAL "")
    else()
        set(${PARSED_ARGS_OF}_IMPLEMENTATIONS_LIST ${PARSED_ARGS_IMPLEMENTATION} CACHE INTERNAL "")
        set(MODULES_WITH_IMPLEMENTATIONS ${MODULES_WITH_IMPLEMENTATIONS} ${PARSED_ARGS_OF} CACHE INTERNAL "")
    endif()
endfunction(crf_implementation)


# This function adds a new option to the CERNRoboticFramework. It creates the variable:
#    - BUILD_OPTION_NAME: It says if the option is ON or OFF.
#
# crf_option(
#     option_name   -> Name of the option. Ex: UnitTest.
#     "description" -> Small brief of the option.
#     boolean       -> Turn ON/OFF the option.
# )
macro(crf_option variable description value)
    set(__value ${value})
    set(__condition "")
    set(__varname "__value")
    foreach(arg ${ARGN})
        if(arg STREQUAL "IF" OR arg STREQUAL "if")
            set(__varname "__condition")
        else()
            list(APPEND ${__varname} ${arg})
        endif()
    endforeach()
    unset(__varname)
    if(__condition STREQUAL "")
        set(__condition 2 GREATER 1)
    endif()
    # First clear old variables
    unset(ENABLE_${variable} CACHE)
    if(${__condition})
        if(__value MATCHES ";")
            if(${__value})
                option(ENABLE_${variable} "${description}" ON)
            else()
                option(ENABLE_${variable} "${description}" OFF)
            endif()
        elseif(DEFINED ${__value})
            if(${__value})
                option(ENABLE_${variable} "${description}" ON)
            else()
                option(ENABLE_${variable} "${description}" OFF)
            endif()
        else()
            option(ENABLE_${variable} "${description}" ${__value})
        endif()
    endif()
    unset(__condition)
    unset(__value)
endmacro(crf_option)


# This function clear the following variables:
#    - *_IMPLEMENTATIONS_LIST
#    - MODULES_WITH_IMPLEMENTATIONS
#    - MODULES_LIST
#    - MISSING_LIBRARIES
function(add_modules_and_libraries_subdirectories)
    foreach(module ${MODULES_LIST})
        if(${${module}_LOCATION} STREQUAL "ExternalLibraries")
            set(MODULE_SUBDIRECTORY "${CMAKE_SOURCE_DIR}/libraries/${module}")
        else()
            set(MODULE_SUBDIRECTORY "${CMAKE_SOURCE_DIR}/modules/${${module}_LOCATION}/${module}")
        endif()
        if(${BUILD_${module}})
            add_subdirectory(${MODULE_SUBDIRECTORY})
        endif()
    endforeach(module)

endfunction(add_modules_and_libraries_subdirectories)


# This function clear the following variables:
#    - *_IMPLEMENTATIONS_LIST
#    - MODULES_WITH_IMPLEMENTATIONS
#    - MODULES_LIST
#    - MISSING_LIBRARIES
function(clean_modules_and_implementations_lists)
    foreach(moduleImplemented ${MODULES_WITH_IMPLEMENTATIONS})
        unset(${moduleImplemented}_IMPLEMENTATIONS_LIST CACHE)
    endforeach(moduleImplemented)
    unset(MODULES_WITH_IMPLEMENTATIONS CACHE)
    unset(MODULES_LIST CACHE)
    unset(MISSING_LIBRARIES CACHE)
endfunction(clean_modules_and_implementations_lists)
