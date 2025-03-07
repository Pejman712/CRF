#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ColorsDefinition.cmake)


# This function checks the dependencies of all the modules listed.
function(check_dependencies)
    foreach(module ${MODULES_LIST})
        check_module_dependencies(${module})
    endforeach(module)
endfunction(check_dependencies)


# This function checks the dependencies of a specific module using it's
# DependenciesAndmplementations.cmake file, in which all internal and external dependencies are
# listed.
#
# check_module_dependencies(
#     module_name -> Name of the Module. Ex: TemplateModule.
# )
function(check_module_dependencies variable)
    set(MODULE_EXISTS FALSE)
    foreach(module ${MODULES_LIST})
        if(${module} STREQUAL ${variable})
            set(MODULE_EXISTS TRUE)
        endif()
    endforeach(module)
    if (NOT MODULE_EXISTS)
        set(BUILD_${variable} OFF CACHE INTERNAL "")
        set(DISABLE_${variable} TRUE CACHE INTERNAL "")
        set(CHECKED_${variable} TRUE CACHE INTERNAL "")
        return()
    endif()
    if(NOT BUILD_${variable} OR CHECKED_${variable})
        return()
    endif()
    if (${${variable}_LOCATION} STREQUAL "ExternalLibraries")
        set(DEPENDENCY_FILE "${CMAKE_SOURCE_DIR}/libraries/${variable}/cmake/DependenciesAndmplementations.cmake")
    else()
        set(DEPENDENCY_FILE "${CMAKE_SOURCE_DIR}/modules/${${variable}_LOCATION}/${variable}/cmake/DependenciesAndmplementations.cmake")
    endif()
    if(NOT EXISTS ${DEPENDENCY_FILE})
        message(FATAL_ERROR "The dependency file of the ${variable} module can not be found")
    endif()
    include(${DEPENDENCY_FILE})
endfunction(check_module_dependencies)


# This function checks for a requestor module if other given modules are going to be compiled. If
# any of these is OFF or DISABLED the requestor module is DISABLED, since all these modules are
# considered its dependencies. It modifies this variable:
#    - DISABLE_REQUESTOR_MODULE_NAME: Is set to ON if any of the dependency modules is missing.
# It creates the variable:
#    - REQUESTOR_MODULE_NAME_MISSING_MODULES: List of all the needed modules that are DISABLED.
#
# check_module_status(
#     REQUESTER module_name -> Name of the Module. Ex: TemplateModule.
#     CHECK module_names -> List of all the module names that are a dependency. Ex: Utility.
# )
function(check_module_status)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs REQUESTER CHECK)
    cmake_parse_arguments(PARSED_ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(NOT DEFINED PARSED_ARGS_REQUESTER)
        message(FATAL_ERROR "You must provide a requester module")
    endif()
    if(NOT DEFINED PARSED_ARGS_CHECK)
        message(FATAL_ERROR "You must provide a set of modules to be checked")
    endif()
    # First clear old variables
    unset(${PARSED_ARGS_REQUESTER}_MISSING_MODULES CACHE)
    foreach(module ${PARSED_ARGS_CHECK})
        check_module_dependencies(${module})
        if(DISABLE_${module})
            set(DISABLE_${PARSED_ARGS_REQUESTER} TRUE CACHE INTERNAL "")
            set(${PARSED_ARGS_REQUESTER}_MISSING_MODULES ${${PARSED_ARGS_REQUESTER}_MISSING_MODULES} ${module} CACHE INTERNAL "")
        endif()
    endforeach(module)
endfunction(check_module_status)


# This function checks for a requestor module if the given libraries are available. If any of these
# is missing the requestor module is DISABLED, since all these libraries are considered its
# dependencies. It modifies this variable:
#    - DISABLE_REQUESTOR_MODULE_NAME: Is set to ON if any of the dependency modules is missing.
#    - MISSING_LIBRARIES: If the library is missing it will be added to this list.
# It creates the variable:
#    - REQUESTOR_MODULE_NAME_MISSING_LIBRARIES: List of all the needed libraries that are missing.
#
# check_library_status(
#     REQUESTER module_name -> Name of the Module. Ex: TemplateModule.
#     CHECK library_names -> List of all the library names that are a dependency. It has to be the
#                            same name than in the _FOUND variable created by FIND_PACKAGE().
#                            Ex: Boost.
# )
function(check_library_status)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs REQUESTER CHECK)
    cmake_parse_arguments(PARSED_ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(NOT DEFINED PARSED_ARGS_REQUESTER)
        message(FATAL_ERROR "You must provide a requester module")
    endif()
    if(NOT DEFINED PARSED_ARGS_CHECK)
        message(FATAL_ERROR "You must provide a set of libraries to be checked")
    endif()
    # First clear old variables
    unset(${PARSED_ARGS_REQUESTER}_MISSING_LIBRARIES CACHE)
    foreach(library ${PARSED_ARGS_CHECK})
        if(NOT ${library}_FOUND)
            set(DISABLE_${PARSED_ARGS_REQUESTER} TRUE CACHE INTERNAL "")
            set(${PARSED_ARGS_REQUESTER}_MISSING_LIBRARIES ${${PARSED_ARGS_REQUESTER}_MISSING_LIBRARIES} ${library} CACHE INTERNAL "")
            set(LIBRARY_ALREADY_LISTED FALSE)
            foreach(libraryMissing ${MISSING_LIBRARIES})
                if (${libraryMissing} STREQUAL ${library})
                    set(LIBRARY_ALREADY_LISTED TRUE)
                endif()
            endforeach(libraryMissing)
            if (NOT ${LIBRARY_ALREADY_LISTED})
                set(MISSING_LIBRARIES ${MISSING_LIBRARIES} ${library} CACHE INTERNAL "")
            endif()
        endif()
    endforeach(library)
endfunction(check_library_status)

# This function simply marks the module as checked
function(mark_module_or_library_as_checked module)
    set(CHECKED_${module} TRUE CACHE INTERNAL "")
endfunction(mark_module_or_library_as_checked)

# This function install the missing libraries listed in MISSING_LIBRARIES.
function(install_missing_libraries)
    set(NEW_LIBRARIES_INSTALLED FALSE CACHE INTERNAL "")
    if(NOT DEFINED MISSING_LIBRARIES)
        return()
    endif()
    message(WARNING "The automatic installation of libraries is not yet implemented")
endfunction(install_missing_libraries)


# This function clear all the following variables:
#    - BUILD_*
#    - CHECKED_*
#    - DISABLE_*
function(clear_checked_variable_of_disabled_modules)
    foreach(module ${MODULES_LIST})
        if(BUILD_${module} AND CHECKED_${module} AND DISABLE_${module})
            set(CHECKED_${module} FALSE CACHE INTERNAL "")
        endif()
    endforeach(module)
endfunction(clear_checked_variable_of_disabled_modules)


# This function prints all the modules in MODULES_LIST and their status, listing missing libraries
# and modules.
function(print_modules_summary)
    message(STATUS "      ")
    message(STATUS "      ")
    message(STATUS "${BOLD_MAGENTA}==================================== Modules Dependency Summary ====================================${COLOR_RESET}")
    message(STATUS "      ")
    foreach(locations ${LOCATIONS_LIST})
        message(STATUS "      ")
        message(STATUS "  ${BOLD_MAGENTA} ${locations} ${COLOR_RESET}")
        foreach(module ${MODULES_LIST})
            if(${module}_LOCATION STREQUAL ${locations})
                if(NOT BUILD_${module})
                    message(STATUS "    ${BOLD_RED} - ${module} is OFF ${COLOR_RESET}")
                elseif(BUILD_${module} AND CHECKED_${module} AND DISABLE_${module})
                    message(STATUS "    ${BOLD_YELLOW} - ${module} is DISABLE ${COLOR_RESET}")
                    message(STATUS "         -> Missing Modules: ${YELLOW}${${module}_MISSING_MODULES} ${COLOR_RESET}")
                    message(STATUS "         -> Missing Libraries: ${YELLOW}${${module}_MISSING_LIBRARIES} ${COLOR_RESET}")
                elseif(BUILD_${module} AND CHECKED_${module} AND NOT DISABLE_${module})
                    message(STATUS "    ${BOLD_GREEN} - ${module} is ON ${COLOR_RESET}")
                    print_module_implementations_summary(${module})
                else()
                    message(STATUS "    ${BOLD_YELLOW} - ${module} is UNKNOWN ${COLOR_RESET}")
                endif()
            endif()
        endforeach(module)
    endforeach(locations)
    message(STATUS "      ")
    message(STATUS "      ")
    message(STATUS "${BOLD_MAGENTA}====================================================================================================${COLOR_RESET}")
    message(STATUS "      ")
    message(STATUS "      ")
endfunction(print_modules_summary)

# This function print all the implementations of a specific modules and their status, listing
# missing libraries and modules.
#
# print_module_implementations_summary(
#     module_name -> Name of the Module. Ex: TemplateModule.
# )
function(print_module_implementations_summary implementedModule)
    foreach(module ${MODULES_WITH_IMPLEMENTATIONS})
        if (${implementedModule} STREQUAL ${module})
            foreach(implementation ${${implementedModule}_IMPLEMENTATIONS_LIST})
                if(NOT BUILD_${implementation})
                    message(STATUS "         - ${implementation} is ${RED}OFF ${COLOR_RESET}")
                elseif(BUILD_${implementation} AND DISABLE_${implementation})
                    message(STATUS "         - ${implementation} is ${YELLOW}DISABLE ${COLOR_RESET}")
                    message(STATUS "             -> Missing Modules: ${YELLOW}${${implementation}_MISSING_MODULES} ${COLOR_RESET}")
                    message(STATUS "             -> Missing Libraries: ${YELLOW}${${implementation}_MISSING_LIBRARIES} ${COLOR_RESET}")
                elseif(BUILD_${implementation} AND NOT DISABLE_${implementation})
                    message(STATUS "         - ${implementation} is ${GREEN}ON ${COLOR_RESET}")
                else()
                    message(STATUS "         - ${implementation} is ${YELLOW}UNKNOWN ${COLOR_RESET}")
                endif()
            endforeach(implementation)
        endif()
    endforeach(module)
endfunction(print_module_implementations_summary)
