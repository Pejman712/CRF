#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Pawel Ptasznik CERN EN/SMM/MRO 2017                                                                       ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

include(CTest)
include(ProcessorCount)

enable_testing()

# Download and unpack Google Test at configure time
configure_file(${CMAKE_SOURCE_DIR}/cmake/packages/GTestRepository.in ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} --build .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker settings on Windows
# Not sure if actually needed
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add Google Test directly to our build. This defines the GTest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build)

# Add this include directory manually because of a bug in GMock build system
include_directories(SYSTEM ${CMAKE_BINARY_DIR}/googletest-src/googletest/include)

# Create folder to locate test files
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/testfiles)

# Set enviroment variable to create XML reports
set(ENV{GTEST_OUTPUT} "xml:reports/")

if(CMAKE_BUILD_TYPE MATCHES Debug)
    ProcessorCount(NPROC)

    include(${CMAKE_SOURCE_DIR}/cmake/CodeCoverage.cmake)
    append_coverage_compiler_flags()
    set(GCOVR_ADDITIONAL_ARGS "--print-summary"
    )
    set(COVERAGE_EXCLUDES "/usr/*"
                          "/opt/*"
                          "bin/*"
                          "build/*"
                          "lib/*"
                          "libraries/*"
                          ".*/samples/*"
                          ".*/tests/*"
                          ".*Mock.hpp"
                          ".*MockConfiguration.hpp"
                          ".*/KinovaJacoAPI/*"
                          ".*/KortexAPI/*"
                          ".*/UniversalRobotRTDE/*"
    )

    if(NOT CMAKE_COVERAGE_OUTPUT_FORMAT)
        set(CMAKE_COVERAGE_OUTPUT_FORMAT "XML")
    endif()
    if(${CMAKE_COVERAGE_OUTPUT_FORMAT} STREQUAL "XML")
        setup_target_for_coverage_gcovr_xml(NAME test_coverage EXECUTABLE ctest -j${NPROC} --timeout 200)
    elseif(${CMAKE_COVERAGE_OUTPUT_FORMAT} STREQUAL "HTML")
        setup_target_for_coverage_gcovr_html(NAME test_coverage EXECUTABLE ctest -j${NPROC} --timeout 200)
    else()
        message(FATAL_ERROR "Invalid test coverage output format: ${CMAKE_COVERAGE_OUTPUT_FORMAT}")
    endif()
    message(STATUS "Test coverage output format: ${CMAKE_COVERAGE_OUTPUT_FORMAT}")

    find_program(MEMORYCHECK_COMMAND valgrind)
    set(MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full --fair-sched=yes -v")
endif()
