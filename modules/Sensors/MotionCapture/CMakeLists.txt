#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "MotionCapture")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS ErrorHandler
                Types
                EventLogger
                CommonInterfaces
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_Vicon)
    set(MODULE_LIBS ViconAPI
                    ${MODULE_LIBS}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Vicon.cpp
    )
endif()

if(DISABLE_Vicon)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_Vicon)
    add_executable(ViconSample ./samples/ViconSample.cpp)
    target_link_libraries(ViconSample ${MODULE_NAME} ${MODULE_LIBS} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()
