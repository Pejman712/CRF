#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "Gripper")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
                ${Boost_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${Boost_INCLUDE_DIRS}
)
set(SRC_FILES
)

if (NOT DISABLE_SchunkGripperCANOpen)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CANOpenDevicesDeprecated
                    CANSocket
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/SchunkGripperCANOpen/SchunkGripperCANOpen.cpp
    )
endif()

if (NOT DISABLE_ECBPMiSerial)
    set(MODULE_LIBS ${MODULE_LIBS}
                    SerialCommunication
                    Tool
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/ECBPMi/ECBPMiSerial.cpp
    )
endif()

if (DISABLE_SchunkGripperCANOpen AND DISABLE_ECBPMiSerial)
    add_library(${MODULE_NAME} INTERFACE ${SRC_FILES})
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME})
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

find_package(Boost COMPONENTS program_options)

if (NOT DISABLE_SchunkGripperCANOpen)
    add_executable(SchunkGripperCANOpenSample ./samples/SchunkGripperCANOpenSample.cpp)
    target_link_libraries(SchunkGripperCANOpenSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if (NOT DISABLE_ECBPMiSerial)
    add_executable(ECBPMiSerialSample ./samples/ECBPMiSerialSample.cpp)
    target_link_libraries(ECBPMiSerialSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()
