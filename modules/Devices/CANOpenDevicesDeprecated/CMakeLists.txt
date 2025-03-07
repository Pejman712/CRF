#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "CANOpenDevicesDeprecated")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
                CANSocket
                ${Boost_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${Boost_INCLUDE_DIR}
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(SRC_FILES ./src/CANOpenIOs/CANOpenIOModule.cpp
              ./src/CANOpenMotors/MaxonEPOS4.cpp
              ./src/CANOpenMotors/MaxonEPOS2.cpp
              ./src/CANOpenMotors/ERB.cpp
              ./src/CANOpenContext.cpp
              ./src/CANOpenSdoManager.cpp
              ./src/ObjectDictionary.cpp
              ./src/ObjectDictionaryRegister.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(epos2Sample ./samples/epos2Sample.cpp)
target_link_libraries(epos2Sample ${MODULE_NAME})

add_executable(epos4Sample ./samples/epos4Sample.cpp)
target_link_libraries(epos4Sample ${MODULE_NAME})

add_executable(canIOSample ./samples/canIOSample.cpp)
target_link_libraries(canIOSample ${MODULE_NAME})

add_executable(ERBTest ./samples/ERBTest.cpp)
target_link_libraries(ERBTest ${MODULE_NAME})
