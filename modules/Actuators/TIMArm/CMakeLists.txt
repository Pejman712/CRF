#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2018                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "TIMArm")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                RobotArm
                EtherCATRobotArm
                KinovaArm
                ${Boost_SYSTEM_LIBRARY}
)
set(INCLUDE_DIRS ./include
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(SRC_FILES ./src/TIMArmConfiguration.cpp
              ./src/TIMArm.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

if (ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(TIMArmExample ./samples/TIMArmExample.cpp)
target_link_libraries(TIMArmExample ${MODULE_NAME} TIMArm)
