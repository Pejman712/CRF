#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "KinovaArm")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                RobotArm
                RobotArmKinematics
                ${KinovaApi_LIBRARIES}
                ${CMAKE_DL_LIBS}
)
set(INCLUDE_DIRS ./include
                 ${KinovaApi_INCLUDE_DIRS}
)
set(SRC_FILES ./src/KinovaApiInterface.cpp
              ./src/KinovaJaco.cpp
              ./src/KinovaArmConfiguration.cpp
              ./src/KinovaAdmittanceController.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(KinovaArmExample ./samples/KinovaArmExample.cpp)
target_link_libraries(KinovaArmExample ${MODULE_NAME})

add_executable(KinovaAdmittanceExample ./samples/KinovaAdmittanceExample.cpp)
target_link_libraries(KinovaAdmittanceExample ${MODULE_NAME})
