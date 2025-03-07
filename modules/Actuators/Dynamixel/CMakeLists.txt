#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Carlos Veiga Almagro CERN EN/SMM/MRO                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "Dynamixel")

find_package(nlohmann_json REQUIRED)
find_package(DynamixelSDK REQUIRED)

set(MODULE_LIBS EventLogger
                CommonInterfaces
                ${nlohmann_json_LIBRARIES}
                ${DynamixelSDK_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${nlohmann_json_INCLUDE_DIRS}
                 ${DynamixelSDK_INCLUDE_DIRS}
)
set(SRC_FILES ./src/DynamixelSDK.cpp
              ./src/Dynamixel.cpp
              ./src/DynamixelConfiguration.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)
