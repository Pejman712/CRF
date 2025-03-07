#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Shuqi Zhao CERN BE/CEM/MRO 2023                                                                           ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "KortexAPI")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                ${Kortex_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${Kortex_INCLUDE_DIRS}
)
set(SRC_FILES ./src/KortexMovementAPIInterface.cpp
              ./src/KortexVisionAPIInterface.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
