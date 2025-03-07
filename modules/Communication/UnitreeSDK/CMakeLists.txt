#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Lucas Braud CERN BE/CEM/MRO 2023                                                                          ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "UnitreeSDK")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
)

add_library(${MODULE_NAME} INTERFACE)
target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
