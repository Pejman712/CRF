#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(CAVITY_MANAGER_TESTS_FILES main.cpp
)

if (NOT DISABLE_EtherCATSRFCavityManager)
    set(CAVITY_MANAGER_TESTS_FILES ${CAVITY_MANAGER_TESTS_FILES}
                                   EtherCATSRFCavityManagerTests.cpp
    )
endif()

add_executable(ut-srfcavityManager ${CAVITY_MANAGER_TESTS_FILES})
target_link_libraries(ut-srfcavityManager gmock
                                         EventLogger
                                         SRFCavityManager
                                         EtherCATDevices
)

add_test(EtherCATSRFCavityManagerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-srfcavityManager)
