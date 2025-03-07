#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MISSION_MANAGER_TESTS_FILES main.cpp
)

if (NOT DISABLE_RPSurveyLHC)
    set(MISSION_MANAGER_TESTS_FILES ${MISSION_MANAGER_TESTS_FILES}
                                    RPSurveyLHC/RPSurveyLHCTests.cpp
    )

    if (NOT DISABLE_MissionManagerCommunicationPoint)
        set(MISSION_MANAGER_TESTS_FILES ${MISSION_MANAGER_TESTS_FILES}
                                        MissionManagerCommunicationPoint/RPSurveyLHCCommunicationPointTests.cpp
        )
    endif()
endif()

if (NOT DISABLE_ScienceGateway)
    set(MISSION_MANAGER_TESTS_FILES ${MISSION_MANAGER_TESTS_FILES}
                                    ScienceGateway/ScienceGatewayTests.cpp
    )
endif()

add_executable(ut-missionmanager ${MISSION_MANAGER_TESTS_FILES})
target_link_libraries(ut-missionmanager gmock
                                        EventLogger
                                        MissionManager
)

add_test(MissionManagerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-missionmanager)
