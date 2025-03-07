#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(CONTROLLER_TESTS_FILES main.cpp
)

if (NOT DISBALE_PositionCtrlVelocityFF)
    set(CONTROLLER_TESTS_FILES ${CONTROLLER_TESTS_FILES}
                               PositionCtrlVelocityFFTests/PositionCtrlVelocityFFTests.cpp
    )
endif()

if (NOT DISBALE_PositionCtrlVelocityFF) # And not all others in the future
    set(CONTROLLER_TESTS_FILES ${CONTROLLER_TESTS_FILES}
                               IControllerTests.cpp
    )
endif()

add_executable(ut-controller ${CONTROLLER_TESTS_FILES})
target_link_libraries(ut-controller gmock
                                    Controller
                                    EventLogger
)

add_test(ControllerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-controller)
