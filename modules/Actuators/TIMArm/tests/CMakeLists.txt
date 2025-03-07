#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(TIM_ARM_TESTS_FILES main.cpp
                        TIMArmConfigurationTests.cpp
                        # TIMArmTests.cpp
)

add_executable(ut-timarm ${TIM_ARM_TESTS_FILES})
target_link_libraries(ut-timarm gmock
                                TIMArm
)

add_test(TIMArmUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-timarm)
