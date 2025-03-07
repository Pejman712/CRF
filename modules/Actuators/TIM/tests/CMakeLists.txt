#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(TIM_TESTS_FILES main.cpp
)

if (NOT DISABLE_TIMS300 AND NOT DISABLE_Snap7)
    set(TIM_TESTS_FILES ${TIM_TESTS_FILES}
                        TIMS300Tests.cpp
    )
endif()

add_executable(ut-tim ${TIM_TESTS_FILES})
if (NOT DISABLE_TIMS300 AND NOT DISABLE_Snap7)
    target_link_libraries(ut-tim gmock
                                 TIM
                                 Snap7
    )
else()
    target_link_libraries(ut-tim gmock
                                 TIM
    )
endif()

add_test(TIMUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-tim)
