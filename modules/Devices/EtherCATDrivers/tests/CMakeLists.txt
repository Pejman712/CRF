#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Natalija Topalovic CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(ETHERCATDRIVERS_TEST_FILES main.cpp
)

if (NOT DISABLE_EtherCATMaster)
    set(ETHERCATDRIVERS_TEST_FILES ${ETHERCATDRIVERS_TEST_FILES}
                                   EtherCATMaster/EtherCATMasterTests.cpp
    )
endif()

if (NOT DISABLE_BasicEtherCATDriver)
    set(ETHERCATDRIVERS_TEST_FILES ${ETHERCATDRIVERS_TEST_FILES}
                                   BasicEtherCATDriver/BasicEtherCATDriverTests.cpp
    )
endif()

add_executable(ut-ethercatdrivers ${ETHERCATDRIVERS_TEST_FILES})
target_link_libraries(ut-ethercatdrivers gmock
                                         EtherCATDrivers
)

add_test(EtherCATDriversUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-ethercatdrivers)
