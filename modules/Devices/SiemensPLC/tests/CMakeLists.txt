#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(SIEMENS_PLC_TESTS_FILES main.cpp
                            SiemensPLC300Tests.cpp
                            SiemensPLC1500Tests.cpp
                            SiemensPLCTypeConverterTests.cpp
)

add_executable(ut-siemensplc ${SIEMENS_PLC_TESTS_FILES})
target_link_libraries(ut-siemensplc gmock
                                    SiemensPLC)

add_test(SiemensPLCUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-siemensplc)
