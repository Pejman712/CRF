#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Bartosz Sójka CERN BE/CEM/MRO 2024                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(TYPES_TESTS_FILES main.cpp
                      ArithmeticTests.cpp
                      CardanXYZTests.cpp
                      ComparisonTests.cpp
                      ConversionsTests.cpp
                      EulerAnglesTests.cpp
                      EulerZXZTests.cpp
                      IsValidRotationTests.cpp
                      JsonConvertersTests.cpp
                      RepresentationsTests.cpp
                      RotationClassTests.cpp
)

add_executable(ut-rotation ${TYPES_TESTS_FILES})
target_link_libraries(ut-rotation gmock
                                  Rotation
)

add_test(RotationUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-rotation)
