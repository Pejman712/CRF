#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(LIBRARY_NAME "ReflexxesMotionLibrary")

if(DISABLE_${LIBRARY_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/ReflexxesAPI.cpp
              ./src/TypeIIRMLCalculatePositionalExtrems.cpp
              ./src/TypeIIRMLDecisions.cpp
              ./src/TypeIIRMLDecisionTree1A.cpp
              ./src/TypeIIRMLDecisionTree1B.cpp
              ./src/TypeIIRMLDecisionTree1C.cpp
              ./src/TypeIIRMLDecisionTree2.cpp
              ./src/TypeIIRMLFallBackStrategy.cpp
              ./src/TypeIIRMLIsPhaseSynchronizationPossible.cpp
              ./src/TypeIIRMLPolynomial.cpp
              ./src/TypeIIRMLPosition.cpp
              ./src/TypeIIRMLQuicksort.cpp
              ./src/TypeIIRMLSetupModifiedSelectionVector.cpp
              ./src/TypeIIRMLStep1.cpp
              ./src/TypeIIRMLStep1IntermediateProfiles.cpp
              ./src/TypeIIRMLStep1Profiles.cpp
              ./src/TypeIIRMLStep2.cpp
              ./src/TypeIIRMLStep2IntermediateProfiles.cpp
              ./src/TypeIIRMLStep2PhaseSynchronization.cpp
              ./src/TypeIIRMLStep2Profiles.cpp
              ./src/TypeIIRMLStep2WithoutSynchronization.cpp
              ./src/TypeIIRMLStep3.cpp
              ./src/TypeIIRMLVelocityCalculatePositionalExtrems.cpp
              ./src/TypeIIRMLVelocity.cpp
              ./src/TypeIIRMLVelocityFallBackStrategy.cpp
              ./src/TypeIIRMLVelocityIsPhaseSynchronizationPossible.cpp
              ./src/TypeIIRMLVelocityMethods.cpp
              ./src/TypeIIRMLVelocitySetupPhaseSyncSelectionVector.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRC_FILES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIRS})
