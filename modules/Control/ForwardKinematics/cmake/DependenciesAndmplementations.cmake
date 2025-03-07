#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
# TODO(anyone): MathExprTk and urdfdom are implementation specific. We shouldn't force them
# but the Robot module needs to construct both modules so we need to know that all are available
check_module_status(REQUESTER
                        ForwardKinematics
                    CHECK
                        Types
                        MathExprTk
)

check_library_status(REQUESTER
                        ForwardKinematics
                    CHECK
                        urdfdom
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION MathExprForwardKinematics         OF ForwardKinematics     IS  ON)
crf_implementation(IMPLEMENTATION KinChainForwardKinematics         OF ForwardKinematics     IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(ForwardKinematics)
