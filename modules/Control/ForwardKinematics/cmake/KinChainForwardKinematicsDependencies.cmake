#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Ante Marić CERN BE/CEM/MRO 2023                                                                           ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        KinChainForwardKinematics
                    CHECK
                        EventLogger
                        Types
                        KinematicChain
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                        KinChainForwardKinematics
                     CHECK
                        EIGEN3
                        urdfdom
)
