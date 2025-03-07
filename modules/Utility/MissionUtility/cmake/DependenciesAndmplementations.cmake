#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

check_module_status(REQUESTER
                        MissionUtility
                    CHECK
                        EventLogger
                        CommonInterfaces
)

crf_implementation(IMPLEMENTATION DeployableRobotArm                      OF MissionUtility      IS ON)
crf_implementation(IMPLEMENTATION DeployableTIMRPWagonArm                 OF MissionUtility      IS ON)
crf_implementation(IMPLEMENTATION DeployableRobot                         OF MissionUtility      IS ON)
crf_implementation(IMPLEMENTATION TIMMovement                             OF MissionUtility      IS ON)

mark_module_or_library_as_checked(MissionUtility)
