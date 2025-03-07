#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Shuqi Zhao CERN BE/CEM/MRO 2023                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

check_module_status(REQUESTER
                        ScienceGateway
                    CHECK
                        Robot
                        MotionController
                        TrajectoryGenerator
                        Controller
                        KortexAPI
                        MissionUtility
                        Types
)

check_library_status(REQUESTER
                         ScienceGateway
                     CHECK
                         nlohmann_json
)
