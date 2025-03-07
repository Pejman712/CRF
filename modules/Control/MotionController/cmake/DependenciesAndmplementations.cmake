#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

check_module_status(REQUESTER
                        MotionController
                    CHECK
                        Types
                        ErrorHandler
                        EventLogger
                        Robot
                        Controller
)

crf_implementation(IMPLEMENTATION PathFollower                        OF MotionController        IS  ON)
crf_implementation(IMPLEMENTATION Teleoperation                       OF MotionController        IS  ON)
crf_implementation(IMPLEMENTATION MotionControllerCommunicationPoint  OF MotionController        IS  ON)
crf_implementation(IMPLEMENTATION MotionControllerClient              OF MotionController        IS  ON)

mark_module_or_library_as_checked(MotionController)
