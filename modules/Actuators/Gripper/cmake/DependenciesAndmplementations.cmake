#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        Gripper
                    CHECK
                        EventLogger
                        CommonInterfaces
)

check_library_status(REQUESTER
                        Gripper
                    CHECK
                        Boost
)

crf_implementation(IMPLEMENTATION SchunkGripperCANOpen        OF Gripper        IS  ON)
crf_implementation(IMPLEMENTATION ECBPMiSerial                OF Gripper        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(Gripper)
