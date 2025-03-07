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
                        Robot
                    CHECK
                        EventLogger
                        CommonInterfaces
                        ErrorHandler
                        ForwardKinematics
                        Types
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         Robot
                     CHECK
                         nlohmann_json
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION CombinedRobot                     OF Robot        IS  ON)
crf_implementation(IMPLEMENTATION EtherCATRobot                     OF Robot        IS  ON)
crf_implementation(IMPLEMENTATION KinovaGen3                        OF Robot        IS  ON)
crf_implementation(IMPLEMENTATION KinovaJaco2                       OF Robot        IS  ON)
crf_implementation(IMPLEMENTATION UniversalRobot                    OF Robot        IS  ON)
crf_implementation(IMPLEMENTATION CiA402Robot                       OF Robot        IS  ON)
crf_implementation(IMPLEMENTATION Virtuose6DTAO                     OF Robot        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(Robot)
