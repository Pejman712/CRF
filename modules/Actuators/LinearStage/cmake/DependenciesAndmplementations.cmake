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
                        LinearStage
                    CHECK
                        EventLogger
                        ErrorHandler
                        CommonInterfaces
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         LinearStage
                     CHECK
                         Boost
                         nlohmann_json
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION CANOpenLinearStage                    OF LinearStage        IS  ON)
crf_implementation(IMPLEMENTATION EtherCATLinearActuator                OF LinearStage        IS  ON)
crf_implementation(IMPLEMENTATION LinearActuatorCommunicationPoint      OF LinearStage        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(LinearStage)
