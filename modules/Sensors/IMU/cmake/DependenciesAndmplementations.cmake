#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2023                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        IMU
                    CHECK
                        CommonInterfaces
                        ErrorHandler
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION VMU931                    OF IMU      IS OFF)
crf_implementation(IMPLEMENTATION XsensMT                   OF IMU      IS OFF)
crf_implementation(IMPLEMENTATION IMUCommunicationPoint     OF IMU      IS  ON)
crf_implementation(IMPLEMENTATION Gable                     OF IMU      IS  ON)
crf_implementation(IMPLEMENTATION UnitreeL1IMU              OF IMU      IS  ON)
crf_implementation(IMPLEMENTATION RealSenseIMU              OF IMU      IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(IMU)

