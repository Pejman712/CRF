#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        RobotBaseController
                    CHECK
                        CommonInterfaces
                        Types
)

# The communication point can be treated as an implementation for CMake
crf_implementation(IMPLEMENTATION RobotBaseControllerCommunicationPoint        OF RobotBaseController        IS  ON)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION RobotBaseControllerClient                    OF RobotBaseController        IS  ON)
crf_implementation(IMPLEMENTATION RobotBaseVelocityController                  OF RobotBaseController        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(RobotBaseController)
