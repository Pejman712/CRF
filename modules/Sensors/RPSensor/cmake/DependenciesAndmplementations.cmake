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
                        RPSensor
                    CHECK
                        CommonInterfaces
)

# The communication point can be treated as an implementation for CMake
crf_implementation(IMPLEMENTATION RPSensorCommunicationPoint         OF RPSensor         IS ON)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION RPSensorClient                    OF RPSensor         IS ON)
crf_implementation(IMPLEMENTATION AtomtexBDKG24                     OF RPSensor         IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(RPSensor)
