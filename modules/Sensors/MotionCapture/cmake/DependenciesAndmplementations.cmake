#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        MotionCapture
                    CHECK
                        EventLogger
                        CommonInterfaces
                        ErrorHandler
                        Types
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION Vicon                       OF MotionCapture        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(MotionCapture)
