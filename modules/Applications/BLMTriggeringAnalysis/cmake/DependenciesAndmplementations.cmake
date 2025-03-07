#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        BLMTriggeringAnalysis
                    CHECK
                        EventLogger
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION BLMTriggeringAnalysisClient         OF BLMTriggeringAnalysis         IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(BLMTriggeringAnalysis)
