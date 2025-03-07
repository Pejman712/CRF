#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        SensorValidationResults
                    CHECK
                        EventLogger
                        BLMTriggeringAnalysis
)

# Add the different implementations to check their dependenciess
crf_implementation(IMPLEMENTATION BLMValidationJSONResults            OF SensorValidationResults        IS  ON)
crf_implementation(IMPLEMENTATION PMIValidationJSONResults            OF SensorValidationResults        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(SensorValidationResults)
