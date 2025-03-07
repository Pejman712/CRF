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
                        PMIValidationJSONResults
                    CHECK
                        EventLogger
                        MailSender
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         PMIValidationJSONResults
                     CHECK
                         nlohmann_json
)
