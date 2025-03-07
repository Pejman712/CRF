#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jean Paul Sulca CERN BM-CEM-MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        DigitalFilter
                    CHECK
                        ErrorHandler
                        EventLogger
)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(DigitalFilter)
