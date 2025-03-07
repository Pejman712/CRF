#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        ViconAPI
                    CHECK
                        EventLogger
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         ViconAPI
                     CHECK
                         Vicon
)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(ViconAPI)
