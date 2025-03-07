#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2023                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        Tool
                    CHECK
                        ErrorHandler
                        EventLogger
                        CommonInterfaces
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         Tool
                     CHECK
                         urdfdom
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION ActiveToolCommunicationPoint      OF Tool        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(Tool)
