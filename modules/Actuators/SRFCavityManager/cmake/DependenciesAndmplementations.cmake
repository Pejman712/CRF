#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Adrien Luthi CERN EN/SMM/MRO 2022                                                                         ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        SRFCavityManager
                    CHECK
                        EventLogger
                        CommonInterfaces
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION EtherCATSRFCavityManager           OF SRFCavityManager        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(SRFCavityManager)
