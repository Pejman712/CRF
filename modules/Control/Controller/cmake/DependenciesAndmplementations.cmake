#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        Controller
                    CHECK
                        Types
                        ErrorHandler
)

crf_implementation(IMPLEMENTATION DirectOpenLoopVelocity                    OF Controller        IS  ON)
crf_implementation(IMPLEMENTATION PositionCtrlVelocityFF                    OF Controller        IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(Controller)
