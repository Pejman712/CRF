#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Victor Drame CERN BE/CEM/MRO 2022                                                                         ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_library_status(REQUESTER
                        ArUcoCVDetector
                    CHECK
                        OpenCV
)
