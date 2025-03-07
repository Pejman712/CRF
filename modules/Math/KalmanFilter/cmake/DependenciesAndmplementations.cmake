#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2024                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

check_module_status(
    REQUESTER
        KalmanFilter
    CHECK
        EventLogger
)

check_library_status(
    REQUESTER
        KalmanFilter
    CHECK
        EIGEN3
)

mark_module_or_library_as_checked(KalmanFilter)
