#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##         Bartosz Sójka CERN BE/CEM/MRO 2024                                                                        ##
#######################################################################################################################

# Check for the libraries dependencies
check_library_status(REQUESTER
                         StreamIO
                     CHECK
                         EIGEN3
                         orocos_kdl
                         OpenCV
)
