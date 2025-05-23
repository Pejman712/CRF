#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the libraries dependencies
check_library_status(REQUESTER
                         TRACInverseKinematics
                     CHECK
                         Boost
                         EIGEN3
                         NLopt
                         orocos_kdl
)

# Inform that this library was already checked to not repeat this procedure
mark_module_or_library_as_checked(TRACInverseKinematics)