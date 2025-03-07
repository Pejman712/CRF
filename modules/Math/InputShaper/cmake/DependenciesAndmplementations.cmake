#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022                                                                  ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        InputShaper
                    CHECK
                        EventLogger
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         InputShaper
                     CHECK
                         EIGEN3
)

crf_implementation(IMPLEMENTATION CubicPolynomialShaper           OF InputShaper    IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(InputShaper)
