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
                        GeometricMethods
                    CHECK
                        EventLogger
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION DeBoor                       OF GeometricMethods    IS ON)
crf_implementation(IMPLEMENTATION Sinusoid                     OF GeometricMethods    IS ON)
crf_implementation(IMPLEMENTATION CubicPolynomial              OF GeometricMethods    IS ON)
crf_implementation(IMPLEMENTATION CubicOrientationSpline       OF GeometricMethods    IS ON)
crf_implementation(IMPLEMENTATION PointToPoint                 OF GeometricMethods    IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(GeometricMethods)
