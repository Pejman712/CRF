#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##         Jean Paul Sulca CERN BM-CEM-MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION MassSpringDamperR1    OF MassSpringDamper    IS ON)
crf_implementation(IMPLEMENTATION MassSpringDamperRn    OF MassSpringDamper    IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(MassSpringDamper)
