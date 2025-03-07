#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        GraphOptimization
                    CHECK
                        EventLogger
                        #CommonInterfaces
                        G2O
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         GraphOptimization
                     CHECK
                         nlohmann_json
                         PCL
)

# Add the different implementations to check their dependencies
#crf_implementation(IMPLEMENTATION HokuyoLaser         OF Laser         IS  OFF)
#crf_implementation(IMPLEMENTATION RPLiDAR             OF Laser         IS  ON)
#crf_implementation(IMPLEMENTATION UnitreeL1           OF Laser         IS  ON)
#crf_implementation(IMPLEMENTATION VelodyneHDL         OF Laser         IS OFF)


# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(GraphOptimization)
