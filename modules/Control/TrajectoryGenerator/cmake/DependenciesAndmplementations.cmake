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
                        TrajectoryGenerator
                    CHECK
                        Types
                        EventLogger
)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION PointToPointJointsTrajectory       OF TrajectoryGenerator    IS  ON)
crf_implementation(IMPLEMENTATION CubicJointsTrajectory              OF TrajectoryGenerator    IS  ON)
crf_implementation(IMPLEMENTATION CubicTaskTrajectory                OF TrajectoryGenerator    IS  ON)
crf_implementation(IMPLEMENTATION PreplannedTaskTrajectory           OF TrajectoryGenerator    IS  ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(TrajectoryGenerator)
