#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        RGBDCameras
                    CHECK
                        Cameras
                        EventLogger
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                        RGBDCameras
                     CHECK
                        OpenCV
                        PCL
                        nlohmann_json
)

# The communication point can be treated as an implementation for CMake
crf_implementation(IMPLEMENTATION RGBDCameraCommunicationPoint         OF RGBDCameras         IS ON)
crf_implementation(IMPLEMENTATION RGBDCameraClient                     OF RGBDCameras         IS ON)
crf_implementation(IMPLEMENTATION RealSenseCamera                      OF RGBDCameras         IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(RGBDCameras)
