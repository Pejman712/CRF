#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check which external packages used by the CERNRoboticFramework are installed in the system. The
# packages are arranged alphabetically in categories.
#
# find_package(Name            Version         Requirement      Components)
#

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/packages)


######################################## Generic Packages #########################################
set(Boost_USE_DEBUG_LIBS ON)
set(Boost_USE_RELEASE_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_STLPORT OFF)
find_package(Boost                             QUIET)
find_package(spdlog                            QUIET)
find_package(CUDA                              QUIET)
find_package(Doxygen                           QUIET)

##################################### Mathematical Packages #######################################
find_package(Eigen3                            QUIET)
find_package(orocos_kdl                        QUIET)
find_package(NLopt                             QUIET)
find_package(ompl                              QUIET)
find_package(GSL                               QUIET)

####################################### Actuators Packages ########################################
find_package(SOEM                              QUIET)
find_package(KinovaApi                         QUIET)
find_package(Kortex                            QUIET)
find_package(ur_rtde                           QUIET)
find_package(RaptorAPI                         QUIET)

######################################## Sensors Packages #########################################
find_package(realsense2                        QUIET)
find_package(Vicon                             QUIET)
find_package(UnitreeL1SDK                      QUIET)
find_package(RPLiDARSDK                        QUIET)

######################### Serialization and Configuration Files Packages ##########################
find_package(nlohmann_json                     QUIET)
find_package(LZ4                               QUIET)
find_package(urdfdom                           QUIET)

##################################### Communication Packages ######################################
find_package(CURLPP                            QUIET)
find_package(lely                              QUIET)

############################### Image and Video Processing Packages ###############################
find_package(OpenCV                            QUIET)
find_package(x264                              QUIET)
find_package(x265                              QUIET)
find_package(VISP                              QUIET)
find_package(ZBar                              QUIET)

################################### 3D Data Processing Packages ###################################
find_package(PCL                               QUIET)
find_package(octomap                           QUIET)
find_package(fcl                               QUIET)
find_package(icl_core                          QUIET)
find_package(gpu_voxels                        QUIET)

################################ Visualization and User Interface #################################
find_package(VTK                               QUIET)
