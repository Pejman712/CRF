#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

cmake_minimum_required(VERSION 3.5)
project(CERNRoboticFramework C CXX)

file(MAKE_DIRECTORY bin)
file(MAKE_DIRECTORY lib)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CorePackages.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CRFOptions.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModulesDependencies.cmake)

# Clean the lists if they exist in the cache to determine them again
clean_modules_and_implementations_lists()

# External Libraries
# ================================================================================================

crf_external_library(LIBRARY MathExprTk                                                     IS  ON)
crf_external_library(LIBRARY ReflexxesMotionLibrary                                         IS  ON)
crf_external_library(LIBRARY Snap7                                                          IS  ON)
crf_external_library(LIBRARY ThreadPool                                                     IS  ON)
crf_external_library(LIBRARY TimeOptimalTrajectoryGenerator                                 IS  ON)
crf_external_library(LIBRARY TRACInverseKinematics                                          IS  ON)


# Modules
# ================================================================================================

crf_module(MODULE CERNBot                         IN Actuators                              IS  ON)
crf_module(MODULE CERNBot2                        IN Actuators                              IS  ON)
crf_module(MODULE CHARMBot                        IN Actuators                              IS  ON)
crf_module(MODULE Dynamixel                       IN Actuators                              IS OFF)
crf_module(MODULE SPSRobot                        IN Actuators                              IS  ON)
crf_module(MODULE EtherCATRobotArm                IN Actuators                              IS  ON)
crf_module(MODULE Gripper                         IN Actuators                              IS  ON)
crf_module(MODULE KinovaArm                       IN Actuators                              IS  ON)
crf_module(MODULE LinearStage                     IN Actuators                              IS  ON)
crf_module(MODULE MechanicalStabilizer            IN Actuators                              IS  ON)
crf_module(MODULE PanTilt                         IN Actuators                              IS OFF)
crf_module(MODULE Robot                           IN Actuators                              IS  ON)
crf_module(MODULE PilzArm                         IN Actuators                              IS  ON)
crf_module(MODULE RobotArm                        IN Actuators                              IS  ON)
crf_module(MODULE RobotBase                       IN Actuators                              IS  ON)
crf_module(MODULE RobotiqGripper                  IN Actuators                              IS OFF)
crf_module(MODULE RobotiqGripper140               IN Actuators                              IS OFF)
crf_module(MODULE SchunkArm                       IN Actuators                              IS  ON)
crf_module(MODULE SchunkPowerCube                 IN Actuators                              IS OFF)
crf_module(MODULE Shielding                       IN Actuators                              IS  ON)
crf_module(MODULE SRFCavityManager                IN Actuators                              IS  ON)
crf_module(MODULE TIM                             IN Actuators                              IS  ON)
crf_module(MODULE TIMArm                          IN Actuators                              IS  ON)
crf_module(MODULE TIMRPWagon                      IN Actuators                              IS  ON)
crf_module(MODULE Tools                           IN Actuators                              IS OFF)
crf_module(MODULE UniversalRobotDeprecated        IN Actuators                              IS  ON)
crf_module(MODULE YoubotArm                       IN Actuators                              IS OFF)
crf_module(MODULE YoubotBase                      IN Actuators                              IS OFF)

# ================================================================================================

crf_module(MODULE BLMTriggeringAnalysis           IN Applications                           IS  ON)
crf_module(MODULE DsoDevel                        IN Applications                           IS OFF)
crf_module(MODULE FTSensorCalibrator              IN Applications                           IS OFF)
crf_module(MODULE HealthDetection                 IN Applications                           IS OFF)
crf_module(MODULE MissionManager                  IN Applications                           IS  ON)
crf_module(MODULE PersonFollower-DEPRECATED       IN Applications                           IS OFF)
crf_module(MODULE SensorValidationResults         IN Applications                           IS  ON)
crf_module(MODULE TIMAlignment-DEPRECATED         IN Applications                           IS OFF)
crf_module(MODULE WallDetector                    IN Applications                           IS OFF)

# ================================================================================================

crf_module(MODULE CANSocket                       IN Communication                          IS  ON)
crf_module(MODULE CommunicationPointServer        IN Communication                          IS  ON)
crf_module(MODULE ComponentAccessControl          IN Communication                          IS  ON)
crf_module(MODULE DataPackets                     IN Communication                          IS  ON)
crf_module(MODULE DataPacketSocket                IN Communication                          IS  ON)
crf_module(MODULE Haption                         IN Communication                          IS  ON)
crf_module(MODULE IPC-DEPRECATED                  IN Communication                          IS OFF)
crf_module(MODULE KinovaJacoAPI                   IN Communication                          IS  ON)
crf_module(MODULE KortexAPI                       IN Communication                          IS  ON)
crf_module(MODULE MailSender                      IN Communication                          IS  ON)
crf_module(MODULE NetworkClient                   IN Communication                          IS OFF)
crf_module(MODULE NetworkServer                   IN Communication                          IS OFF)
crf_module(MODULE SerialCommunication             IN Communication                          IS  ON)
crf_module(MODULE Sockets                         IN Communication                          IS  ON)
crf_module(MODULE SOEMAPI                         IN Communication                          IS  ON)
crf_module(MODULE SqlAdapter                      IN Communication                          IS OFF)
crf_module(MODULE TimeSynchronization             IN Communication                          IS OFF)
crf_module(MODULE UnitreeSDK                      IN Communication                          IS ON)
crf_module(MODULE UniversalRobotRTDE              IN Communication                          IS ON)
crf_module(MODULE ViconAPI                        IN Communication                          IS  ON)
crf_module(MODULE WebServers                      IN Communication                          IS OFF)

# ================================================================================================

crf_module(MODULE ClosedLoopController            IN Control                                IS  ON)
crf_module(MODULE Controller                      IN Control                                IS  ON)
crf_module(MODULE ForwardKinematics               IN Control                                IS  ON)
crf_module(MODULE InverseKinematics               IN Control                                IS  ON)
crf_module(MODULE MotionController                IN Control                                IS  ON)
crf_module(MODULE RobotArmController              IN Control                                IS  ON)
crf_module(MODULE RobotArmControllers-DEPRECATED  IN Control                                IS OFF)
crf_module(MODULE RobotArmKinematics              IN Control                                IS  ON)
crf_module(MODULE RobotBaseController             IN Control                                IS  ON)
crf_module(MODULE TrajectoryGenerator             IN Control                                IS  ON)
crf_module(MODULE TrajectoryGeneratorDeprecated   IN Control                                IS  ON)
crf_module(MODULE TrajectoryPointGenerator        IN Control                                IS  ON)

# ================================================================================================

crf_module(MODULE CANOpenDevicesDeprecated        IN Devices                                IS  ON)
crf_module(MODULE CANopenDrivers                  IN Devices                                IS  ON)
crf_module(MODULE Controllino                     IN Devices                                IS OFF)
crf_module(MODULE EtherCATDevices                 IN Devices                                IS  ON)
crf_module(MODULE EtherCATDrivers                 IN Devices                                IS  ON)
crf_module(MODULE SiemensPLC                      IN Devices                                IS  ON)
crf_module(MODULE TeltonikaRUT                    IN Devices                                IS OFF)
crf_module(MODULE Tool                            IN Devices                                IS  ON)

# ================================================================================================

crf_module(MODULE DistanceMeasures                IN Math                                   IS  ON)
crf_module(MODULE FourierTransform                IN Math                                   IS  ON)
crf_module(MODULE GeometricMethods                IN Math                                   IS  ON)
crf_module(MODULE InputShaper                     IN Math                                   IS  ON)
crf_module(MODULE Jacobian                        IN Math                                   IS  ON)
crf_module(MODULE KalmanFilter                    IN Math                                   IS  ON)
crf_module(MODULE KinematicChain                  IN Math                                   IS  ON)
crf_module(MODULE MassSpringDamper                IN Math                                   IS  ON)
crf_module(MODULE PeakDetection                   IN Math                                   IS OFF)
crf_module(MODULE Rotation                        IN Math                                   IS  ON)
crf_module(MODULE DigitalFilter                   IN Math                                   IS  ON)

# ================================================================================================

crf_module(MODULE CollisionDetector               IN Navigation                             IS  ON)
crf_module(MODULE GraphOptimization               IN Navigation                             IS  ON)
crf_module(MODULE MotionPlanner                   IN Navigation                             IS OFF)
crf_module(MODULE PathPlanner                     IN Navigation                             IS  ON)
crf_module(MODULE RobotPoseEstimators             IN Navigation                             IS OFF)
crf_module(MODULE StateEstimator                  IN Navigation                             IS OFF)

# ================================================================================================

crf_module(MODULE Cameras                         IN Sensors                                IS  ON)
crf_module(MODULE EnvironmentalSensors            IN Sensors                                IS  ON)
crf_module(MODULE FTSensor                        IN Sensors                                IS  ON)
crf_module(MODULE IMU                             IN Sensors                                IS  ON)
crf_module(MODULE Inclinometer                    IN Sensors                                IS OFF)
crf_module(MODULE Laser                           IN Sensors                                IS  ON)
crf_module(MODULE LeakDetector                    IN Sensors                                IS OFF)
crf_module(MODULE MotionCapture                   IN Sensors                                IS  ON)
crf_module(MODULE Radars                          IN Sensors                                IS OFF)
crf_module(MODULE RGBDCameras                     IN Sensors                                IS  ON)
crf_module(MODULE RPSensor                        IN Sensors                                IS  ON)
crf_module(MODULE SonyDSCCamera                   IN Sensors                                IS OFF)
crf_module(MODULE ThermalCamera                   IN Sensors                                IS OFF)
crf_module(MODULE TrackingCamera                  IN Sensors                                IS OFF)
crf_module(MODULE XlsAdapter                      IN Sensors                                IS OFF)

# ================================================================================================

crf_module(MODULE CommonInterfaces                IN Utility                                IS  ON)
crf_module(MODULE CommunicationUtility            IN Utility                                IS  ON)
crf_module(MODULE DataLogger                      IN Utility                                IS OFF)
crf_module(MODULE DeviceManager                   IN Utility                                IS  ON)
crf_module(MODULE ErrorHandler                    IN Utility                                IS  ON)
crf_module(MODULE EventLogger                     IN Utility                                IS  ON)
crf_module(MODULE GraphPlot                       IN Utility                                IS  ON)
crf_module(MODULE MissionUtility                  IN Utility                                IS  ON)
crf_module(MODULE RGBDVisionUtility               IN Utility                                IS OFF)
crf_module(MODULE RobotUtility                    IN Utility                                IS  ON)
crf_module(MODULE Types                           IN Utility                                IS  ON)
crf_module(MODULE VisionUtility                   IN Utility                                IS  ON)

# ================================================================================================

crf_module(MODULE DepthEstimation                 IN Vision                                 IS OFF)
crf_module(MODULE DirectSparseOdometry            IN Vision                                 IS OFF)
crf_module(MODULE HandEyeCalibration              IN Vision                                 IS OFF)
crf_module(MODULE InsulateFrameWorks              IN Vision                                 IS OFF)
crf_module(MODULE Mapper3d                        IN Vision                                 IS  OFF)
crf_module(MODULE ObjectPoseEstimator             IN Vision                                 IS OFF)
crf_module(MODULE CodeReader                      IN Vision                                 IS  ON)
crf_module(MODULE VideoCodecs                     IN Vision                                 IS  ON)

# Tests
# ================================================================================================
crf_option(UnitTests                        "Build Modules Unit Tests using Gtest"              ON)
crf_option(IntegrationTests                 "Build Modules Unit Tests using Gtest"             OFF)
crf_option(IntallDependencies               "Tries to install missing libraries"               OFF)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(BUILD_SHARED_LIBS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Werror -O2 -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Werror -O -g -pthread")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
set(CMAKE_CXX_STANDARD 17)

check_dependencies()
if (ENABLE_IntallDependencies)
    install_missing_libraries()
    if(NEW_LIBRARIES_INSTALLED)
        clear_checked_variable_of_disabled_modules()
        check_dependencies()
    endif()
endif()

# Prepare testing environment
if(ENABLE_UnitTests)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/TestConfiguration.cmake)
endif()

add_modules_and_libraries_subdirectories()

# Show modules and implementations that are allowed to be compiled
print_modules_summary()
