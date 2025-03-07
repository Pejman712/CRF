#pragma once
/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "FTSensor/IFTSensor.hpp"
#include "RobotArmControllersDeprecated/IRobotArmController.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "FTSensorCalibrator/SinusFunctor.hpp"
#include "FTSensorCalibrator/FTSensorCalibratorConfig.hpp"

namespace crf {
namespace applications {
namespace ftsensorcalibrator {

class FTSensorCalibrator : public utility::commoninterfaces::IInitializable {
 public:
    FTSensorCalibrator() = delete;

    FTSensorCalibrator(std::shared_ptr<robotarmcontroller::IRobotArmController> controller,
                       std::shared_ptr<sensors::ftsensor::IFTSensor> ftSensor,
                       std::string configFilePath);

    ~FTSensorCalibrator();

    bool initialize() override;

    bool deinitialize() override;

    // Moves the robot arm in an arc, then calculates the biases and the weight/ torque
    bool calibrate();

    // Checks in a given point the force values,
    // then rotates the arm around y by 45 degrees and checks again
    bool validateCalibration();

    FtSensorCalibratorConfig getCalibrationConfig();

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<robotarmcontroller::IRobotArmController> controller_;
    std::shared_ptr<sensors::ftsensor::IFTSensor> ftSensor_;
    FtSensorCalibratorConfig configFile_;
    std::string configFilePath_;
    std::string logFilePath_;
    std::ofstream myfile_;
    bool initialized_;

    // Stores only the mass and the the torque created by the mass
    std::vector<double> biases_;  // vector of 6 double corresponding to 6 measurement axis
    // vector of 2, contains the weight and the torque on Z
    std::vector<double> gravityCompensation_;
    // Stores the values of force-torque during the calibration,
    // it is 7x(number of points), first entry is position next 6 is the values
    Eigen::MatrixXf dataStorage_;
    // The robot arm position, when object is initialized
    utility::types::JointPositions initialJP_;
    // the point where the calibration arc starts
    utility::types::JointPositions calibrationInit_;
    utility::types::JointPositions calibrationStep_;
    bool setJointPositionsBlocking(const utility::types::JointPositions& target);
    bool setTaskPoseBlocking(const utility::types::TaskPose& target);
    bool logMeasurementPoint(float pos,
                             const crf::utility::types::TaskForceTorque& dataPoint, int index);
    Eigen::MatrixXf fitSinusoids();
    bool calculateMass(const Eigen::MatrixXf& sinusoidParams);
};

}  // namespace ftsensorcalibrator
}  // namespace applications
}  // namespace crf
