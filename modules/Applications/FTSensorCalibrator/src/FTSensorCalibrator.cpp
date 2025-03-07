/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "FTSensorCalibrator/FTSensorCalibrator.hpp"
#include "FTSensorCalibrator/FTSensorCalibratorConfig.hpp"
#include "Types/Types.hpp"
#include <unsupported/Eigen/NonLinearOptimization>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

using crf::applications::robotarmcontroller::IRobotArmController;
using crf::sensors::ftsensor::IFTSensor;
using crf::sensors::ftsensor::IFTSensor;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::areAlmostEqual;
using json = nlohmann::json;

namespace crf {
namespace applications {
namespace ftsensorcalibrator {

FTSensorCalibrator::FTSensorCalibrator(std::shared_ptr<IRobotArmController> controller,
                       std::shared_ptr<IFTSensor> ftSensor,
                       std::string configFilePath):
        logger_("FTSensorCalibrator"),
        controller_(controller),
        ftSensor_(ftSensor),
        configFile_(),
        configFilePath_(configFilePath),
        logFilePath_(),
        myfile_(),
        initialized_(false),
        biases_(6),
        gravityCompensation_(2),  // Stores only the mass and the the torque created by the mass
        dataStorage_(),
        initialJP_(6),
        calibrationInit_(6),
        calibrationStep_(6) {
}

FTSensorCalibrator::~FTSensorCalibrator() {
    deinitialize();
}

bool FTSensorCalibrator::initialize() {
    if (initialized_) {
        logger_->debug("Already initialized!");
        return false;
    }

    if (!configFile_.parse(configFilePath_)) {
        logger_->debug("Config file parse failed!");
        return false;
    }

    logFilePath_ = configFile_.getLogFile();
    initialJP_ = controller_->getJointPositions();
    logger_->debug("Calibration Initialized");
    initialized_ = true;
    return true;
}

bool FTSensorCalibrator::deinitialize() {
    if (!initialized_) {
        logger_->debug("Not initialized!");
        return false;
    }

    if (!setJointPositionsBlocking(initialJP_)) {
        logger_->debug("Can not set robot arm position!");
        return false;
    }

    initialized_ = false;
    return true;
}

bool FTSensorCalibrator::calibrate() {
    if (!initialized_) {
        logger_->debug("Not initialized!");
        return false;
    }

    // Delete the contents of the log file
    myfile_.open(logFilePath_, std::ofstream::trunc);

    // Set datastorage size
    dataStorage_ = Eigen::MatrixXf(
        configFile_.getPathResolution() * configFile_.getMeasurementPerPoint(), 7);
    // Go to the base calibration position
    calibrationInit_ = configFile_.getInitialJP();
    calibrationStep_ = (configFile_.getEndJP() - calibrationInit_) *
                (1.0 / configFile_.getPathResolution());
    if (!setJointPositionsBlocking(calibrationInit_)) {
        return false;
    }

    // Log Data
    int dataIndex = 0;
    for (int i = 0; i < configFile_.getPathResolution(); i++) {
        calibrationInit_ += calibrationStep_;
        setJointPositionsBlocking(calibrationInit_);
        std::this_thread::sleep_for(
                std::chrono::milliseconds(10 * configFile_.getTimeBetweenMeasurementsMS()));
        for (int k = 0; k < configFile_.getMeasurementPerPoint(); k++) {
            TaskForceTorque ft = ftSensor_->getRawFT();
            logMeasurementPoint(calibrationInit_(4), ft, dataIndex);
            dataIndex++;
            std::this_thread::sleep_for(
                    std::chrono::milliseconds(configFile_.getTimeBetweenMeasurementsMS()));
        }
    }

    Eigen::MatrixXf fittedFunctions = fitSinusoids();

    json calib;
    for (int j = 0; j < 6; j++) {
        biases_[j] = fittedFunctions(j, 2);
    }
    calib["biases"] = biases_;

    // sets the gravityCompensation_ parameters
    calculateMass(fittedFunctions);

    calib["GravityCompensation"]["weight"] = gravityCompensation_[0];
    calib["GravityCompensation"]["torque"] = gravityCompensation_[1];
    myfile_ << calib.dump();
    myfile_.close();

    return true;
}

bool FTSensorCalibrator::validateCalibration() {
    if (!initialized_) {
        logger_->debug("Not initialized!");
        return false;
    }
    // We want the arm to be stationary thats why we sleep
    std::this_thread::sleep_for(std::chrono::seconds(10 *
         configFile_.getTimeBetweenMeasurementsMS()));
    // Update the biases in the first point
    // Generally values should be clsoe to zero, no matter what happens after update
    TaskPose cp = controller_->getTaskPose();
    ftSensor_->updateBias(cp);
    std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    if (!ftSensor_->isCalibrated()) {
        logger_->debug("FTSensor Validation Failed");
        return false;
    }

    TaskForceTorque ft1 = ftSensor_->getFTGravityFree(
        controller_->getTaskPose(), false);
    logger_->debug("The FT values in first position : {0}, {1}, {2}, {3}, {4}, {5}",
            ft1(0), ft1(1), ft1(2), ft1(3), ft1(4), ft1(5));

    double RMSForce = sqrt(pow(ft1(0), 2) + pow(ft1(1), 2) + pow(ft1(2), 2));
    double RMSTorque = sqrt(pow(ft1(3), 2) + pow(ft1(4), 2) + pow(ft1(5), 2));
    if ((RMSForce > configFile_.getForceThresholdN()) ||
            (RMSTorque > configFile_.getTorqueThresholdNm())) {
        logger_->debug("Validation failed, force error is too big");
        return false;
    }


    // Rotate it by 45 degrees around y
    crf::math::rotation::CardanXYZ rpy = cp.getCardanXYZ();
    if (rpy.pitch > 0) {
        rpy.pitch -= M_PI / 4;
    } else {
        rpy.pitch += M_PI / 4;
    }
    TaskPose cp2(rpy);
    setTaskPoseBlocking(cp2);
    std::this_thread::sleep_for(std::chrono::seconds(
        10 * configFile_.getTimeBetweenMeasurementsMS()));

    // Get second measurement
    TaskForceTorque ft2 = ftSensor_->getFTGravityFree(
        controller_->getTaskPose(), false);
    logger_->debug("The FT values in second position : {0}, {1}, {2}, {3}, {4}, {5}",
                   ft2(0), ft2(1), ft2(2), ft2(3), ft2(4), ft2(5));
    double RMSForce2 = sqrt(pow(ft2(0), 2) + pow(ft2(1), 2) + pow(ft2(2), 2));
    double RMSTorque2 = sqrt(pow(ft2(3), 2) + pow(ft2(4), 2) + pow(ft2(5), 2));
    if ((RMSForce2 > configFile_.getForceThresholdN()) ||
        (RMSTorque2 > configFile_.getTorqueThresholdNm())) {
        logger_->debug("Validation failed, force error is too big");
        return false;
    }

    logger_->debug("Validation succesful!");
    return true;
}

bool FTSensorCalibrator::setJointPositionsBlocking(const JointPositions& target) {
    bool result = controller_->setJointPositions(target);
    if (!result) {
        logger_->debug("Arm positioning failed");
        return false;
    }
    while (!areAlmostEqual(controller_->getJointPositions(), target)) {
        std::this_thread::sleep_for(
                std::chrono::milliseconds(10));
    }
    return true;
}

bool FTSensorCalibrator::setTaskPoseBlocking(const TaskPose& target) {
    bool result = controller_->setTaskPose(target);
    if (!result) {
        logger_->debug("Arm positioning failed");
        return false;
    }

    while (!areAlmostEqual(controller_->getTaskPose(), target)) {
        std::this_thread::sleep_for(
                std::chrono::milliseconds(10));
    }
    return true;
}

bool FTSensorCalibrator::logMeasurementPoint(
    float pos, const TaskForceTorque& dataPoint, int index) {
    dataStorage_(index, 0) = pos;
    for (int l = 0; l < 6; l++) {
        dataStorage_(index, l+1) = dataPoint(l);
    }
    return true;
}

Eigen::MatrixXf FTSensorCalibrator::fitSinusoids() {
    Eigen::MatrixXf sinusoidParameters(6, 3);
    // Fit a sinusoid to each one of the 6 axis
    for (int i = 1; i < 7; i++) {
        float mean = dataStorage_.col(i).mean();
        float stdev = std::sqrt((dataStorage_.col(i).array() - mean).square().sum()
                / (dataStorage_.col(i).size() - 1));
        logger_->info(" Mean: {0} Stdev: {1}", mean, stdev);
        Eigen::VectorXf x(3);
        x(0) = stdev;             // initial value for 'a'
        x(1) = 0;             // initial value for 'b'
        x(2) = mean;
        sinusFunctor functor;
        functor.measuredValues = dataStorage_;
        functor.m = dataStorage_.rows();  // number of data points
        functor.n = 3;  // number of paramters to optimize
        functor.w = 1;  // angular frequency
        functor.col = i;
        Eigen::LevenbergMarquardt<sinusFunctor, float> lm(functor);
        lm.parameters.epsfcn = 1e-8;  // smallest step in paramter space
        lm.parameters.maxfev = 1000;  // number of function evaluations
        Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);

        logger_->info("LM optimization status : {0}", status);
        logger_->info("Optimization results", status);
        logger_->info("\tamplitude: {0}", x(0));
        logger_->info("\tomega : {0}", functor.w);
        logger_->info("\tphase : {0}", x(1));
        logger_->info("\toffset : {0}", x(2));

        sinusoidParameters(i-1, 0) = x(0);
        sinusoidParameters(i-1, 1) = x(1);
        sinusoidParameters(i-1, 2) = x(2);
    }
    return sinusoidParameters;
}

bool FTSensorCalibrator::calculateMass(const Eigen::MatrixXf& sinusoidParams) {
    Eigen::MatrixXf mat(100, 6);
    float x = 0;
    for (int i =0; i < 100; i++) {
        for (int j = 0; j < 6; j++) {
            mat(i, j) = sinusoidParams(j, 0) * sin(x + sinusoidParams(j, 1));
        }
        x += (2 * M_PI) / 100;
    }

    mat = mat.array().square();
    Eigen::VectorXf sumOfRow(3);
    sumOfRow << 1, 1, 1;

    // Calculate mass
    Eigen::MatrixXf massMatrix = mat.block(0, 0, 100, 3);
    Eigen::VectorXf sumOfMass = massMatrix * sumOfRow;
    float mass = sqrt(sumOfMass.mean());
    gravityCompensation_[0] = mass;
    logger_->info("Mass {0}", mass);

    // Calculate torque
    Eigen::MatrixXf torqueMatrix = mat.block(0, 3, 100, 3);
    Eigen::VectorXf sumOfTorque = torqueMatrix * sumOfRow;
    float torque = sqrt(sumOfTorque.maxCoeff());
    gravityCompensation_[1] = torque;
    logger_->info("Torque {0}", torque);

    return true;
}

FtSensorCalibratorConfig FTSensorCalibrator::getCalibrationConfig() {
    return configFile_;
}

}  // namespace ftsensorcalibrator
}  // namespace applications
}  // namespace crf
