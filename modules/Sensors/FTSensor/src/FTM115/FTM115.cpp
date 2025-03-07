/* © Copyright CERN 2018.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <stdio.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include <CANSocket/ICANSocket.hpp>
#include <FTSensor/FTM115/FTM115.hpp>

namespace crf {
namespace sensors {
namespace ftsensor {

FTM115::FTM115(std::shared_ptr<crf::communication::cansocket::ICANSocket> socket):
    socket_(socket),
    logger_("FTM115"),
    initialized_(false),
    calibrated_(false),
    weight_(),
    torque_(),
    readerThread_(),
    coefficientMatrix_(6, 6),
    biases_(),
    runThread_(false) {
    logger_->debug("CTor");
    filter_ = std::make_unique<crf::math::kalmanfilter::SimplifiedFTKalmanFilter>(
        processNoise_, measurementNoise_);
    ft_ = crf::utility::types::TaskForceTorque();
    rawFt_ = crf::utility::types::TaskForceTorque();
}

FTM115::~FTM115() {
    logger_->debug("DTor");
    deinitialize();
}

bool FTM115::initialize() {
    logger_->debug("initialize");
    if (readerThread_.joinable()) {
        logger_->warn("Reader loop already started");
        return false;
    }

    if (!socket_->initialize()) {
        logger_->error("Initialization of CAN socket failed");
        return false;
    }

    // Sets the active calibration, theoretically there are from 0 to 15, but only 0 works
    can_frame frame{};
    frame.can_id = setActiveCalibrarion_;
    frame.can_dlc = 0x01;
    frame.data[0] = defaultCalibration_;
    int len = socket_->write(&frame);
    if (len != frameDataSize_) {
        logger_->critical("CAN not write to CAN");
        return false;
    }

    // Empty out the cansocket buffer, you can not be sure who left what on it
    while (true) {
        if (socket_->read(&frame) != frameDataSize_) {
            break;
        }
    }

    // Get the coefficient matrix
    if (!readCoefficientMatrix()) {
        logger_->debug("Failed to read coefficient matrix!");
        return false;
    }

    // Hot start the filter
    updateFTValue();
    filter_->resetEstimate(rawFt_);

    // Launch the processing thread
    runThread_ = true;
    readerThread_ = std::thread([this](){reader();});
    initialized_ = true;
    return true;
}

bool FTM115::deinitialize() {
    logger_->debug("stopMeasurementLoop()");
    if (!readerThread_.joinable()) {
        logger_->warn("mainControlLoop was not started");
        return false;
    }
    runThread_ = false;
    readerThread_.join();
    if (!socket_->deinitialize()) {
        logger_->error("Denitialization of CAN socket failed");
        return false;
    }
    initialized_ = false;
    return true;
}

crf::utility::types::TaskForceTorque FTM115::getFT() {
    logger_->debug("getFT");
    if (!initialized_) {
        logger_->debug("FTM115 was not initialized");
    }
    return ft_;
}

crf::utility::types::TaskForceTorque FTM115::getRawFT() {
    logger_->debug("getRawFT");
    if (!initialized_) {
        logger_->debug("FTM115 was not initialized");
    }
    return rawFt_;
}

crf::utility::types::TaskForceTorque FTM115::getFTGravityFree(
    const crf::utility::types::TaskPose& sensorPosition,
    bool inWorldCoordinateSystem) {
    if (!calibrated_) {
        logger_->debug("You need to calibrate the sensor to use getFTGravityFree()");
        return crf::utility::types::TaskForceTorque();
    }

    crf::utility::types::TaskForceTorque gravityFree = ft_;
    Eigen::Matrix3d orientation(sensorPosition.getRotationMatrix());
    Eigen::MatrixXd rotation = Eigen::MatrixXd::Zero(6, 6);
    rotation.block<3, 3>(0, 0) = orientation;
    rotation.block<3, 3>(3, 3) = orientation;

    Eigen::Vector3d gravityEndFrame = orientation.transpose()*Eigen::Vector3d(0, 0, weight_);
    Eigen::Vector3d gravityTorque = Eigen::Vector3d(0, 0, torque_/weight_).cross(gravityEndFrame);

    Eigen::VectorXd FT(6);
    for (int j = 0; j < 6; j++) {
        FT(j) = ft_[j];
    }
    FT.head(3) +=  gravityEndFrame;
    FT.tail(3) +=  gravityTorque;

    if (inWorldCoordinateSystem) {
        Eigen::VectorXd FTRotated = rotation * FT;
        for (int j = 0; j < 6; j++) {
            gravityFree[j] = FTRotated(j);
        }
    } else {
        for (int j = 0; j < 6; j++) {
            gravityFree[j] = FT(j);
        }
    }
    return gravityFree;
}

bool FTM115::updateBias(const crf::utility::types::TaskPose& sensorPosition,
    const std::string& logFilePath) {
    if (!readerThread_.joinable()) {
        logger_->warn("Need to start sensor, before updating bias");
        return false;
    }

    // Load calibration file
    std::ifstream inCalibFile;
    inCalibFile.open(logFilePath);
    if (!inCalibFile.is_open()) {
        logger_->debug("Calibration file not found, maybe you need to run the sensor calibration "
            "algorithm first");
    }
    nlohmann::json json;
    try {
        json = nlohmann::json::parse(inCalibFile);
    } catch (const nlohmann::detail::parse_error& e) {
        logger_->debug("Parse error in {0}", logFilePath);
        return false;
    }

    weight_ = json["GravityCompensation"]["weight"];
    torque_ = json["GravityCompensation"]["torque"];
    // Get a measurement if thread is not runnning, otherwise we already have one
    if (!runThread_) {
        updateFTValue();
    }

    Eigen::Matrix3d orientation(sensorPosition.getRotationMatrix());
    Eigen::MatrixXd rotation = Eigen::MatrixXd::Zero(6, 6);
    rotation.block<3, 3>(0, 0) = orientation;
    rotation.block<3, 3>(3, 3) = orientation;

    Eigen::Vector3d gravityEndFrame = orientation.transpose()*Eigen::Vector3d(0, 0, weight_);
    double momentArm = torque_ / weight_;
    if (weight_ == 0) {
        momentArm = 0;
    }
    Eigen::Vector3d gravityTorque = Eigen::Vector3d(0, 0, momentArm).cross(gravityEndFrame);

    Eigen::VectorXd FT(6);
    FT << 0, 0, 0, 0, 0, 0;
    for (int j = 0; j < 6; j++) {
        FT(j) = ft_[j];
    }
    FT.head(3) +=  gravityEndFrame;
    FT.tail(3) +=  gravityTorque;

    for (int j = 0; j < 6; j++) {
        biases_[j] = FT(j);
    }

    // We need to reset the filter, because the values would jump after substracting the bias
    filter_->resetEstimate(crf::utility::types::TaskForceTorque(rawFt_.raw() - biases_.raw()));
    calibrated_ = true;
    return true;
}

bool FTM115::isCalibrated() {
    return calibrated_;
}

bool FTM115::readCoefficientMatrix() {
    can_frame frame{};
    frame.can_id = readCoefficientMatrix_;
    frame.can_dlc = 0x01;
    for (int i = 0; i < 6; i++) {
        frame.data[0] = i;
        int len = socket_->write(&frame);
        if (len != frameDataSize_) {
            logger_->debug("CAN not write to CAN");
            return false;
        }

        can_frame read_frame[3]{};
        for (unsigned int j = 0; j < 3; j++) {
            socket_->read(&read_frame[j]);
            if (read_frame[j].can_id != (readCoefficientMatrix_ + j)) {
                logger_->debug("Could not recieve Coefficient Vector");
                return false;
            }
        }

        char temp1[4];
        char temp2[4];
        for (int k = 0; k < 3; k++) {
            for (int j = 0; j < 4; j++) {
                std::memcpy(&temp1[j], &read_frame[k].data[3-j], sizeof(char));
                std::memcpy(&temp2[j], &read_frame[k].data[7-j], sizeof(char));
            }
            std::memcpy(&coefficientMatrix_(i, k*2), &temp1, sizeof(float));
            std::memcpy(&coefficientMatrix_(i, k*2 +1), &temp2, sizeof(float));
        }
    }
    return true;
}

bool FTM115::updateFTValue() {
    Eigen::VectorXi ft(6);
    ft << 0, 0, 0, 0, 0, 0;
    can_frame frame{};
    frame.can_id = readFTData_;
    if (socket_->write(&frame) != frameDataSize_) {
        logger_->debug("Unable to write to CAN");
        return false;
    }

    // Read out the raw sensor values from the sensor
    can_frame recv_frame{};
    for (int i = 0; i < 2; i++) {
        socket_->read(&recv_frame);
        if (recv_frame.can_id == readFTData_) {
            ft(0) = static_cast<int16_t >(recv_frame.data[2] * 256 + recv_frame.data[3]);
            ft(2) = static_cast<int16_t>(recv_frame.data[4] * 256 + recv_frame.data[5]);
            ft(4) = static_cast<int16_t>(recv_frame.data[6] * 256 + recv_frame.data[7]);
        } else if (recv_frame.can_id == static_cast<unsigned int>(readFTData_ + 1)) {
            ft(1) = static_cast<int16_t>(recv_frame.data[0] * 256 +  recv_frame.data[1]);
            ft(3) = static_cast<int16_t>(recv_frame.data[2] * 256 +  recv_frame.data[3]);
            ft(5) = static_cast<int16_t>(recv_frame.data[4] * 256 +  recv_frame.data[5]);
        } else {
            i--;
        }
    }

    // Convert sensor measurements to forces and torques
    Eigen::VectorXf raw_ft = coefficientMatrix_ * ft.cast<float>();

    crf::utility::types::TaskForceTorque temp({0, 0, 0, 0, 0, 0});
    for (int j = 0; j < 6; j++) {
        temp[j] = raw_ft(j) / measurementCountPerUnit_;
    }

    // There is an offset between the Schunk Sensor and Schunk Arm default coordinates
    // You need to rotate the sensor measurements to align them with the arm
    Eigen::Matrix3d rotationMatrix =
        SchunkMechanicalOffset_.getHomogeneousTransformationMatrix().block<3, 3>(0, 0);
    Eigen::Vector3d forces(temp[0], temp[1], temp[2]);
    Eigen::Vector3d torques(temp[3], temp[4], temp[5]);
    Eigen::Vector3d forces_rotated = rotationMatrix*forces;
    Eigen::Vector3d torques_rotated = rotationMatrix*torques;
    temp = crf::utility::types::TaskForceTorque(
        {forces_rotated(0), forces_rotated(1), forces_rotated(2),
        torques_rotated(0), torques_rotated(1), torques_rotated(2)});
    rawFt_ = temp;
    // if calibrated we know the bias, lets get rid of it
    if (calibrated_) {
        temp = crf::utility::types::TaskForceTorque(temp.raw() - biases_.raw());
    }

    ft_ = filter_->updateEstimation(temp);
    return true;
}

bool FTM115::reader() {
    while (runThread_) {
        updateFTValue();
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepDurationMilliseconds_));
    }
    return true;
}

}  // namespace ftsensor
}  // namespace sensors
}  // namespace crf
