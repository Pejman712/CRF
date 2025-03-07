/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#pragma once

#include <iostream>
#include <stdio.h>
#include <thread>
#include <memory>
#include <string>

#include <CANSocket/ICANSocket.hpp>
#include <FTSensor/IFTSensor.hpp>
#include <KalmanFilter/SimplifiedFTKalmanFilter.hpp>
#include <Eigen/Eigen>

namespace crf {
namespace sensors {
namespace ftsensor {

/*
 * @brief Applies kalman filter to the measurements
 */
class FTM115 : public IFTSensor {
 public:
    FTM115() = delete;
    explicit FTM115(std::shared_ptr<communication::cansocket::ICANSocket> socket);
    ~FTM115() override;

    bool initialize() override;
    bool deinitialize() override;

    crf::utility::types::TaskForceTorque getFT() override;
    crf::utility::types::TaskForceTorque getRawFT() override;
    crf::utility::types::TaskForceTorque getFTGravityFree(
        const crf::utility::types::TaskPose& sensorPosition,
        bool inWorldCoordinateSystem = false) override;
    bool updateBias(const crf::utility::types::TaskPose& sensorPosition,
        const std::string& logFilePath) override;
    bool isCalibrated() override;

 private:
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    utility::logger::EventLogger logger_;
    bool initialized_;
    crf::utility::types::TaskForceTorque ft_;
    crf::utility::types::TaskForceTorque rawFt_;
    bool calibrated_;
    float weight_;
    float torque_;
    std::thread readerThread_;
    Eigen::MatrixXf coefficientMatrix_;
    crf::utility::types::TaskForceTorque biases_;
    bool runThread_;
    std::unique_ptr<crf::math::kalmanfilter::SimplifiedFTKalmanFilter> filter_;

    const uint16_t setActiveCalibrarion_ = 0x206;
    const uint16_t readCoefficientMatrix_ = 0x202;
    const uint16_t readFTData_ = 0x200;
    const uint8_t defaultCalibration_ = 0x00;
    const unsigned int sleepDurationMilliseconds_ = 0;
    const unsigned int measurementCountPerUnit_ = 1000000;
    const int frameDataSize_ = 16;
    const utility::types::TaskPose SchunkMechanicalOffset_ =
        utility::types::TaskPose(
            {0, 0, 0}, crf::math::rotation::CardanXYZ({0, 0, 2.322}));
    const crf::utility::types::TaskForceTorque processNoise_ =
        crf::utility::types::TaskForceTorque({1E-3, 1E-3, 1E-3, 1E-3, 1E-3, 1E-3});
    const crf::utility::types::TaskForceTorque measurementNoise_ =
        crf::utility::types::TaskForceTorque({9.0, 9.0, 9.0, 0.3, 0.3, 0.3});

    /*
     * @brief This is the calibration matrix which was provided by the manufacturer, it is coded
     *        into the sensor and needs to be read at every startup. The size is 6x6.
     */
    bool readCoefficientMatrix();
    /*
     * @brief Queries the sensor and updates the measurement value.
     */
    bool updateFTValue();
    bool reader();
};

}  // namespace ftsensor
}  // namespace sensors
}  // namespace crf
