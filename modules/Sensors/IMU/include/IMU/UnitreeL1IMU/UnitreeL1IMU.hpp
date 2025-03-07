#pragma once

#include <array>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <Eigen/Dense> // For calibration data

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunication.hpp"
#include "IMU/IIMU.hpp"
#include "unilidar_sdk/unitree_lidar_sdk.h"

namespace crf {
namespace sensors {
namespace imu {

enum class LaserConnectionType {
    Serial = 0,
    Ethernet = 1
};

class UnitreeL1IMU : public IIMU {
public:
    UnitreeL1IMU(LaserConnectionType connectionType, const std::string& deviceOrAdress,
                 unsigned int baudrateOrPort);
    UnitreeL1IMU() = delete;
    UnitreeL1IMU(const UnitreeL1IMU&) = delete;
    UnitreeL1IMU(UnitreeL1IMU&&) = delete;

    ~UnitreeL1IMU() override;

    bool initialize() override;
    bool deinitialize() override;
    IMUSignals getSignal() override;
    crf::expected<bool> calibrate() override;

private:
    LaserConnectionType connectionType_;
    std::string deviceOrAdress_;
    unsigned int baudrateOrPort_;
    crf::utility::logger::EventLogger logger_;
    bool initialized_;
    std::unique_ptr<unitree_lidar_sdk::UnitreeLidarReader> lreader_;
    IMUSignals imuData_;
    Eigen::Vector3d initialAccBias_;
    Eigen::Vector3d initialGyrBias_;
    Eigen::Vector3d previousAcc_;  // For low-pass filtering
    Eigen::Matrix3d gravityAlignmentRotation_;  // Gravity alignment matrix

    // Calibration data
    Eigen::Vector4d quatBias_;
    std::vector<Eigen::Quaterniond> quatData_;

    // Private methods for calibration
    void collectCalibrationData(int samples);
    void calculateQuaternionCalibration();

    bool writeMsgToDevice(const std::string& msg);
    void initializeIMU(int sampleCount);
    Eigen::Vector3d lowPassFilter(const Eigen::Vector3d& current, const Eigen::Vector3d& previous, double alpha);
    void alignGravity(const Eigen::Vector3d& measuredGravity);

};

} // namespace imu
} // namespace sensors
} // namespace crf
