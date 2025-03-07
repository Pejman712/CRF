#pragma once

#include <array>
#include <string>
#include <librealsense2/rs.hpp>
#include "EventLogger/EventLogger.hpp"
#include "IMU/IIMU.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>

namespace crf {
namespace sensors {
namespace imu {

class RealSenseIMU : public IIMU {
public:
    explicit RealSenseIMU(const std::string& serialNumber);
    ~RealSenseIMU() override;

    bool initialize() override;
    bool deinitialize() override;
    IMUSignals getSignal() override;
    crf::expected<bool> calibrate() override;

private:
    std::string serialNumber_;
    rs2::pipeline pipeline_;
    rs2::config config_;
    rs2::device device_;
    utility::logger::EventLogger logger_;
    bool initialized_;
    std::array<double, 3> gyroOffset_ = {0.0, 0.0, 0.0};
    std::array<double, 3> accelOffset_ = {0.0, 0.0, 0.0};
    std::array<double, 9> accelScale_ = {1.0, 0.0, 0.0,
                                         0.0, 1.0, 0.0,
                                         0.0, 0.0, 1.0};
    std::array<double, 3> prevAngularVelocity = {0.0, 0.0, 0.0};
    std::array<double, 3> prevLinearAcceleration = {0.0, 0.0, 0.0};
    std::array<double, 3> theta = {0.0, 0.0, 0.0};
    double alpha_ = 0.2;  // Default smoothing factor
    void loadCalibration(const std::string& filepath);
    bool firstGyroReading_ = true;
    double last_ts_gyro_ = 0.0;

    // Private members for callback and data handling
    void imu_callback(const rs2::frame& frame);
    std::mutex data_mutex_;
    std::condition_variable data_cv_;
    std::queue<IMUSignals> imu_data_queue_;
};

} // namespace imu
} // namespace sensors
} // namespace crf
