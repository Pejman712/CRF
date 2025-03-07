#include <iostream>
#include <string>
#include <memory>
#include <array>
#include <librealsense2/rs.hpp>
#include <cmath>
#include <mutex>
#include <chrono>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iomanip>  // For controlling precision
#include <nlohmann/json.hpp>
#include "EventLogger/EventLogger.hpp"
#include "IMU/RealSenseIMU/RealSenseIMU.hpp"

namespace crf {
namespace sensors {
namespace imu {

RealSenseIMU::RealSenseIMU(const std::string& serialNumber)
    : serialNumber_(serialNumber), logger_("RealSenseIMU"), initialized_(false) 
    {

    logger_->debug("Constructor called");

    // Initialize RealSense device
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    bool found = false;

    if (devices.size() == 0) {
        logger_->error("No RealSense cameras detected");
        throw std::runtime_error("No RealSense cameras detected!");
    }

    for (rs2::device device : devices) {
        std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (serial == serialNumber) {
            device_ = device;
            found = true;
            break;
        }
    }

    if (!found) {
        logger_->error("Device with serial number {} does not exist", serialNumber);
        throw std::runtime_error("Invalid serial number");
    }
}

RealSenseIMU::~RealSenseIMU() {
    deinitialize();
    logger_->info("RealSense IMU destroyed.");
}


bool RealSenseIMU::initialize() {
    if (initialized_) return true;

    logger_->info("Initializing RealSense IMU.");

    logger_->info("Reloading calibration data.");
    std::filesystem::path calibration_path = std::filesystem::current_path() /
        "../modules/Sensors/IMU/calibration/RealSenseIMUCalibration/calibration.json";
        calibration_path = calibration_path.lexically_normal();  // Clean up the path
    loadCalibration(calibration_path.string());


    config_.enable_device(serialNumber_);
    config_.disable_all_streams();
    config_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 100);
    config_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

    try {
        // Start the pipeline with the callback
        pipeline_.start(config_, [this](rs2::frame frame) {
            this->imu_callback(frame);
        });

        // Disable motion correction if supported
        for (auto&& sensor : pipeline_.get_active_profile().get_device().query_sensors()) {
            if (sensor.supports(RS2_OPTION_ENABLE_MOTION_CORRECTION)) {
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
            }
        }

        initialized_ = true;
        logger_->info("RealSense IMU initialized successfully.");
    } catch (const rs2::error& e) {
        logger_->error("Failed to start RealSense pipeline: {}", e.what());
        initialized_ = false;
    }

    return initialized_;
}


void RealSenseIMU::imu_callback(const rs2::frame& frame) {
    if (auto motion = frame.as<rs2::motion_frame>()) {
        // Constants for filters
        const double filter_alpha = 0.95;          // Low-pass filter coefficient
        //const double complementary_alpha = 0.8;   // Complementary filter coefficient

        auto profile = motion.get_profile().stream_type();
        double ts = motion.get_timestamp();  // Timestamp in milliseconds

        if (profile == RS2_STREAM_GYRO) {
            auto gyro_data = motion.get_motion_data();

            if (firstGyroReading_) {
                firstGyroReading_ = false;
                last_ts_gyro_ = ts;
                return;
            }

            // Bias-corrected gyro data
            double corrected_gyro_x = gyro_data.x - gyroOffset_[0];
            double corrected_gyro_y = gyro_data.y - gyroOffset_[1];
            double corrected_gyro_z = gyro_data.z - gyroOffset_[2];

            // Low-pass filter for angular velocity
            std::array<double, 3> filteredAngularVelocity = {
                filter_alpha * corrected_gyro_x + (1 - filter_alpha) * prevAngularVelocity[0],
                filter_alpha * corrected_gyro_y + (1 - filter_alpha) * prevAngularVelocity[1],
                filter_alpha * corrected_gyro_z + (1 - filter_alpha) * prevAngularVelocity[2]
            };
            prevAngularVelocity = filteredAngularVelocity;

            // Calculate delta time for integration (convert ms to seconds)
            double dt_gyro = (ts - last_ts_gyro_) / 1000.0;
            if (dt_gyro <= 0 || dt_gyro > 0.1) {  // Assuming IMU data rates higher than 10 Hz
                dt_gyro = 0.01;  // Default to 10 ms
            }
            last_ts_gyro_ = ts;

            // Integrate gyro data for orientation (theta)
            theta[0] += filteredAngularVelocity[0] * dt_gyro;  // Roll (X-axis)
            theta[1] += filteredAngularVelocity[1] * dt_gyro;  // Pitch (Y-axis)
            theta[2] += filteredAngularVelocity[2] * dt_gyro;  // Yaw (Z-axis)

            // Normalize angles to [-pi, pi]
            for (int i = 0; i < 3; ++i) {
                if (theta[i] > M_PI) {
                    theta[i] -= 2 * M_PI;
                } else if (theta[i] < -M_PI) {
                    theta[i] += 2 * M_PI;
                }
            }

            // Lock and push the data into the queue
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (!imu_data_queue_.empty()) {
                    imu_data_queue_.back().angularVelocity = filteredAngularVelocity;
                    imu_data_queue_.back().position = theta;
                } else {
                    IMUSignals imuSignals;
                    imuSignals.angularVelocity = filteredAngularVelocity;
                    imuSignals.position = theta;
                    imu_data_queue_.push(imuSignals);
                }
            }
            data_cv_.notify_one();

            // Debug output
            /*std::cout << "Gyro Update - Theta (rad): roll=" << theta[0]
                      << ", pitch=" << theta[1]
                      << ", yaw=" << theta[2] << std::endl;*/

        } else if (profile == RS2_STREAM_ACCEL) {
            auto accel_data = motion.get_motion_data();

            // Apply calibration scale and bias
            double corrected_accel_x = accelScale_[0] * accel_data.x + accelScale_[1] * accel_data.y + accelScale_[2] * accel_data.z - accelOffset_[0];
            double corrected_accel_y = accelScale_[3] * accel_data.x + accelScale_[4] * accel_data.y + accelScale_[5] * accel_data.z - accelOffset_[1];
            double corrected_accel_z = accelScale_[6] * accel_data.x + accelScale_[7] * accel_data.y + accelScale_[8] * accel_data.z - accelOffset_[2];

            // Low-pass filter for linear acceleration
            std::array<double, 3> filteredLinearAcceleration = {
                filter_alpha * corrected_accel_x + (1 - filter_alpha) * prevLinearAcceleration[0],
                filter_alpha * corrected_accel_y + (1 - filter_alpha) * prevLinearAcceleration[1],
                filter_alpha * corrected_accel_z + (1 - filter_alpha) * prevLinearAcceleration[2]
            };
            prevLinearAcceleration = filteredLinearAcceleration;

            // Calculate acceleration-based angles
            /*double accel_angle_roll = atan2(filteredLinearAcceleration[1],
                                            filteredLinearAcceleration[2]);  // Roll angle from Y and Z
            double accel_angle_pitch = atan2(-filteredLinearAcceleration[0],
                                             sqrt(pow(filteredLinearAcceleration[1], 2) + pow(filteredLinearAcceleration[2], 2)));  // Pitch angle
            // Note: Yaw cannot be determined from accelerometer alone without magnetometer data*/

            // Apply complementary filter to fuse gyro and accelerometer for orientation
            //theta[0] = complementary_alpha * theta[0] + (1 - complementary_alpha) * accel_angle_roll;
            ///theta[1] = complementary_alpha * theta[1] + (1 - complementary_alpha) * accel_angle_pitch;
            // theta[2] remains unchanged in accelerometer update

            // Normalize angles to [-pi, pi]
            /*for (int i = 0; i < 2; ++i) {  // Only normalize roll and pitch
                if (theta[i] > M_PI) {
                    theta[i] -= 2 * M_PI;
                } else if (theta[i] < -M_PI) {
                    theta[i] += 2 * M_PI;
                }
            }*/

            // Lock and push the data into the queue
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (!imu_data_queue_.empty()) {
                    imu_data_queue_.back().linearAcceleration = filteredLinearAcceleration;
                    imu_data_queue_.back().position = theta;
                } else {
                    IMUSignals imuSignals;
                    imuSignals.linearAcceleration = filteredLinearAcceleration;
                    imuSignals.position = theta;
                    imu_data_queue_.push(imuSignals);
                }
            }
            data_cv_.notify_one();

            // Debug output
            /*std::cout << "Accel Update - Theta (rad): roll=" << theta[0]
                      << ", pitch=" << theta[1]
                      << ", yaw=" << theta[2] << std::endl;*/
        }
    }
}






void RealSenseIMU::loadCalibration(const std::string& filepath) {
    // Step 1: Try to open the file and check if it was successful
    std::ifstream file(filepath);
    if (!file.is_open()) {
        logger_->error("Failed to open calibration file: {}", filepath);
        std::cerr << "Error: Could not open file " << filepath << std::endl;
        return;
    }
    std::cout << "Successfully opened calibration file: " << filepath << std::endl;

    // Step 2: Parse the JSON and check for parsing errors
    nlohmann::json calibration_data;
    try {
        file >> calibration_data;
    } catch (const nlohmann::json::parse_error& e) {
        logger_->error("Failed to parse calibration file: {}", e.what());
        std::cerr << "Error: JSON parsing failed - " << e.what() << std::endl;
        return;
    }
    file.close();

    // Step 3: Print the parsed JSON to confirm structure and content
    std::cout << "Parsed calibration data:\n" << calibration_data.dump(4) << std::endl;

    // Step 4: Load accelerometer and gyroscope calibration data if they exist
    if (calibration_data.contains("imus") && calibration_data["imus"].is_array()) {
        auto& imu_data = calibration_data["imus"][0];
        if (imu_data.contains("accelerometer")) {
            auto& accelerometer_data = imu_data["accelerometer"];

            if (accelerometer_data.contains("bias")) {
                for (int i = 0; i < 3; ++i) {
                    accelOffset_[i] = accelerometer_data["bias"][i];
                }
                std::cout << "Accel Bias Loaded: x=" << accelOffset_[0] << ", y=" << accelOffset_[1] << ", z=" << accelOffset_[2] << std::endl;
            }

            if (accelerometer_data.contains("scale_and_alignment")) {
                for (int i = 0; i < 9; ++i) {
                    accelScale_[i] = accelerometer_data["scale_and_alignment"][i];
                }
                std::cout << "Accel Scale and Alignment Matrix Loaded: ";
                for (int i = 0; i < 9; ++i) {
                    std::cout << accelScale_[i] << " ";
                }
                std::cout << std::endl;
            }
        }

        if (imu_data.contains("gyroscope")) {
            auto& gyroscope_data = imu_data["gyroscope"];
            if (gyroscope_data.contains("bias")) {
                for (int i = 0; i < 3; ++i) {
                    gyroOffset_[i] = gyroscope_data["bias"][i];
                }
                std::cout << "Gyro Bias Loaded: x=" << gyroOffset_[0] << ", y=" << gyroOffset_[1] << ", z=" << gyroOffset_[2] << std::endl;
            }
        }
    } else {
        std::cerr << "Error: 'imus' array not found in calibration data." << std::endl;
    }
    
    logger_->info("Calibration data loaded successfully from {}", filepath);
}

bool RealSenseIMU::deinitialize() {
    if (!initialized_) return true;

    try {
        pipeline_.stop();
        initialized_ = false;
        logger_->info("RealSense IMU deinitialized successfully.");
    } catch (const std::exception& e) {
        logger_->error("Failed to deinitialize RealSense IMU: {}", e.what());
        return false;
    }
    return true;
}


crf::expected<bool> RealSenseIMU::calibrate() {
    logger_->info("Reloading calibration data.");
    std::filesystem::path calibration_path = std::filesystem::current_path() /
        "../modules/Sensors/IMU/calibration/RealSenseIMUCalibration/calibration.json";
        calibration_path = calibration_path.lexically_normal();  // Clean up the path
    loadCalibration(calibration_path.string());
    return true;
}


IMUSignals RealSenseIMU::getSignal() {
    std::unique_lock<std::mutex> lock(data_mutex_);

    // Wait until data is available
    data_cv_.wait(lock, [this]() { return !imu_data_queue_.empty(); });

    // Retrieve the next available IMU data
    IMUSignals imuSignals = imu_data_queue_.front();
    imu_data_queue_.pop();

    return imuSignals;
}





} // namespace imu
} // namespace sensors
} // namespace crf
