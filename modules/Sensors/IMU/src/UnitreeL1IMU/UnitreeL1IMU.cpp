/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Dadi Hrannar Davidsson CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */


#include <unitree_lidar_sdk.h>

#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>
#include <boost/mpl/aux_/na_spec.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <fstream>
#include <unistd.h>  // For usleep


#include "IMU/IIMU.hpp"
#include "IMU/UnitreeL1IMU/UnitreeL1IMU.hpp"
#include "VisionUtility/PointCloud/PointConverter.hpp"





namespace crf {
namespace sensors {
namespace imu {



// Constructor
UnitreeL1IMU::UnitreeL1IMU(LaserConnectionType connectionType, const std::string& deviceOrAdress,
                           unsigned int baudrateOrPort)
    : connectionType_(connectionType),
      deviceOrAdress_(deviceOrAdress),
      baudrateOrPort_(baudrateOrPort),
      logger_("UnitreeL1"),
      initialized_(false),
      quatBias_(Eigen::Vector4d::Zero()) {
    logger_->debug("Constructor called");
}

// Destructor
UnitreeL1IMU::~UnitreeL1IMU() {
    logger_->debug("Destructor called");
    deinitialize();
}   

bool UnitreeL1IMU::initialize() {
    lreader_ = std::unique_ptr<unitree_lidar_sdk::UnitreeLidarReader>(unitree_lidar_sdk::createUnitreeLidarReader());
    logger_->debug("Initialize called");

    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    int cloud_scan_num = 18;  // Set the cloud scan number, adjust if necessary
    if (lreader_->initialize(cloud_scan_num, deviceOrAdress_)) {
        logger_->error("Failed to initialize Unitree Lidar");
        return false;
    } else {
        logger_->info("Lidar initialized successfully");
    }
    initialized_ = true;
    // Enhanced IMU initialization with averaging
    initializeIMU(100);  // Collect 100 samples for initialization

    
    return true;
}


// Deinitialize the Lidar
bool UnitreeL1IMU::deinitialize() {
    logger_->debug("Deinitialize called");
    if (!initialized_) {
        logger_->warn("Lidar not initialized");
        return false;
    }

    // Set the Lidar to STANDBY mode and stop data capture
    lreader_->setLidarWorkingMode(unitree_lidar_sdk::STANDBY);
    initialized_ = false;
    return true;
}


void UnitreeL1IMU::initializeIMU(int sampleCount) {
    Eigen::Vector3d accSum(0, 0, 0);
    Eigen::Vector3d gyrSum(0, 0, 0);

    // Collect multiple samples to get a more accurate initial bias estimate
    for (int i = 0; i < sampleCount; ++i) {
        IMUSignals imuSignals = getSignal();
        if (imuSignals.linearAcceleration == crf::Code::NotInitialized) {
            logger_->error("IMU data not available during initialization.");
            return;  // Remove the "false" and just return
        }

        // Collect data for averaging
        accSum += Eigen::Vector3d(
            imuSignals.linearAcceleration.value()[0],
            imuSignals.linearAcceleration.value()[1],
            imuSignals.linearAcceleration.value()[2]
        );
        gyrSum += Eigen::Vector3d(
            imuSignals.angularVelocity.value()[0],
            imuSignals.angularVelocity.value()[1],
            imuSignals.angularVelocity.value()[2]
        );

        usleep(1000);  // Adjust as needed for sampling rate
    }

    // Compute and store average bias values
    initialAccBias_ = accSum / sampleCount;
    initialGyrBias_ = gyrSum / sampleCount;
    // Apply gravity alignment
    alignGravity(initialAccBias_);
}

Eigen::Vector3d UnitreeL1IMU::lowPassFilter(const Eigen::Vector3d& current, const Eigen::Vector3d& previous, double alpha) {
    // Tune the alpha value for more stability, e.g., from 0.1 to 0.05 for a stronger filter
    return alpha * current + (1.0 - alpha) * previous;
}



void UnitreeL1IMU::alignGravity(const Eigen::Vector3d& measuredGravity) {
    // Align the measured gravity with expected gravity to ensure accurate orientation initialization
    Eigen::Vector3d expectedGravity(0, 0, -1.0);

    Eigen::Matrix3d hatGrav;
    hatGrav << 0.0, measuredGravity(2), -measuredGravity(1),
              -measuredGravity(2), 0.0, measuredGravity(0),
              measuredGravity(1), -measuredGravity(0), 0.0;

    double alignCos = expectedGravity.dot(measuredGravity) / (expectedGravity.norm() * measuredGravity.norm());
    if (std::abs(alignCos) > 1.0) alignCos = std::copysign(1.0, alignCos);

    Eigen::Vector3d alignAngle = hatGrav * measuredGravity / (hatGrav * measuredGravity).norm() * std::acos(alignCos);
    gravityAlignmentRotation_ = Eigen::AngleAxisd(alignAngle.norm(), alignAngle.normalized()).toRotationMatrix();
}




IMUSignals UnitreeL1IMU::getSignal() {
    if (!initialized_) {
        logger_->error("Device not initialized");
        IMUSignals output;
        output.position = crf::Code::NotInitialized;
        output.linearVelocity = crf::Code::NotInitialized;
        output.angularVelocity = crf::Code::NotInitialized;
        output.linearAcceleration = crf::Code::NotInitialized;
        output.angularAcceleration = crf::Code::NotInitialized;
        output.magneticField = crf::Code::NotInitialized;
        return output;
    }

    // Poll for data directly from the Unitree Lidar SDK
    unitree_lidar_sdk::MessageType result;
    while (true) {
        result = lreader_->runParse();  // Polling for data

        if (result == unitree_lidar_sdk::IMU) {
            unitree_lidar_sdk::IMUUnitree imuUnitree = lreader_->getIMU();

            IMUSignals output;
            output.quaternion = std::array<double, 4>();
            output.angularVelocity = std::array<double, 3>();
            output.linearAcceleration = std::array<double, 3>();

            auto& angularVelocityArray = output.angularVelocity.value();
            auto& linearAccelerationArray = output.linearAcceleration.value();
            auto& quaternionArray = output.quaternion.value();

            // Retrieve and filter acceleration data
            Eigen::Vector3d currentAcc(
                imuUnitree.linear_acceleration[0],
                imuUnitree.linear_acceleration[1],
                imuUnitree.linear_acceleration[2]
            );

            Eigen::Vector3d filteredAcc = lowPassFilter(currentAcc, previousAcc_, 0.1);
            previousAcc_ = filteredAcc;

             // Retrieve and store quaternion data
            for (int i = 0; i < 4; ++i) {
                quaternionArray[i] = static_cast<double>(imuUnitree.quaternion[i]);
            }


            // Store filtered acceleration
            for (int i = 0; i < 3; ++i) {
                linearAccelerationArray[i] = filteredAcc[i];
            }

            // Retrieve raw gyroscope data
            Eigen::Vector3d rawGyro(
                imuUnitree.angular_velocity[0],
                imuUnitree.angular_velocity[1],
                imuUnitree.angular_velocity[2]
            );

            // Apply gyroscope bias correction
            Eigen::Vector3d correctedGyro = rawGyro - initialGyrBias_;

            // Store corrected angular velocity
            for (int i = 0; i < 3; ++i) {
                angularVelocityArray[i] = correctedGyro[i];
            }

            // Similar filtering can be applied to angular velocity if needed
            return output;
        }

        usleep(500);  // Sleep between polling
    }
}




// Integrated Calibration Method
crf::expected<bool> UnitreeL1IMU::calibrate() {
    logger_->debug("calibrate");
    if (!initialized_) {
        logger_->error("Device not cannot be calibrated, as it is not initalized");
        return crf::Code::NotInitialized;
    }

    logger_->info("Starting calibration...");
    collectCalibrationData(100);  // Adjust sample count as needed
    calculateQuaternionCalibration();
    logger_->info("Calibration completed.");

    return true;
}

// Private Methods
void UnitreeL1IMU::collectCalibrationData(int samples) {
    for (int i = 0; i < samples; ++i) {
        IMUSignals imuSignals = getSignal();

        Eigen::Quaterniond quat(
            imuSignals.quaternion.value()[0],
            imuSignals.quaternion.value()[1],
            imuSignals.quaternion.value()[2],
            imuSignals.quaternion.value()[3]
        );

        quatData_.push_back(quat);
        usleep(1000);  // Adjust sampling rate as needed
    }
}

void UnitreeL1IMU::calculateQuaternionCalibration() {
    Eigen::Quaterniond avgQuat(0, 0, 0, 0);
    for (const auto& quat : quatData_) {
        avgQuat.w() += quat.w();
        avgQuat.x() += quat.x();
        avgQuat.y() += quat.y();
        avgQuat.z() += quat.z();
    }
    avgQuat.w() /= quatData_.size();
    avgQuat.x() /= quatData_.size();
    avgQuat.y() /= quatData_.size();
    avgQuat.z() /= quatData_.size();
    avgQuat.normalize();

    quatBias_[0] = avgQuat.w();
    quatBias_[1] = avgQuat.x();
    quatBias_[2] = avgQuat.y();
    quatBias_[3] = avgQuat.z();
}

} // namespace imu
} // namespace sensors
} // namespace crf












