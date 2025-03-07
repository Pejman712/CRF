#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <signal.h>
#include <Eigen/Dense>
#include "EventLogger/EventLogger.hpp"
#include "IMU/UnitreeL1IMU/UnitreeL1IMU.hpp"

bool ctrl_c_pressed = false;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

// Function to normalize angles in radians to 0-2π
double normalizeAngleRadians(double angle) {
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return angle;
}

// Declare radiansToDegrees function before quaternionToEulerAngles
Eigen::Vector3d radiansToDegrees(const Eigen::Vector3d& eulerRadians) {
    return eulerRadians * (180.0 / M_PI);
}

Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& q, bool toDegrees = false) {
    // Convert quaternion to rotation matrix and then to Euler angles (in radians) using ZYX order
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order

    // Debug print before normalization
    std::cout << "Raw Euler Angles (Radians): Roll: " << euler[0] 
              << ", Pitch: " << euler[1] 
              << ", Yaw: " << euler[2] << std::endl;

    // Normalize each angle to the 0-2π range
    euler[0] = normalizeAngleRadians(euler[0]); // Roll
    euler[1] = normalizeAngleRadians(euler[1]); // Pitch
    euler[2] = normalizeAngleRadians(euler[2]); // Yaw
    
    // Debug print after normalization
    std::cout << "Normalized Euler Angles (Radians): Roll: " << euler[0] 
              << ", Pitch: " << euler[1] 
              << ", Yaw: " << euler[2] << std::endl;

    // Convert to degrees if requested
    if (toDegrees) {
        euler = radiansToDegrees(euler);
    }

    return euler;
}


int main(int argc, char** argv) {
    // Open a file to log the quaternion data
    std::ofstream logFile("quaternion_log.csv");
    if (!logFile.is_open()) {
        std::cerr << "Failed to open log file for writing." << std::endl;
        return -1;
    }
    // Write the CSV header
    logFile << "w,x,y,z\n";

    // Create an instance of the UnitreeL1IMU with Serial connection type
    std::unique_ptr<crf::sensors::imu::UnitreeL1IMU> imu {new crf::sensors::imu::UnitreeL1IMU(
        crf::sensors::imu::LaserConnectionType::Serial, "/dev/ttyUSB0", 2000000)};
    
    // Capture CTRL+C signal to allow a graceful shutdown
    signal(SIGINT, ctrlc);

    // Initialize the IMU
    if (!imu->initialize()) {
        std::cout << "Not possible to initialize the IMU" << std::endl;
        return -1;
    }
    std::cout << "IMU Sensor initialized" << std::endl;

    // Calibrate the IMU (optional, if calibration function is used)
    if (!imu->calibrate()) {
        std::cout << "Calibration failed" << std::endl;
        return -1;
    }

    // Main loop to read IMU data
    while (!ctrl_c_pressed) {
        auto start = std::chrono::system_clock::now();
        crf::sensors::imu::IMUSignals imuSignals = imu->getSignal();
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - start;
        std::cout << "Acquisition time: " << elapsedSeconds.count() << " s" << std::endl;

        // Print the IMU signals
        std::cout << "IMU Signals:" << std::endl;
        std::cout << "\tQuaternion: [" 
                  << imuSignals.quaternion.value()[0] << ", " 
                  << imuSignals.quaternion.value()[1] << ", " 
                  << imuSignals.quaternion.value()[2] << ", " 
                  << imuSignals.quaternion.value()[3] << "]" << std::endl;

        // Log the quaternion data to the file
        logFile << imuSignals.quaternion.value()[0] << ","
                << imuSignals.quaternion.value()[1] << ","
                << imuSignals.quaternion.value()[2] << ","
                << imuSignals.quaternion.value()[3] << "\n";

        std::cout << "\tAngular Velocity: [" 
                  << imuSignals.angularVelocity.value()[0] << ", " 
                  << imuSignals.angularVelocity.value()[1] << ", " 
                  << imuSignals.angularVelocity.value()[2] << "]" << std::endl;

        std::cout << "\tLinear Acceleration (Filtered): [" 
                  << imuSignals.linearAcceleration.value()[0] << ", " 
                  << imuSignals.linearAcceleration.value()[1] << ", " 
                  << imuSignals.linearAcceleration.value()[2] << "]" << std::endl;

        // Convert the quaternion to an Eigen::Quaterniond
        Eigen::Quaterniond quat(
            imuSignals.quaternion.value()[0],  // w
            imuSignals.quaternion.value()[1],  // x
            imuSignals.quaternion.value()[2],  // y
            imuSignals.quaternion.value()[3]   // z
        );

        double quatNorm = quat.norm();
        if (std::abs(quatNorm - 1.0) > 1e-6) {
            std::cout << "Warning: Quaternion norm is " << quatNorm << ", normalizing." << std::endl;
            quat.normalize();
        }
        
        bool useDegrees = true;  // Set to false if you want radians

        // Get Euler angles with the option to convert to degrees
        Eigen::Vector3d eulerAngles = quaternionToEulerAngles(quat, useDegrees);

        // Print the Euler angles
        std::cout << "\tEuler Angles (Degrees): Roll: " << eulerAngles[0]
                  << ", Pitch: " << eulerAngles[1]
                  << ", Yaw: " << eulerAngles[2] << std::endl;

        // Sleep for a short interval before the next read
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Close the log file
    logFile.close();

    // Deinitialize the IMU
    if (!imu->deinitialize()) {
        std::cout << "Not possible to deinitialize the IMU" << std::endl;
        return -1;
    }

    return 0;
}
