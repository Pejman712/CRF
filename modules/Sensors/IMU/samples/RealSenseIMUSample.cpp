#include <iostream>
#include <memory>
#include <csignal>
#include <chrono>
#include <thread>
#include "IMU/RealSenseIMU/RealSenseIMU.hpp"

bool ctrl_c_pressed = false;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char** argv) {
    // Set the serial number of the RealSense device (adjust as needed)
    std::string serialNumber = "147322072674";

    // Create an instance of RealSenseIMU
    std::unique_ptr<crf::sensors::imu::RealSenseIMU> imu {new crf::sensors::imu::RealSenseIMU(serialNumber)};

    // Capture CTRL+C signal for graceful shutdown
    signal(SIGINT, ctrlc);

    // Initialize the IMU
    if (!imu->initialize()) {
        std::cerr << "Failed to initialize RealSense IMU" << std::endl;
        return -1;
    }

    // Calibrate the IMU to remove bias and set up filters
    if (!imu->calibrate()) {
        std::cerr << "IMU calibration failed" << std::endl;
        return -1;
    }
    std::cout << "RealSense IMU calibration complete" << std::endl;
    std::cout << "RealSense IMU initialized successfully" << std::endl;

    // Main loop to read IMU data
    while (!ctrl_c_pressed) {
        // Fetch the IMU signals
        crf::sensors::imu::IMUSignals imuSignals = imu->getSignal();

        // Check and process angular velocity data
        if (imuSignals.angularVelocity) { 
            const std::array<double, 3>& angularVelocity = imuSignals.angularVelocity.value();
            std::cout << "Angular Velocity: ["
                      << angularVelocity[0] << ", "
                      << angularVelocity[1] << ", "
                      << angularVelocity[2] << "]" << std::endl;
        }

        // Check and process linear acceleration data
        if (imuSignals.linearAcceleration) { 
            const std::array<double, 3>& linearAcceleration = imuSignals.linearAcceleration.value();
            std::cout << "Linear Acceleration: ["
                      << linearAcceleration[0] << ", "
                      << linearAcceleration[1] << ", "
                      << linearAcceleration[2] << "]" << std::endl;
        }

        //Check and process posistion moved
        if (imuSignals.position) {
            double rad2degree = 1;// 180/M_PI;
            const std::array<double, 3>& position = imuSignals.position.value();
            std::cout << "Position: Roll: ["
                      << position[0] * rad2degree << ", Pitch: "
                      << position[1] * rad2degree << ", Yaw: "
                      << position[2] * rad2degree << "]" << std::endl;
        }

        // Optional: Sleep if needed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Deinitialize the IMU
    imu->deinitialize();
    std::cout << "RealSense IMU deinitialized" << std::endl;

    return 0;
}
