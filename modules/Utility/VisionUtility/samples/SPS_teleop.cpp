 /*© Copyright CERN 2024.  All rights reserved.
 * This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pejman Habibiroudkenar & Dadi Hrannar Davidsson CERN EN/SMM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <random>
#include <chrono>
#include <thread>
#include <signal.h>
#include <csignal>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>

//#include <gmock/gmock.h>

#include <Eigen/Dense>
#include "EventLogger/EventLogger.hpp"

//Wheel Encoders
#include "SPSRobot/SPSRobot.hpp"
#include "RobotBase/EtherCATRobotBase.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>

#include "VisionUtility/PointCloud/Gicp.hpp"  

#include <sys/select.h>
//Initialize the Robot


using crf::actuators::robotbase::SPSRobot;



// Maximum number of queued inputs
const int MAX_QUEUE_SIZE = 5;

// Buffer to store inputs
std::queue<char> inputQueue;

// Set to store currently pressed keys
std::set<char> pressedKeys;

// Last pressed key to detect changes
char lastKeyPressed = '\0';





// Function to make terminal non-blocking
void setNonBlockingInput(bool enable) {
    static struct termios oldt;
    static int oldf;

    if (enable) {
        struct termios newt;

        // Get the terminal settings for stdin
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Disable canonical mode and echo
        newt.c_lflag &= ~(ICANON | ECHO);

        // Set the new settings immediately
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        // Get the current file status flags
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

        // Set the file descriptor to non-blocking
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    } else {
        // Restore the terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        // Restore the file status flags
        fcntl(STDIN_FILENO, F_SETFL, oldf);
    }
}




int getPressedKey() {
    fd_set fds;
    struct timeval tv;
    int ch;

    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    // Set timeout to zero for non-blocking
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    if (ret > 0) {
        ch = getchar();
        return ch;
    }
    return -1;
}



namespace po = boost::program_options;
bool ctrl_c_pressed = false;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("ethercat_port", po::value<std::string>(), "EtherCAT port name (e.g., enp2s0)")
        ("sps_config", po::value<std::string>(), "SPSRobot configuration file path");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const std::exception& e) {
        std::cout << "Error parsing command line arguments: " << e.what() << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    // Check for required arguments and handle them appropriately
    if (!vm.count("ethercat_port")) {
        std::cout << "Missing ethercat port argument" << std::endl << desc << std::endl;
        return -1;
    }
    if (!vm.count("sps_config")) {
        std::cout << "Missing SPSRobot configuration file argument" << std::endl << desc << std::endl;
        return -1;
    }

    std::string ethercat_port = vm["ethercat_port"].as<std::string>();
    std::string sps_config_path = vm["sps_config"].as<std::string>();
   // std::string device = vm["device"].as<std::string>();
  //  std::string unitree_config_path = vm["unitree_config"].as<std::string>();

    // Load SPSRobot configuration file
    std::ifstream sps_config_file(sps_config_path);
    if (!sps_config_file.is_open()) {
        std::cout << "Unable to open SPSRobot configuration file: " << sps_config_path << std::endl;
        return -1;
    }
    nlohmann::json sps_config;
    sps_config_file >> sps_config;

    // Initialize SPSRobot
    std::shared_ptr<SPSRobot> bot = std::make_shared<SPSRobot>(ethercat_port.c_str(), sps_config);
    if (!bot->initialize()) {
        std::cout << "Could not initialize the SPSRobot" << std::endl;
        return -1;
    }
    std::cout << "SPSRobot initialized successfully" << std::endl;


    // Enable non-blocking input
    float base_speed = 0.8;
    setNonBlockingInput(true);

    char input;

    std::array<double, 6> velocityArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


            // Enable non-blocking input
        setNonBlockingInput(true);
    while (!ctrl_c_pressed) {
    // Add a small sleep to prevent high CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Reset velocityArray to zero at the beginning of each loop
    velocityArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Read all available input
    int ch;
    char lastInput = '\0';
    while ((ch = getPressedKey()) != -1) {
        lastInput = static_cast<char>(ch);
    }

    // If we have any input, process the last one
    if (lastInput != '\0') {
        if (lastInput == 'c') {
            std::cout << "Quit Program" << std::endl;
            break;
        }

        // Update velocity based on input
        if (lastInput == 'w') {
            std::cout << "Move Forward" << std::endl;
            velocityArray[0] = base_speed;
            velocityArray[1] = 0.0;
            velocityArray[5] = 0.0;
        }
        else if (lastInput == 's') {
            std::cout << "Move Backward" << std::endl;
            velocityArray[0] = -base_speed;
            velocityArray[1] = 0.0;
            velocityArray[5] = 0.0;
        }
        else if (lastInput == 'a') {
            std::cout << "Move Left" << std::endl;
            velocityArray[0] = 0.0;
            velocityArray[1] = -base_speed;
            velocityArray[5] = 0.0;
        }
        else if (lastInput == 'd') {
            std::cout << "Move Right" << std::endl;
            velocityArray[0] = 0.0;
            velocityArray[1] = base_speed;
            velocityArray[5] = 0.0;
        }
        else if (lastInput == 'e') {
            std::cout << "Turning Right" << std::endl;
            velocityArray[0] = 0.0;
            velocityArray[1] = 0.0;
            velocityArray[5] = base_speed;
        }
        else if (lastInput == 'q') {
            std::cout << "Turning left" << std::endl;
            velocityArray[0] = 0.0;
            velocityArray[1] = 0.0;
            velocityArray[5] = -base_speed;
        }
        else if (lastInput == ' ') {
            std::cout << "Stop" << std::endl;
            // velocityArray is already zero
        }
    }

    // Set the robot velocity
    crf::utility::types::TaskVelocity robotVelocity(velocityArray);
    bot->setTaskVelocity(robotVelocity);


    // Check for key releases (no direct way, so we need to read all input)
    // For simplicity, we can clear the pressedKeys set after each loop iteration
    // and rely on continuous key presses to keep keys in the set
    pressedKeys.clear();

    
        
        
     
    }

    setNonBlockingInput(false);
    return 0;
}
