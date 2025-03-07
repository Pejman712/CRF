/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"

#define ARM_VELOCITY_CONTROL_DURATION std::chrono::seconds(5)
#define ARM_CONTROL_INTERVAL std::chrono::milliseconds(5)

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Too few arguments" << std::endl;
        std::cout << "[1] CONFIG_FILENAME " << std::endl;
        return -1;
    }

    crf::utility::logger::EventLogger logger("KinovaArmExample");

    /*
     * You can construct KinovaJaco with nullptr, default API interface
     * will be created in the constructor
     * You can also construct it explicitly:
     *    KinovaJaco arm(std::make_shared<KinovaApiInterface>());
     */
    // crf::actuators::kinovaarm::KinovaJaco arm(nullptr);
    std::ifstream robotData(argv[1]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    crf::actuators::kinovaarm::KinovaJaco arm(
        std::make_shared<crf::actuators::kinovaarm::KinovaApiInterface>(), robotJSON);
    logger->info("Going to initialize arm");
    logger->info("arm.initialize(): {}", arm.initialize());

    std::cout << "Home pos : " << arm.moveHomePosition() << std::endl;
    crf::utility::types::JointPositions jointPositions = arm.getJointPositions().get();
    jointPositions[5] = 0;
    std::cout << "Move pos : " << arm.setJointPositions(jointPositions) << std::endl;
    bool stop = false;
    while (!stop) {
        auto jointPositions2 = arm.getJointPositions().get();
        if (fabs(jointPositions2[2] - jointPositions[2]) < 0.01) {
            stop = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::chrono::seconds elapsed(0);
    logger->info("Going to controll arm in velocity mode for {} seconds",
        ARM_VELOCITY_CONTROL_DURATION.count());
    crf::utility::types::JointVelocities vel(6);
    vel[0] = 0.1;  // move first joint
    while (elapsed < ARM_VELOCITY_CONTROL_DURATION) {
        if (!arm.setJointVelocities(vel)) {
            logger->warn("Failed to setJointVelocities");
            return -1;
        }
        auto currentJointPositions = arm.getJointPositions();
        if (currentJointPositions) {
            logger->info("Position: {}", currentJointPositions.get());
        }
        std::this_thread::sleep_for(ARM_CONTROL_INTERVAL);
        elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - start);
    }
    logger->info("arm.deinitialize(): {}", arm.deinitialize());
    return 0;
}
