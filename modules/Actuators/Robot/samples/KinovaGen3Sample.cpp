/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Robot/KinovaGen3/KinovaGen3.hpp"
#include "KortexAPI/KortexMovementAPIInterface.hpp"

#include <math.h>
#include <time.h>


int main(int argc, char**argv) {
    std::cout << "Connecting... " << std::endl;
    std::shared_ptr<crf::communication::kortexapi::KortexMovementAPIInterface> kinovainterface =
        std::make_shared<crf::communication::kortexapi::KortexMovementAPIInterface>();
    std::ifstream robotConfigFilePath(argv[1]);
    crf::actuators::robot::KinovaGen3 kinovag3{
        kinovainterface,
        crf::actuators::robot::KinovaGen3Configuration(nlohmann::json::parse(robotConfigFilePath))};

    if (kinovag3.initialize()) {
        // get all information from robots
        auto joint_position = kinovag3.getJointPositions();
        auto joint_velocity = kinovag3.getJointVelocities();
        auto joint_torque = kinovag3.getJointForceTorques();
        auto task_position = kinovag3.getTaskPose();
        auto task_velocity = kinovag3.getTaskVelocity();
        auto task_force = kinovag3.getTaskForceTorque();

        std::cout << "joint_position: ";
        std::cout << joint_position.value() << std::endl;
        std::cout << "joint_velocity: ";
        std::cout << joint_velocity.value() << std::endl;
        std::cout << "joint_torque: ";
        std::cout << joint_torque.value() << std::endl;
        std::cout << "task_position: ";
        std::cout << task_position.value() << std::endl;
        std::cout << "task_velocity: ";
        std::cout << task_velocity.value() << std::endl;
        std::cout << "task_force: ";
        std::cout << task_force.value() << std::endl;

        // set velocity in high level control
        auto joint_set_velocity = crf::utility::types::JointVelocities(
            {0, 0, 0, 0, 0, 0.2});
        kinovag3.setJointVelocities(true, joint_set_velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // set position in high level control
        auto joint_set_position1 = crf::utility::types::JointPositions(
            {joint_position.value()[0]-0.3, joint_position.value()[1]-0.2,
            joint_position.value()[2], joint_position.value()[3],
            joint_position.value()[4], joint_position.value()[5]-0.8});
        kinovag3.setJointPositions(true, joint_set_position1);
        auto joint_set_position2 = crf::utility::types::JointPositions(
            {joint_position.value()[0]-0.3, joint_position.value()[1]-0.1,
            joint_position.value()[2]-0.3, joint_position.value()[3]+0.2,
            joint_position.value()[4]-0.8, joint_position.value()[5]-0.8});
        kinovag3.setJointPositions(true, joint_set_position2);
    }
}
