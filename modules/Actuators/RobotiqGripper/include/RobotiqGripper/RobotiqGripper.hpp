#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "Gripper/IGripper.hpp"
#include "EventLogger/EventLogger.hpp"

#include <modbus/modbus.h>
#include <string>
#include <memory>

namespace crf {
namespace robots {
namespace robotiqgripper {

class RobotiqGripper: public gripper::IGripper {
 public:
    RobotiqGripper() = delete;
    explicit RobotiqGripper(const std::string& devAddress);
    RobotiqGripper(const RobotiqGripper& other) = delete;
    RobotiqGripper(RobotiqGripper&& other) = delete;
    ~RobotiqGripper() override;

    bool initialize() override;
    bool deinitialize() override;

    boost::optional<float> getPosition() override;
    bool isGrasping() override;

    bool setPosition(float percentage) override;
    bool setPosition(GripperState state) override;

    // Sets the grasping force for the next movements
    bool setGraspingForce(float percentage) override;
    // stops the movement of the gripper
    bool stopGripper() override;

    // the gripper will start moving immediately with given speed, until it fully closes/opens
    // negative percentage opens, positive closes gripper
    bool setVelocity(float percentage) override;

 private:
    utility::logger::EventLogger logger_;
    std::string deviceAddress_;
    bool initialized_;
    // shared_ptr instead of unique_ptr because of incomplete type
    std::shared_ptr<modbus_t> bus_;

    const int gripperInputRegister;
    const int gripperOutputRegister;

    // GripperState
    bool readGripperState();
    bool gripperActivated_;
    bool objectDetected_;
    uint8_t posRequestEcho_;
    uint8_t actualPosition_;
    uint8_t current_;

    // Target Values
    bool writeGripperState(uint8_t targetPosition, uint8_t targetSpeed);
    const uint8_t defaultVelocity;
    uint8_t targetForce_;
};

}  // namespace robotiqgripper
}  // namespace robots
}  // namespace crf
