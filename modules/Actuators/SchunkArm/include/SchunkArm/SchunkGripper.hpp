#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

// The unit system of SchunkGripper device is opposite of the interface
// inside the function the units are changed to match the convention of the interface

#include "Gripper/IGripper.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "SchunkArm/SchunkDevice.hpp"
#include "SchunkArm/SchunkCommands.hpp"

#include "EventLogger/EventLogger.hpp"
#include <boost/optional.hpp>
#include <string>
#include <memory>

namespace crf::actuators::schunkarm {

class SchunkGripper: public gripper::IGripper {
 public:
    SchunkGripper() = delete;
    explicit SchunkGripper(std::shared_ptr<communication::cansocket::ICANSocket> sock);
    SchunkGripper(const SchunkGripper& other) = delete;
    SchunkGripper(SchunkGripper&& other) = delete;
    ~SchunkGripper() = default;

    // Reads out the state of the motor
    bool initialize() override;
    bool deinitialize() override;

    // Position is between [0 - open, 100 - closed]
    boost::optional<float> getPosition() override;
    bool isGrasping() override;

    // 0% == open, 100% == closed
    bool setPosition(float percentage) override;
    bool setPosition(GripperState state) override;
    // you can set values from 0-100
    // the default value is 25%
    // if the value is too high, the gripper overheats
    bool setGraspingForce(float percentage) override;

    // puts on the break, should reduce the power consumption, avoids overheating
    bool stopGripper() override;
    // the gripper will start moving immediately with given speed, until it fully closes/opens
    // negative percentage opens, positive closes gripper
    bool setVelocity(float percentage) override;
    // updates the state of the gripper, handles errors during movement
    bool updateState(can_frame messageToGripper);

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    SchunkDevice gripperDevice_;
    bool initialized_;

    // the gripper state will be updated every given millisec
    const int getStateTimePeriodMilisec_;
    // in percentage of the max value
    const float defaultGripperVelocity_;
    // under what percentage the gripper is considered closed
    const float gripperOpenThreshold_;
    const float gripperClosedThreshold_;

    float gripperMaxPosition_;  // in radians
    // this is absolute value between [0,100]
    float targetVelocityPercentage_;
    int gripperDirection_;
};

}  // namespace crf::actuators::schunkarm
