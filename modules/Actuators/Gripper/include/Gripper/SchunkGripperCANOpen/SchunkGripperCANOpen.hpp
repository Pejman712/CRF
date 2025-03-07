/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thomas Breant CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#pragma once

#include "Gripper/IGripper.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "CANOpenDevices/CANOpenMotors/ERB.hpp"
#include "CANOpenDevices/CANOpenContext.hpp"

#include "EventLogger/EventLogger.hpp"
#include <boost/optional.hpp>
#include <string>
#include <memory>

namespace crf {
namespace actuators {
namespace gripper {

// The unit system of SchunkGripperCANOpen device is opposite of the interface
// inside the function the units are changed to match the convention of the interface

class SchunkGripperCANOpen: public gripper::IGripper {
 public:
    SchunkGripperCANOpen() = delete;
    explicit SchunkGripperCANOpen(std::shared_ptr<communication::cansocket::ICANSocket> sock,
        std::shared_ptr<crf::devices::canopendevices::CANOpenContext> ctx);
    SchunkGripperCANOpen(const SchunkGripperCANOpen& other) = delete;
    SchunkGripperCANOpen(SchunkGripperCANOpen&& other) = delete;
    ~SchunkGripperCANOpen() override;

    bool initialize() override;
    bool deinitialize() override;
    boost::optional<float> getPosition() override;
    bool isGrasping() override;
    bool setPosition(float percentage) override;
    bool setPosition(float position, float velocity);
    bool setPosition(GripperState state) override;
    bool setGraspingForce(float percentage) override;
    bool stopGripper() override;
    bool setVelocity(float percentage) override;

 private:
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    std::shared_ptr<crf::devices::canopendevices::CANOpenContext> ctx_;

    bool initialized_;
    int defaultGripperVelocity_;
    float gripperMaxPosition_;
    float gripperMaxVelocity_;
    float targetVelocity_;
    int gripperDirection_;
    std::shared_ptr<crf::devices::canopendevices::ERB> gripperDevice_;

    // Under what percentage the gripper is considered closed
    const float gripperOpenThreshold_ = 2;
    const float gripperClosedThreshold_ = 100 - gripperOpenThreshold_;

    utility::logger::EventLogger logger_;
};

}  // namespace gripper
}  // namespace actuators
}  // namespace crf
