#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <atomic>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "RobotArm/IRobotArm.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "SchunkPowerCube/SchunkPowerCubeConfiguration.hpp"
#include "SchunkPowerCube/SchunkPowerCubeDevice.hpp"

namespace crf {
namespace robots {
namespace schunkpowercube {

class SchunkPowerCube : public robotarm::IRobotArm {
 public:
    SchunkPowerCube() = delete;
    SchunkPowerCube(const SchunkPowerCube&) = delete;
    SchunkPowerCube(SchunkPowerCube&&) = delete;
    SchunkPowerCube(std::shared_ptr<communication::cansocket::ICANSocket>,
        const std::string&) = delete;
    SchunkPowerCube(std::shared_ptr<communication::cansocket::ICANSocket> socket,
        const nlohmann::json& robotConfigFileName);
    ~SchunkPowerCube() override;

    bool initialize() override;
    bool deinitialize() override;

    boost::optional<crf::utility::types::JointPositions> getJointPositions() override;
    boost::optional<crf::utility::types::JointVelocities> getJointVelocities() override;
    boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques() override;
    boost::optional<crf::utility::types::TaskPose> getTaskPose() override;
    boost::optional<crf::utility::types::TaskVelocity> getTaskVelocity() override;
    boost::optional<crf::utility::types::TaskForceTorque> getTaskForceTorque() override;

    bool setJointPositions(const crf::utility::types::JointPositions& jointPositions) override;
    bool setJointVelocities(const crf::utility::types::JointVelocities& jointVelocities) override;
    bool setJointForceTorques(const crf::utility::types::JointForceTorques& jointForceTorques) override; // NOLINT
    bool setTaskPose(const crf::utility::types::TaskPose& position) override;
    bool setTaskVelocity(
        const crf::utility::types::TaskVelocity& velocity, bool TCP) override;
    bool stopArm() override;
    bool enableBrakes() override;
    bool disableBrakes() override;
    std::shared_ptr<robotarm::RobotArmConfiguration> getConfiguration() override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    nlohmann::json robotConfigFileName_;
    std::shared_ptr<SchunkPowerCubeConfiguration> configuration_;
    std::vector<std::shared_ptr<SchunkPowerCubeDevice> > motors_;

    std::vector<bool> brakesStatus_;
    std::vector<bool> movingStatus_;
    std::vector<bool> errorStatus_;
    std::vector<bool> hardwareOkStatus_;

    utility::types::JointPositions jointPositions_;
    utility::types::JointVelocities jointVelocities_;
    crf::utility::types::JointForceTorques jointForceTorques_;
    std::vector<float> jointsOffset_;

    std::map<int, int> jointCanIDCorrespondance_;

    bool initialized_;
    bool brakesEnabled_;

    std::atomic<bool> stopThread_;
    std::thread canReceiverThread_;
    std::thread canSenderThread_;
    void canReceiver();
    std::chrono::milliseconds updateFrequency_;
    void canSender();

    std::map<char, std::function<bool(SchunkPowerCube*, const can_frame&, int)> > framesHandlers_;
    bool msgStatusHandler(const can_frame& frame, int jointNumber);
    bool msgPositionHandler(const can_frame& frame, int jointNumber);
    bool msgVelocityHandler(const can_frame& frame, int jointNumber);
    bool msgCurrentHandler(const can_frame& frame, int jointNumber);
    bool msgSetAckHandler(const can_frame& frame, int jointNumber);
};

}  // namespace schunkpowercube
}  // namespace robots
}  // namespace crf
