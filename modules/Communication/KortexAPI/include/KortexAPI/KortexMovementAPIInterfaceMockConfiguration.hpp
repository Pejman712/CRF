/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <regex>
#include <string>
#include <fstream>
#include <vector>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "KortexAPI/KortexMovementAPIInterfaceMock.hpp"
#include "Robot/KinovaGen3/KinovaGen3.hpp"
#include "KortexAPI/KortexMovementAPIInterface.hpp"
#include "KortexAPI/IKortexMovementAPIInterface.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;

using crf::actuators::robot::KinovaGen3;
using crf::communication::kortexapi::KortexMovementAPIInterfaceMock;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

namespace crf::communication::kortexapi {

class KortexMovementAPIInterfaceMockConfiguration : public KortexMovementAPIInterfaceMock {
 public:
    KortexMovementAPIInterfaceMockConfiguration():
        logger_("KortexMovementAPIInterfaceMockConfiguration") {
            logger_->info("CTor");
        }

    ~KortexMovementAPIInterfaceMockConfiguration() {
        logger_->info("DTor");
    }

    void configureMock() {
        ON_CALL(*this, connect(_, _, _)).
            WillByDefault(Invoke([this](std::string, uint32_t, uint32_t) {return true;}));
        ON_CALL(*this, disconnect()).
            WillByDefault(Invoke([this](){}));
        ON_CALL(*this, set_username(_)).
            WillByDefault(Invoke([this](const char* value){}));
        ON_CALL(*this, set_password(_)).
            WillByDefault(Invoke([this](const char* value){}));
        ON_CALL(*this, set_session_inactivity_timeout(_)).
            WillByDefault(Invoke([this](::google::protobuf::uint32 value){}));
        ON_CALL(*this, set_connection_inactivity_timeout(_)).
            WillByDefault(Invoke([this](::google::protobuf::uint32 value){}));
        ON_CALL(*this, CreateSession()).
            WillByDefault(Invoke([this](){}));
        ON_CALL(*this, CloseSession()).
            WillByDefault(Invoke([this](){}));
        ON_CALL(*this, SetActivationStatus(_)).
            WillByDefault(Invoke([this](bool isActive){}));
        ON_CALL(*this, RefreshFeedback(_, _)).
            WillByDefault(Invoke(returnEmptyRefreshFeedbackClass));
        ON_CALL(*this, base_feedback(_, _)).
            WillByDefault(Invoke(returnEmptybase_feedbackClass));
        ON_CALL(*this, actuator_feedback(_)).
            WillByDefault(Invoke(returnEmptyActuatorFeedbackClass));
        ON_CALL(*this, set_high_level_velocity(_, _)).
            WillByDefault(Invoke([this](float value, int index){}));
        ON_CALL(*this, clear_velocity_high()).
            WillByDefault(Invoke([this](){}));
        ON_CALL(*this, send_command_velocity()).
            WillByDefault(Invoke([this](){}));
        ON_CALL(*this, set_high_level_position(_, _)).
            WillByDefault(Invoke([this](const std::vector<double>& jointPositions, int Dof){}));
        ON_CALL(*this, send_command_velocity()).
            WillByDefault(Invoke([this](){}));
    }

    static Kinova::Api::BaseCyclic::Feedback returnEmptyRefreshFeedbackClass(
        uint32_t deviceId, const Kinova::Api::RouterClientSendOptions& options) {
        Kinova::Api::BaseCyclic::Feedback* empty_RefreshFeedback;
        empty_RefreshFeedback = new Kinova::Api::BaseCyclic::Feedback();
        return *empty_RefreshFeedback;
    }

    static ::Kinova::Api::BaseCyclic::BaseFeedback returnEmptybase_feedbackClass(
        uint32_t deviceId, const Kinova::Api::RouterClientSendOptions& options) {
        ::Kinova::Api::BaseCyclic::BaseFeedback* empty_base_feedback;
        empty_base_feedback = new ::Kinova::Api::BaseCyclic::BaseFeedback();
        return *empty_base_feedback;
    }

    static ::Kinova::Api::BaseCyclic::ActuatorFeedback returnEmptyActuatorFeedbackClass(
        int index) {
        ::Kinova::Api::BaseCyclic::ActuatorFeedback* empty_ActuatorFeedback;
        empty_ActuatorFeedback = new ::Kinova::Api::BaseCyclic::ActuatorFeedback();
        return *empty_ActuatorFeedback;
    }

 private:
    crf::utility::logger::EventLogger logger_;
};
}  // namespace crf::communication::kortexapi
