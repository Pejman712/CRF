/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include "TIM/TIMClient/TIMClient.hpp"

namespace crf::actuators::tim {

TIMClient::TIMClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds serverReplyTimeout,
    const float frequency,
    const uint32_t priority):
    crf::utility::devicemanager::PriorityAccessClient(
        socket,
        serverReplyTimeout,
        frequency,
        priority),
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    priority_(priority),
    logger_("TIMClient") {
    logger_->debug("CTor");

    receivers_.insert({"setCurrentPosition", &receiverSetCurrentPosition_});
    receivers_.insert({"setTargetPosition", &receiverSetTargetPosition_});
    receivers_.insert({"setTargetVelocity", &receiverSetTargetVelocity_});
    receivers_.insert({"moveToTarget", &receiverMoveToTarget_});
    receivers_.insert({"jog", &receiverJog_});
    receivers_.insert({"stop", &receiverStop_});
    receivers_.insert({"emergencyStop", &receiverEmergencyStop_});
    receivers_.insert({"extendChargingArm", &receiverExtendChargingArm_});
    receivers_.insert({"retractChargingArm", &receiverRetractChargingArm_});
    receivers_.insert({"startCharging", &receiverStartCharging_});
    receivers_.insert({"stopCharging", &receiverStopCharging_});
    receivers_.insert({"enableEconomyMode", &receiverEnableEconomyMode_});
    receivers_.insert({"disableEconomyMode", &receiverDisableEconomyMode_});
    receivers_.insert({"rebootRobotArmWagon", &receiverRebootRobotArmWagon_});
    receivers_.insert({"setObstacleArea_", &receiverSetObstacleArea_});
    receivers_.insert({"devicesRetracted", &receiverDevicesRetracted_});
    receivers_.insert({"allowMovement", &receiverAllowMovement_});
    receivers_.insert({"acknowledgeAlarms", &receiverAcknowledgeAlarms_});
}

TIMClient::~TIMClient() {
    logger_->debug("DTor");
    if (initialized_) deinitialize();
}

bool TIMClient::initialize() {
    return PriorityAccessClient::initialize();
}
bool TIMClient::deinitialize() {
    return PriorityAccessClient::deinitialize();
}

bool TIMClient::isConnected() {
    logger_->debug("isConnected");
    if (!requestStatus()) return false;
    return statusIsConnected_;
}

std::optional<bool> TIMClient::setCurrentPosition(const float &position) {
    logger_->debug("setCurrentPosition");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setCurrentPosition";
    json.data["position"] = position;
    json.data["priority"] = priority_;
    return sendPacket(json, receiverSetCurrentPosition_);
}

std::optional<bool> TIMClient::setTargetPosition(const float &position) {
    logger_->debug("setTargetPosition");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setTargetPosition";
    json.data["position"] = position;
    json.data["priority"] = priority_;
    return sendPacket(json, receiverSetTargetPosition_);
}

std::optional<bool> TIMClient::setTargetVelocity(const float &velocity) {
    logger_->debug("setTargetVelocity");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setTargetVelocity";
    json.data["velocity"] = velocity;
    json.data["priority"] = priority_;
    return sendPacket(json, receiverSetTargetVelocity_);
}

std::optional<bool> TIMClient::moveToTarget(const float &position, const float &velocity) {
    logger_->debug("moveToTarget");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "moveToTarget";
    json.data["position"] = position;
    json.data["velocity"] = velocity;
    json.data["priority"] = priority_;
    return sendPacket(json, receiverMoveToTarget_);
}

std::optional<bool> TIMClient::jog(const float &velocity) {
    logger_->debug("jog");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "jog";
    json.data["velocity"] = velocity;
    json.data["priority"] = priority_;
    return sendPacket(json, receiverJog_);
}


std::optional<float> TIMClient::getCurrentPosition() {
    logger_->debug("getCurrentPosition");
    if (!requestStatus()) return std::nullopt;
    return statusCurrentPosition_.load();
}

std::optional<float> TIMClient::getTargetPosition() {
    logger_->debug("getTargetPosition");
    if (!requestStatus()) return std::nullopt;
    return statusTargetPosition_.load();
}

std::optional<float> TIMClient::getCurrentVelocity() {
    logger_->debug("getCurrentVelocity");
    if (!requestStatus()) return std::nullopt;
    return statusCurrentVelocity_.load();
}

std::optional<float> TIMClient::getTargetVelocity() {
    logger_->debug("getTargetVelocity");
    if (!requestStatus()) return std::nullopt;
    return statusTargetVelocity_.load();
}

std::optional<float> TIMClient::getCurrentMaximumVelocity() {
    logger_->debug("getCurrentMaximumVelocity");
    if (!requestStatus()) return std::nullopt;
    return statusCurrentMaximumVelocity_.load();
}

std::optional<bool> TIMClient::isMoving() {
    logger_->debug("isMoving");
    if (!requestStatus()) return std::nullopt;
    return statusIsMoving_.load();
}

std::optional<bool> TIMClient::isTargetReached() {
    logger_->debug("isTargetReached");
    if (!requestStatus()) return std::nullopt;
    return statusIsTargetReached_.load();
}

std::optional<bool> TIMClient::stop() {
    logger_->debug("stop");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "stop";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverStop_);
}

std::optional<bool> TIMClient::emergencyStop() {
    logger_->debug("emergencyStop");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "emergencyStop";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverEmergencyStop_);
}

std::optional<bool> TIMClient::extendChargingArm() {
    logger_->debug("extendChargingArm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "extendChargingArm";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverExtendChargingArm_);
}

std::optional<bool> TIMClient::retractChargingArm() {
    logger_->debug("retractChargingArm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "retractChargingArm";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverRetractChargingArm_);
}

std::optional<bool> TIMClient::startCharging() {
    logger_->debug("startCharging");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "startCharging";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverStartCharging_);
}

std::optional<bool> TIMClient::stopCharging() {
    logger_->debug("stopCharging");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "stopCharging";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverStopCharging_);
}

std::optional<bool> TIMClient::isCharging() {
    logger_->debug("isCharging");
    if (!requestStatus()) return std::nullopt;
    return statusIsCharging_.load();
}

std::optional<float> TIMClient::getChargingCurrent() {
    logger_->debug("getChargingCurrent");
    if (!requestStatus()) return std::nullopt;
    return statusChargingCurrent_.load();
}

std::optional<float> TIMClient::getBatteryVoltage() {
    logger_->debug("getBatteryVoltage");
    if (!requestStatus()) return std::nullopt;
    return statusBatteryVoltage_.load();
}

std::optional<float> TIMClient::getBatteryCurrent() {
    logger_->debug("getBatteryCurrent");
    if (!requestStatus()) return std::nullopt;
    return statusBatteryCurrent_.load();
}

std::optional<bool> TIMClient::enableEconomyMode() {
    logger_->debug("enableEconomyMode");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "enableEconomyMode";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverEnableEconomyMode_);
}

std::optional<bool> TIMClient::disableEconomyMode() {
    logger_->debug("disableEconomyMode");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "disableEconomyMode";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverDisableEconomyMode_);
}

std::optional<bool> TIMClient::rebootRobotArmWagon() {
    logger_->debug("rebootRobotArmWagon");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "rebootRobotArmWagon";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverRebootRobotArmWagon_);
}

std::optional<bool> TIMClient::isInEconomy() {
    logger_->debug("isInEconomy");
    if (!requestStatus()) return std::nullopt;
    return statusIsInEconomy_.load();
}

std::array<LHCObstacle, 2> TIMClient::getClosestObstacleAreas() {
    logger_->debug("getClosestObstacleAreas");
    if (!requestStatus()) return std::array<LHCObstacle, 2> {LHCObstacle(), LHCObstacle()};
    std::scoped_lock<std::mutex> lck(mtx_);
    return statusClosestObstacleAreas_;
}

LHCObstacle TIMClient::getCurrentObstacleArea() {
    logger_->debug("getCurrentObstacleArea");
    if (!requestStatus()) return LHCObstacle();
    std::scoped_lock<std::mutex> lck(mtx_);
    return statusCurrentObstacleArea_;
}

std::optional<bool> TIMClient::setObstacleArea(const LHCObstacle &obstacle) {
    logger_->debug("getClosestObstacleAreas");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "devicesRetracted";
    json.data["priority"] = priority_;
    json.data["obstacle"]["identifier"] = obstacle.identifier();
    json.data["obstacle"]["type"] = obstacle.type();
    json.data["obstacle"]["startPosition"] = obstacle.startPosition();
    json.data["obstacle"]["endPosition"] = obstacle.endPosition();
    json.data["obstacle"]["maximumVelocity"] = obstacle.maximumVelocity();
    json.data["obstacle"]["mustRetractDevices"] = obstacle.mustRetractDevices();
    return sendPacket(json, receiverSetObstacleArea_);
}

std::optional<bool> TIMClient::devicesRetracted(bool deviceStatus) {
    logger_->debug("devicesRetracted");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "devicesRetracted";
    json.data["devicesStatus"] = deviceStatus;
    json.data["priority"] = priority_;
    return sendPacket(json, receiverDevicesRetracted_);
}

std::optional<bool> TIMClient::allowMovement(bool allow) {
    logger_->debug("allowMovement");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "allowMovement";
    json.data["allow"] = allow;
    json.data["priority"] = priority_;
    return sendPacket(json, receiverAllowMovement_);
}


std::optional<bool> TIMClient::devicesRetracted() {
    logger_->error("devicesRetracted");
    if (!requestStatus()) return std::nullopt;
    return statusDevicesRetracted_.load();
}

std::optional<bool> TIMClient::isFrontWarningFieldActive() {
    logger_->debug("isFrontWarningFieldActive");
    if (!requestStatus()) return std::nullopt;
    return statusIsFrontWarningFieldActive_;
}

std::optional<bool> TIMClient::isBackWarningFieldActive() {
    logger_->debug("isBackWarningFieldActive");
    if (!requestStatus()) return std::nullopt;
    return statusIsBackWarningFieldActive_.load();
}

std::optional<bool> TIMClient::isSafeToMove() {
    logger_->debug("isSafeToMove");
    if (!requestStatus()) return std::nullopt;
    return statusIsSafeToMove_;
}

TIMAlarms TIMClient::getAlarms() {
    logger_->debug("getAlarms");
    if (!requestStatus()) return TIMAlarms();
    std::scoped_lock<std::mutex> lck(mtx_);
    return statusAlarms_;
}

std::optional<bool> TIMClient::acknowledgeAlarms() {
    logger_->debug("moveWithDevicesDeployed");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "acknowledgeAlarms";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverAcknowledgeAlarms_);
}

std::shared_ptr<TIMConfiguration> TIMClient::getConfiguration() {
    logger_->debug("getConfiguration");
    if (!requestStatus()) return nullptr;
    std::scoped_lock<std::mutex> lck(mtx_);
    return statusConfiguration_;
}

// Private methods

void TIMClient::parseStatus(const nlohmann::json& json) {
    try {
        statusCurrentPosition_ = json["currentPosition"].get<std::optional<float>>();
        statusTargetPosition_ = json["targetPosition"].get<std::optional<float>>();
        statusCurrentVelocity_ = json["currentVelocity"].get<std::optional<float>>();
        statusTargetVelocity_ = json["targetVelocity"].get<std::optional<float>>();
        statusCurrentMaximumVelocity_ = json["currentMaximumVelocity"].get<std::optional<float>>();
        statusIsMoving_ = json["isMoving"].get<std::optional<bool>>();
        statusIsTargetReached_ = json["isTargetReached"].get<std::optional<bool>>();
        statusDevicesRetracted_ = json["devicesRetracted"].get<std::optional<bool>>();
        statusIsCharging_ = json["isCharging"].get<std::optional<bool>>();
        statusChargingCurrent_ = json["chargingCurrent"].get<std::optional<float>>();
        statusBatteryVoltage_ = json["batteryVoltage"].get<std::optional<float>>();
        statusBatteryCurrent_ = json["batteryCurrent"].get<std::optional<float>>();
        statusIsInEconomy_ = json["isInEconomy"].get<std::optional<bool>>();

        statusIsFrontWarningFieldActive_ =
            json["isFrontWarningFieldActive"].get<std::optional<bool>>();
        statusIsBackWarningFieldActive_ =
            json["isBackWarningFieldActive"].get<std::optional<bool>>();
        statusIsSafeToMove_ = json["isSafeToMove"].get<std::optional<bool>>();

        std::scoped_lock<std::mutex> lck(mtx_);
        if (json["previousObstacleArea"] != nullptr) {
            nlohmann::json obsJSON = json["previousObstacleArea"];
            statusClosestObstacleAreas_[0].identifier(obsJSON["identifier"]);
            statusClosestObstacleAreas_[0].type(obsJSON["type"]);
            statusClosestObstacleAreas_[0].startPosition(obsJSON["startPosition"]);
            statusClosestObstacleAreas_[0].endPosition(obsJSON["endPosition"]);
            statusClosestObstacleAreas_[0].maximumVelocity(obsJSON["maximumVelocity"]);
            statusClosestObstacleAreas_[0].mustRetractDevices(obsJSON["mustRetractDevices"]);
        } else {
            statusClosestObstacleAreas_[0] = LHCObstacle();
        }
        if (json["nextObstacleArea"] != nullptr) {
            nlohmann::json obsJSON = json["nextObstacleArea"];
            statusClosestObstacleAreas_[1].identifier(obsJSON["identifier"]);
            statusClosestObstacleAreas_[1].type(obsJSON["type"]);
            statusClosestObstacleAreas_[1].startPosition(obsJSON["startPosition"]);
            statusClosestObstacleAreas_[1].endPosition(obsJSON["endPosition"]);
            statusClosestObstacleAreas_[1].maximumVelocity(obsJSON["maximumVelocity"]);
            statusClosestObstacleAreas_[1].mustRetractDevices(obsJSON["mustRetractDevices"]);
        } else {
            statusClosestObstacleAreas_[1] = LHCObstacle();
        }
        if (json["currentObstacleArea"] != nullptr) {
            nlohmann::json obsJSON = json["currentObstacleArea"];
            statusCurrentObstacleArea_.identifier(obsJSON["identifier"]);
            statusCurrentObstacleArea_.type(obsJSON["type"]);
            statusCurrentObstacleArea_.startPosition(obsJSON["startPosition"]);
            statusCurrentObstacleArea_.endPosition(obsJSON["endPosition"]);
            statusCurrentObstacleArea_.maximumVelocity(obsJSON["maximumVelocity"]);
            statusCurrentObstacleArea_.mustRetractDevices(obsJSON["mustRetractDevices"]);
        } else {
            statusCurrentObstacleArea_ = LHCObstacle();
        }
        if (json["alarms"] != nullptr) {
            nlohmann::json alarmJSON = json["alarms"];
            statusAlarms_.barcodeReaderError(alarmJSON["barcodeReaderError"].get<bool>());
            statusAlarms_.batteryError(alarmJSON["batteryError"].get<bool>());
            statusAlarms_.chargingArmRequiresAcknowledgement(alarmJSON["chargingArmRequiresAcknowledgement"].get<bool>()); // NOLINT
            statusAlarms_.chargingArmMotorError(alarmJSON["chargingArmMotorError"].get<bool>()); // NOLINT
            statusAlarms_.frontBumperPressed(alarmJSON["frontBumperPressed"].get<bool>());
            statusAlarms_.frontLaserScannerError(alarmJSON["frontLaserScannerError"].get<bool>()); // NOLINT
            statusAlarms_.frontProtectiveFieldReading(alarmJSON["frontProtectiveFieldReading"].get<bool>()); // NOLINT
            statusAlarms_.backBumperPressed(alarmJSON["backBumperPressed"].get<bool>());
            statusAlarms_.backLaserScannerError(alarmJSON["backLaserScannerError"].get<bool>()); // NOLINT
            statusAlarms_.backProtectiveFieldReading(alarmJSON["backProtectiveFieldReading"].get<bool>()); // NOLINT
            statusAlarms_.mainMotorError(alarmJSON["mainMotorError"].get<bool>());
            statusAlarms_.mainMotorRequiresAcknowledgement(alarmJSON["mainMotorRequiresAcknowledgement"].get<bool>()); // NOLINT
            statusAlarms_.positionEncoderError(alarmJSON["positionEncoderError"].get<bool>());
            statusAlarms_.positionEncoderReadingError(alarmJSON["positionEncoderReadingError"].get<bool>()); // NOLINT
            statusAlarms_.velocityEncoderError(alarmJSON["velocityEncoderError"].get<bool>());
            statusAlarms_.velocityEncoderReadingError(alarmJSON["velocityEncoderReadingError"].get<bool>()); // NOLINT
            statusAlarms_.emergencyStop(alarmJSON["emergencyStop"].get<bool>());

            // Not implemented (makes no sense to send config all the time in streamer)
            // statusConfiguration_
        }
    } catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
    }
}

std::optional<bool> TIMClient::sendPacket(const communication::datapackets::JSONPacket& json,
    const Receiver<std::optional<bool>>& receiver) {
    logger_->debug("sendPacket: {}", json.data);
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::nullopt;
    }
    if (!lockControl()) {
        return std::nullopt;
    }
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), false);
    socketLock.unlock();

    std::optional<std::optional<bool>> result = receiver.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive answer for {}", json.data);
        return std::nullopt;
    }
    unlockControl();
    return result.value();
}

}  // namespace crf::actuators::tim
