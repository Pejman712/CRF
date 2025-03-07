/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <optional>
#include <map>
#include <tuple>
#include <memory>
#include <chrono>
#include <iostream>
#include <vector>

#include <nlohmann/json.hpp>

#include "TIM/TIMS300/TIMS300.hpp"
#include "SiemensPLC/ISiemensPLC.hpp"
#include "SiemensPLC/SiemensPLCS7.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"
#include "TIM/TIMConfiguration.hpp"
#include "TIM/TIMAlarms.hpp"

namespace crf::actuators::tim {

TIMS300::TIMS300(const nlohmann::json& timConfigFile,
    std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc) :
    timConfigFile_(timConfigFile),
    plc_(plc),
    logger_("TIMS300"),
    isInitialized_(false),
    config_(new crf::actuators::tim::TIMConfiguration),
    commandTimeout_(),
    alarmsDB_(),
    commandsDB_(),
    settingsDB_(),
    statusDB_(),
    obstaclesList_(),
    updatePLCValuesThread_(),
    stopUpdatePCLValuesThread_(true),
    isTIMAlive_(false),
    firstGrabberLoopMutex_(),
    firstGrabberLoop_() {
    logger_->info("CTor");
}

TIMS300::~TIMS300() {
    logger_->info("DTor");
    deinitialize();
}

bool TIMS300::initialize() {
    logger_->info("initialize");
    if (isInitialized_) {
        logger_->warn("There is already a connection established with the PLC");
        return true;
    }
    if (!config_->parse(timConfigFile_)) {
        logger_->error("Failed to read the configuration file");
        return false;
    }
    commandTimeout_ = config_->getCommandTimeout();
    if (plc_ == nullptr) {
        plc_ = std::make_shared<crf::devices::siemensplc::SiemensPLCS7>(
            config_->getIPAddress(), config_->getRack(), config_->getSlot());
    }
    if (!plc_->initialize()) {
        logger_->error("Could not initialize the connection to the PLC");
        return false;
    }
    // We launch the grabber thread and wait for it to read once all the values of the PLC.
    stopUpdatePCLValuesThread_ = false;
    updatePLCValuesThread_ = std::thread(&TIMS300::grabber, this);
    std::unique_lock<std::mutex> lock(firstGrabberLoopMutex_);
    if (firstGrabberLoop_.wait_for(lock, 10*config_->getUpdateInterval()) ==
        std::cv_status::timeout) {
        logger_->error("The update of the PLC values took longer than expected");
        return false;
    }
    if (!saveObstaclesInformation()) {
        logger_->error("Could not get the obstacles information from the PLC");
        return false;
    }
    isInitialized_ = true;
    return true;
}

bool TIMS300::deinitialize() {
    logger_->info("deinitialize");
    if (!isInitialized_) {
        return true;
    }
    if (!isConnected()) {
        return false;
    }
    stopUpdatePCLValuesThread_ = true;
    if (updatePLCValuesThread_.joinable()) {
        updatePLCValuesThread_.join();
    }
    if (!plc_->deinitialize()) {
        logger_->error("Could not deinitialize connection to PLC");
        return false;
    }
    isInitialized_ = false;
    return true;
}

bool TIMS300::isConnected() {
    if (!isInitialized_) {
        logger_->error("The TIMS300 is not initialized");
        return false;
    }
    if (plc_ == nullptr) {
        logger_->error("Missing PLC object for connection");
        return false;
    }
    if (!plc_->isConnected()) {
        logger_->error("Not connected to PLC");
        return false;
    }
    if (!isTIMAlive_) {
        logger_->error("No hearbeat detected");
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::setCurrentPosition(const float &position) {
    logger_->debug("setCurrentPosition: {}", position);
    if (!isConnected()) {
        return std::nullopt;
    }
    if (position < config_->getTIMLimits().minimumPosition) {
        logger_->error("Requested position {} is smaller than the minimum allowed {}", position,
            config_->getTIMLimits().minimumPosition);
        return false;
    }
    if (position > config_->getTIMLimits().maximumPosition) {
        logger_->error("Requested position {} is bigger than the maximum allowed {}", position,
            config_->getTIMLimits().maximumPosition);
        return false;
    }
    if (!setFloatSetting("PositionSetManually", position - config_->getPositionOffset())) {
        return false;
    }
    if (!setBooleanCommand("SetPositionManually", true)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::setTargetPosition(const float &position) {
    logger_->debug("setTargetPosition");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (position < config_->getTIMLimits().minimumPosition) {
        logger_->error("Requested position {} is smaller than the minimum allowed {}", position,
            config_->getTIMLimits().minimumPosition);
        return false;
    }
    if (position > config_->getTIMLimits().maximumPosition) {
        logger_->error("Requested position {} is bigger than the maximum allowed {}", position,
            config_->getTIMLimits().maximumPosition);
        return false;
    }
    if (!setFloatSetting("TargetPosition", position - config_->getPositionOffset())) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::setTargetVelocity(const float &velocity) {
    logger_->debug("setTargetVelocity");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (velocity <= 0) {
        logger_->warn("Requested velocity is less than 0");
        return false;
    }
    if (velocity > config_->getTIMLimits().maximumVelocity) {
        logger_->warn("Requested velocity {} is bigger than the maximum allowed {}", velocity,
            config_->getTIMLimits().maximumVelocity);
        return false;
    }
    if (!setFloatSetting("TargetVelocity", velocity)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::moveToTarget(const float &position, const float &velocity) {
    logger_->debug("moveToTarget");
    if (!isConnected()) {
        return std::nullopt;
    }

    std::optional<bool> positionOutput = setTargetPosition(position);
    std::optional<bool> velocityOutput = setTargetVelocity(velocity);
    std::optional<bool> safeToMove = isSafeToMove();
    if (!positionOutput || !velocityOutput || !safeToMove) {
        return false;
    }
    if (!positionOutput.value() || !velocityOutput.value() || !safeToMove.value()) {
        return false;
    }

    if (!setBooleanCommand("MoveToTarget", true)) {
        return false;
    }
    logger_->info("Moving to target: position {}, velocity {}", position, velocity);

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < commandTimeout_) {
        end = std::chrono::high_resolution_clock::now();
        std::optional<bool> moving = isMoving();
        std::optional<bool> targetReached = isTargetReached();
        if (!moving && !targetReached) {
            continue;
        }
        if (moving.value() || targetReached.value()) {
            return true;
        }
        std::this_thread::sleep_for(config_->getUpdateInterval());
    }
    logger_->error("The movement didn't start after {} microseconds", commandTimeout_.count());
    return false;
}

std::optional<bool> TIMS300::jog(const float &velocity) {
    logger_->debug("jog");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (fabsf(velocity) > config_->getTIMLimits().maximumVelocity) {
        logger_->warn("Requested velocity {} is bigger than the maximum allowed {}", velocity,
            config_->getTIMLimits().maximumVelocity);
        return false;
    }
    if (!setFloatSetting("TargetVelocity", fabsf(velocity))) {
        return false;
    }
    std::string command;
    if (velocity > 0) {
        command = "JogForward";
    } else if (velocity < 0) {
        command = "JogBackward";
    } else {
        logger_->warn("Requested velocity is 0");
        return false;
    }
    return setBooleanCommand(command, true);
}

std::optional<float> TIMS300::getCurrentPosition() {
    logger_->debug("getCurrentPosition");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.timPosition() + config_->getPositionOffset();
}

std::optional<float> TIMS300::getTargetPosition() {
    logger_->debug("getTargetPosition");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return settingsDB_.targetPosition() + config_->getPositionOffset();
}

std::optional<float> TIMS300::getCurrentVelocity() {
    logger_->debug("getCurrentVelocity");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.timVelocity();
}

std::optional<float> TIMS300::getTargetVelocity() {
    logger_->debug("getTargetVelocity");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return settingsDB_.targetVelocity();
}

std::optional<float> TIMS300::getCurrentMaximumVelocity() {
    logger_->debug("getCurrentMaximumVelocity");
    LHCObstacle obstacle = getCurrentObstacleArea();
    if (obstacle.isEmpty()) {
        return config_->getTIMLimits().maximumVelocity;
    }
    return obstacle.maximumVelocity();
}

std::optional<bool> TIMS300::isMoving() {
    logger_->debug("isMoving");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return !statusDB_.timStopped();
}

std::optional<bool> TIMS300::isTargetReached() {
    logger_->debug("isTargetReached");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.targetReached();
}

std::optional<bool> TIMS300::stop() {
    logger_->debug("stop");
    if (!isConnected()) {
        return std::nullopt;
    }
    /*
     * It is necessary to check if the train is moving before sending stop. Otherwise,
     * the train will store the STOP until the next attempt to move.
     * (adiazros && jplayang)
     */
    if (!isMoving().value()) {
        return true;
    }
    if (!setBooleanCommand("Stop", true)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::emergencyStop() {
    logger_->debug("emergencyStop");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("EmergencyStop", true)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::extendChargingArm() {
    logger_->debug("extendChargingArm");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("ChargingArmManualControl", true)) {
        return false;
    }
    if (!setBooleanCommand("ExtendChargingArm", true)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::retractChargingArm() {
    logger_->debug("retractChargingArm");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("ChargingArmManualControl", true)) {
        return false;
    }
    if (!setBooleanCommand("RetractChargingArm", true)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::startCharging() {
    logger_->debug("startCharging");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("ChargingArmManualControl", false)) {
        return false;
    }
    if (!setBooleanCommand("StartCharging", true)) {
        return false;
    }
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < commandTimeout_) {
        end = std::chrono::high_resolution_clock::now();
        std::optional<bool> charging = isCharging();
        if (!charging) {
            continue;
        }
        if (charging.value()) {
            return true;
        }
        std::this_thread::sleep_for(config_->getUpdateInterval());
    }
    logger_->error("The charge didn't start after {} microseconds", commandTimeout_.count());
    return false;
}

std::optional<bool> TIMS300::stopCharging() {
    logger_->debug("stopCharging");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("ChargingArmManualControl", false)) {
        return false;
    }
    if (!setBooleanCommand("StopCharging", true)) {
        return false;
    }
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < commandTimeout_) {
        end = std::chrono::high_resolution_clock::now();
        std::optional<bool> charging = isCharging();
        if (!charging) {
            continue;
        }
        if (!charging.value() && statusDB_.chargingArmRetracted()) {
            return true;
        }
        std::this_thread::sleep_for(config_->getUpdateInterval());
    }
    logger_->error("The charge didn't stop after {} microseconds", commandTimeout_.count());
    return false;
}

std::optional<bool> TIMS300::isCharging() {
    logger_->debug("isCharging");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.charging() && statusDB_.chargingArmConnected();;
}

std::optional<float> TIMS300::getChargingCurrent() {
    logger_->debug("getChargingCurrent");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.chargingCurrent();
}

std::optional<float> TIMS300::getBatteryVoltage() {
    logger_->debug("getBatteryVoltage");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.batteryVoltage();
}

std::optional<float> TIMS300::getBatteryCurrent() {
    logger_->debug("getBatteryCurrent");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.batteryCurrent();
}

std::optional<bool> TIMS300::enableEconomyMode() {
    logger_->debug("enableEconomyMode");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("EnableEconomyMode", true)) {
        return false;
    }
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < commandTimeout_) {
        end = std::chrono::high_resolution_clock::now();
        std::optional<bool> inEconomy = isInEconomy();
        if (!inEconomy) {
            continue;
        }
        if (inEconomy.value()) {
            return true;
        }
        std::this_thread::sleep_for(config_->getUpdateInterval());
    }
    logger_->error("The economy mode was not enabled after {} seconds",
        commandTimeout_.count());
    return false;
}

std::optional<bool> TIMS300::disableEconomyMode() {
    logger_->debug("disableEconomyMode");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("DisableEconomyMode", true)) {
        return false;
    }
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < commandTimeout_) {
        end = std::chrono::high_resolution_clock::now();
        std::optional<bool> inEconomy = isInEconomy();
        if (!inEconomy) {
            continue;
        }
        if (!inEconomy.value()) {
            return true;
        }
        std::this_thread::sleep_for(config_->getUpdateInterval());
    }
    logger_->error("The economy mode was not disabled after {} seconds",
        commandTimeout_.count());
    return false;
}

std::optional<bool> TIMS300::isInEconomy() {
    logger_->debug("isInEconomy");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.economyMode();
}

std::optional<bool> TIMS300::rebootRobotArmWagon() {
    logger_->debug("rebootRobotArmWagon");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("RebootRobotArmWagon", true)) {
        return false;
    }
    return true;
}

std::array<LHCObstacle, 2> TIMS300::getClosestObstacleAreas() {
    logger_->debug("getClosestObstacleAreas");
    LHCObstacle obstacleBehind;
    LHCObstacle obstacleInFront;
    std::optional<float> position = getCurrentPosition();
    std::optional<float> velocity = getCurrentVelocity();
    std::optional<bool> moving = isMoving();
    if (!position || !velocity || !moving) {
        return std::array<LHCObstacle, 2>{};
    }
    for (const LHCObstacle &obstacle : obstaclesList_) {
        if (position.value() > obstacle.endPosition()) {
            obstacleBehind = obstacle;
        }
        if (position.value() < obstacle.startPosition()) {
            obstacleInFront = obstacle;
            break;
        }
    }
    if (velocity.value() < 0 && moving.value()) {
        return std::array<LHCObstacle, 2>{obstacleInFront, obstacleBehind};
    }
    return std::array<LHCObstacle, 2>{obstacleBehind, obstacleInFront};
}

LHCObstacle TIMS300::getCurrentObstacleArea() {
    logger_->debug("getCurrentObstacleArea");
    std::optional<float> position = getCurrentPosition();
    if (!position) {
        return LHCObstacle();
    }
    for (const LHCObstacle &obstacle : obstaclesList_) {
        if (position.value() >= obstacle.startPosition() &&
            position.value() <= obstacle.endPosition()) {
            return obstacle;
        }
    }
    return LHCObstacle();
}

std::optional<bool> TIMS300::setObstacleArea(const LHCObstacle &obstacle) {
    logger_->debug("setObstacleArea");
    if (!isConnected() || obstacle.isEmpty()) {
        return std::nullopt;
    }
    auto it = obstaclesList_.find(obstacle);
    if (it == obstaclesList_.end()) {
        logger_->error("Unknown obstacle - ID:{}", obstacle.identifier());
        return std::nullopt;
    }
    if (it->identifier() != obstacle.identifier() ||
        it->type() != obstacle.type() ||
        it->startPosition() != obstacle.startPosition() ||
        it->endPosition() != obstacle.endPosition() ||
        it->mustRetractDevices() != obstacle.mustRetractDevices()) {
        logger_->error("Only the maximum velocity can be change in the obstacles");
        return std::nullopt;
    }
    std::array<unsigned int, 2> location = config_->getSettingsVariablesLocation()["ObstacleID"];
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_INT, obstacle.identifier(),
        config_->getSettingsDBLocation()[0], location[0], location[1])) {
        logger_->error("Could not write the setting ObstacleID with {}", obstacle.identifier());
        return std::nullopt;
    }
    if (!setFloatSetting("ObstacleMaximumVelocity", fabsf(obstacle.maximumVelocity()))) {
        return std::nullopt;
    }
    if (!setBooleanCommand("SetObstacleMaximumVelocity", true)) {
        return std::nullopt;
    }
    return true;
}

std::optional<bool> TIMS300::devicesRetracted(bool deviceStatus) {
    logger_->debug("devicesRetracted: {}", deviceStatus);
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("DevicesRetracted", deviceStatus)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::devicesRetracted() {
    logger_->debug("devicesRetracted");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return commandsDB_.devicesRetracted();
}

std::optional<bool> TIMS300::isFrontWarningFieldActive() {
    logger_->debug("isFrontWarningFieldActive");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.frontWarningField();
}

std::optional<bool> TIMS300::isBackWarningFieldActive() {
    logger_->debug("isBackWarningFieldActive");
    if (!isConnected() || statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.backWarningField();
}

std::optional<bool> TIMS300::allowMovement(bool allow) {
    logger_->debug("allowMovement: {}", allow);
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("AllowMovement", allow)) {
        return false;
    }
    return true;
}

std::optional<bool> TIMS300::isSafeToMove() {
    logger_->debug("isSafeToMove");
    if (!isConnected() || statusDB_.isEmpty() || alarmsDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.safeToMove();
}

crf::actuators::tim::TIMAlarms TIMS300::getAlarms() {
    logger_->debug("getAlarms");
    if (!isConnected()) {
        return crf::actuators::tim::TIMAlarms();
    }
    return alarmsDB_;
}

std::optional<bool> TIMS300::acknowledgeAlarms() {
    logger_->debug("acknowledgeAlarms");
    if (!isConnected()) {
        return std::nullopt;
    }
    if (!setBooleanCommand("AcknowledgeAlarms", true)) {
        return false;
    }
    return true;
}

std::shared_ptr<TIMConfiguration> TIMS300::getConfiguration() {
    logger_->debug("getConfiguration");
    return config_;
}

bool TIMS300::saveObstaclesInformation() {
    logger_->debug("saveObstaclesInformation");
    if (!obstaclesList_.empty()) {
        logger_->debug("Obstacles list already saved");
        return true;
    }
    std::string buffer = plc_->readDB(config_->getObstacleDBLocation()[0],
        config_->getObstacleDBLocation()[1], 0);
    if (buffer.length() != config_->getObstacleDBLocation()[1]) {
        logger_->error("Failed to read Obstacles Datablock {} {}", buffer.length(),
            config_->getObstacleDBLocation()[1]);
        return false;
    }
    unsigned int obstaclesNumber = (config_->getObstacleDBLocation()[1]/
        config_->getObstacleDBLocation()[2]);
    for (unsigned int n = 0; n < obstaclesNumber; n++) {
        unsigned int dbPositon = n*config_->getObstacleDBLocation()[2];
        if (!devices::siemensplc::SiemensPLCTypeConverter::getBit(buffer,
            config_->getObstacleVariablesLocation()["Enabled"][1],
            config_->getObstacleVariablesLocation()["Enabled"][0] + dbPositon)) {
            logger_->info("Obstacle {} disabled", dbPositon);
            continue;
        }
        LHCObstacle obstacle;
        obstacle.identifier(n+1);
        obstacle.startPosition(devices::siemensplc::SiemensPLCTypeConverter::getFloat(buffer,
            config_->getObstacleVariablesLocation()["StartPosition"][0] + dbPositon) +
            config_->getPositionOffset());
        obstacle.endPosition(devices::siemensplc::SiemensPLCTypeConverter::getFloat(buffer,
            config_->getObstacleVariablesLocation()["EndPosition"][0] + dbPositon) +
            config_->getPositionOffset());
        obstacle.maximumVelocity(devices::siemensplc::SiemensPLCTypeConverter::getFloat(buffer,
            config_->getObstacleVariablesLocation()["MaximumVelocity"][0] + dbPositon));
        obstacle.mustRetractDevices(devices::siemensplc::SiemensPLCTypeConverter::getBit(buffer,
            config_->getObstacleVariablesLocation()["MustRetractDevices"][1],
            config_->getObstacleVariablesLocation()["MustRetractDevices"][0] + dbPositon));
        obstaclesList_.insert(obstacle);
        logger_->info("Obstacle {} [{}, {}] enabled", obstacle.identifier(),
            obstacle.startPosition(), obstacle.endPosition());
    }
    obstaclesList_.insert(LHCObstacle(0, LHCObstacleType::OuterLimits,
        config_->getTIMLimits().minimumPosition, config_->getTIMLimits().minimumPosition,
        0.3, true));
    obstaclesList_.insert(LHCObstacle(obstaclesNumber+1, LHCObstacleType::OuterLimits,
        config_->getTIMLimits().maximumPosition, config_->getTIMLimits().maximumPosition,
        0.3, true));
    if (obstaclesList_.empty()) {
        logger_->warn("No obstacles found");
        return true;
    }
    return true;
}

bool TIMS300::updatePLCValues() {
    logger_->debug("updatePLCValues");
    bool result = true;
    std::string buffer = plc_->readDB(config_->getAlarmDBLocation()[0],
        config_->getAlarmDBLocation()[1], 0);
    if (buffer.length() == config_->getAlarmDBLocation()[1]) {
        if (!alarmsDB_.parseSiemensPLCBuffer(buffer, config_->getAlarmVariablesLocation())) {
            logger_->warn("Failed to parse Alarms Datablock");
        }
    } else {
        logger_->warn("Failed to read Alarms Datablock");
        result = false;
    }
    buffer = plc_->readDB(config_->getCommandsDBLocation()[0],
        config_->getCommandsDBLocation()[1], 0);
    if (buffer.length() == config_->getCommandsDBLocation()[1]) {
        if (!commandsDB_.parseSiemensPLCBuffer(buffer, config_->getCommandsVariablesLocation())) {
            logger_->warn("Failed to parse Inputs Datablock");
        }
    } else {
        logger_->error("Failed to read Inputs Datablock");
        result = false;
    }
    buffer = plc_->readDB(config_->getSettingsDBLocation()[0],
        config_->getSettingsDBLocation()[1], 0);
    if (buffer.length() == config_->getSettingsDBLocation()[1]) {
        if (!settingsDB_.parseSiemensPLCBuffer(buffer, config_->getSettingsVariablesLocation())) {
            logger_->warn("Failed to parse Settings Datablock");
        }
    } else {
        logger_->error("Failed to read Settings Datablock");
        result = false;
    }
    buffer = plc_->readDB(config_->getStatusDBLocation()[0],
        config_->getStatusDBLocation()[1], 0);
    if (buffer.length() == config_->getStatusDBLocation()[1]) {
        if (!statusDB_.parseSiemensPLCBuffer(buffer, config_->getStatusVariablesLocation())) {
            logger_->warn("Failed to parse Status Datablock");
        }
    } else {
        logger_->error("Failed to read Status Datablock {} {}", buffer.length(),
            config_->getStatusDBLocation()[1]);
        result = false;
    }
    return result;
}

void TIMS300::grabber() {
    logger_->debug("grabber");
    int16_t previousTIMHeartbeat = -1;
    int16_t localHeartbeat = 0;
    isTIMAlive_ = true;
    while (!stopUpdatePCLValuesThread_) {
        std::chrono::time_point start = std::chrono::high_resolution_clock::now();
        updatePLCValues();
        if (previousTIMHeartbeat != statusDB_.timHeartbeat()) {
            previousTIMHeartbeat = statusDB_.timHeartbeat();
            isTIMAlive_ = true;
        } else {
            logger_->warn("No TIM heartbeat detected");
            isTIMAlive_ = false;
        }
        if (localHeartbeat == config_->getMaxHeartbeatValue()) {
            localHeartbeat = 0;
        }
        localHeartbeat++;
        if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_INT, localHeartbeat,
            config_->getCommandsDBLocation()[0],
            config_->getCommandsVariablesLocation()["LocalHeartbeat"][0],
            config_->getCommandsVariablesLocation()["LocalHeartbeat"][1])) {
            logger_->error("Could not write heartbeat command");
        }
        std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start);
        if ((config_->getUpdateInterval() - elapsed).count() > 0) {
            std::this_thread::sleep_for(config_->getUpdateInterval() - elapsed);
        } else {
            logger_->warn("updatePLCValues(): execution time longer than the update interval");
        }
        firstGrabberLoop_.notify_one();
    }
    isTIMAlive_ = false;
}

bool TIMS300::setBooleanCommand(const std::string &name, bool value) {
    std::array<unsigned int, 2> location = config_->getCommandsVariablesLocation()[name];
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, value,
        config_->getCommandsDBLocation()[0], location[0], location[1])) {
        logger_->error("Could not write the command {} with {}", name, value);
        return false;
    }
    return true;
}

bool TIMS300::setFloatSetting(const std::string &name, float value) {
    std::array<unsigned int, 2> location = config_->getSettingsVariablesLocation()[name];
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_REAL, value,
        config_->getSettingsDBLocation()[0], location[0], location[1])) {
        logger_->error("Could not write the setting {} with {}", name, value);
        return false;
    }
    return true;
}

}  // namespace crf::actuators::tim
