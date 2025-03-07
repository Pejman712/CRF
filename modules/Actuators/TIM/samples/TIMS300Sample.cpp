/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <fstream>
#include <optional>
#include <unistd.h>

#include <boost/program_options.hpp>

#include "TIM/TIMS300/TIMS300.hpp"

namespace po = boost::program_options;

/*
    Missing Methods:
        - std::optional<float> getMaximumVelocity();
        - std::map<int, std::tuple<float, float, float>> getClosestObstacleAreas();
        - std::optional<bool> isInObstacleArea();
        - std::optional<bool> enableMovementWithDevicesDeployed();
        - std::optional<bool> isSafeToMove();
*/

bool getTIMStatus(std::shared_ptr<crf::actuators::tim::TIMS300> tim) {
    std::cout << "---------------------- TIM Status ----------------------" << std::endl;
    std::optional<float> currentPosition = tim->getCurrentPosition();
    if (!currentPosition) {
        std::cout << "Could not get current position" << std::endl;
        return false;
    }
    std::cout << "getCurrentPosition() = " << currentPosition.value() << std::endl;

    std::optional<float> targetPosition = tim->getTargetPosition();
    if (!targetPosition) {
        std::cout << "Could not get target position" << std::endl;
        return false;
    }
    std::cout << "getTargetPosition() = " << targetPosition.value() << std::endl;

    std::optional<float> currentVelocity = tim->getCurrentVelocity();
    if (!currentVelocity) {
        std::cout << "Could not get current velocity" << std::endl;
        return false;
    }
    std::cout << "getCurrentVelocity() = " << currentVelocity.value() << std::endl;

    std::optional<float> targetVelocity = tim->getTargetVelocity();
    if (!targetVelocity) {
        std::cout << "Could not get target velocity" << std::endl;
        return false;
    }
    std::cout << "getTargetVelocity() = " << targetVelocity.value() << std::endl;

    std::optional<float> moving = tim->isMoving();
    if (!moving) {
        std::cout << "Could not get moving" << std::endl;
        return false;
    }
    std::cout << "isMoving() = " << moving.value() << std::endl;

    std::optional<float> targetReached = tim->isTargetReached();
    if (!targetReached) {
        std::cout << "Could not get targetReached" << std::endl;
        return false;
    }
    std::cout << "isTargetReached() = " << targetReached.value() << std::endl;

    std::optional<float> charging = tim->isCharging();
    if (!charging) {
        std::cout << "Could not get charging" << std::endl;
        return false;
    }
    std::cout << "isCharging() = " << charging.value() << std::endl;

    std::optional<float> chargingCurrent = tim->getChargingCurrent();
    if (!chargingCurrent) {
        std::cout << "Could not get chargingCurrent" << std::endl;
        return false;
    }
    std::cout << "getChargingCurrent() = " << chargingCurrent.value() << std::endl;

    std::optional<float> batteryVoltage = tim->getBatteryVoltage();
    if (!batteryVoltage) {
        std::cout << "Could not get batteryVoltage" << std::endl;
        return false;
    }
    std::cout << "getBatteryVoltage() = " << batteryVoltage.value() << std::endl;

    std::optional<float> batteryCurrent = tim->getBatteryCurrent();
    if (!batteryCurrent) {
        std::cout << "Could not get batteryCurrent" << std::endl;
        return false;
    }
    std::cout << "getBatteryCurrent() = " << batteryCurrent.value() << std::endl;

    std::optional<float> inEconomy = tim->isInEconomy();
    if (!inEconomy) {
        std::cout << "Could not get inEconomy" << std::endl;
        return false;
    }
    std::cout << "isInEconomy() = " << inEconomy.value() << std::endl;

    std::optional<float> frontWarningField = tim->isFrontWarningFieldActive();
    if (!frontWarningField) {
        std::cout << "Could not get frontWarningField" << std::endl;
        return false;
    }
    std::cout << "isFrontWarningFieldActive() = " << frontWarningField.value() << std::endl;

    std::optional<float> backWarningField = tim->isBackWarningFieldActive();
    if (!backWarningField) {
        std::cout << "Could not get backWarningField" << std::endl;
        return false;
    }
    std::cout << "isBackWarningFieldActive() = " << backWarningField.value() << std::endl;

    crf::actuators::tim::TIMAlarms alarms = tim->getAlarms();
    if (alarms.isEmpty()) {
        std::cout << "Could not get alarms" << std::endl;
        return false;
    }
    std::cout << "getAlarms()" << std::endl;
    std::cout << "  - barcodeReaderError = " <<
         alarms.barcodeReaderError() << std::endl;
    std::cout << "  - batteryError = " <<
         alarms.batteryError() << std::endl;
    std::cout << "  - chargingArmMotorError = " <<
         alarms.chargingArmMotorError() << std::endl;
    std::cout << "  - frontBumperPressed = " <<
         alarms.frontBumperPressed() << std::endl;
    std::cout << "  - frontLaserScannerError = " <<
         alarms.frontLaserScannerError() << std::endl;
    std::cout << "  - frontProtectiveFieldReading = " <<
        alarms.frontProtectiveFieldReading() << std::endl;
    std::cout << "  - backBumperPressed = " <<
        alarms.backBumperPressed() << std::endl;
    std::cout << "  - backLaserScannerError = " <<
        alarms.backLaserScannerError() << std::endl;
    std::cout << "  - backProtectiveFieldReading = " <<
        alarms.backProtectiveFieldReading() << std::endl;
    std::cout << "  - mainMotorError = " <<
        alarms.mainMotorError() << std::endl;
    std::cout << "  - positionEncoderError = " <<
        alarms.positionEncoderError() << std::endl;
    std::cout << "  - positionEncoderReadingError = " <<
        alarms.positionEncoderReadingError() << std::endl;
    std::cout << "  - velocityEncoderError = " <<
        alarms.velocityEncoderError() << std::endl;
    std::cout << "  - velocityEncoderReadingError = " <<
        alarms.velocityEncoderReadingError() << std::endl;
    std::cout << "  - emergencyStop = " <<
        alarms.emergencyStop() << std::endl;

    sleep(5);
    return true;
}

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("configuration", po::value<std::string>(), "Path to the configuration file");
    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    if (!vm.count("configuration")) {
        std::cout << "Missing configuration file." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::ifstream timData(vm["configuration"].as<std::string>());
    nlohmann::json timJSON = nlohmann::json::parse(timData);

    auto tim = std::make_shared<crf::actuators::tim::TIMS300>(timJSON);

    if (!tim->initialize()) {
        std::cout << "Could not initialize connection to TIM" << std::endl;
        return -1;
    }
    sleep(2);

    getTIMStatus(tim);

    std::cout << "--------------- Change Current Position ----------------" << std::endl;
    std::optional<bool> result = tim->setCurrentPosition(4998);
    if (!result) {
        std::cout << "Could not set current position" << std::endl;
        return -1;
    }
    std::cout << "setCurrentPosition() = " << result.value() << std::endl;

    getTIMStatus(tim);

    std::cout << "-------------------- Move to Target --------------------" << std::endl;
    result = tim->moveToTarget(5000, 0.3);
    if (!result) {
        std::cout << "Could not move to target" << std::endl;
        return -1;
    }
    std::cout << "moveToTarget() = " << result.value() << std::endl;

    getTIMStatus(tim);
    sleep(10);
    getTIMStatus(tim);

    std::cout << "-------------------- Move to Target --------------------" << std::endl;
    result = tim->moveToTarget(4998, 0.3);
    if (!result) {
        std::cout << "Could not move to target" << std::endl;
        return -1;
    }
    std::cout << "moveToTarget() = " << result.value() << std::endl;

    getTIMStatus(tim);
    sleep(10);
    getTIMStatus(tim);


    std::cout << "--------------------- Jog Backward ---------------------" << std::endl;
    for (int i=0 ; i < 15 ; i++) {
        result = tim->jog(-0.2);
        if (!result) {
            std::cout << "Could not jog" << std::endl;
            return -1;
        }
        std::cout << "jog() = " << result.value() << std::endl;
        sleep(1);
    }

    getTIMStatus(tim);


    std::cout << "--------------------- Jog Forward ----------------------" << std::endl;
    for (int i=0 ; i < 10 ; i++) {
        result = tim->jog(0.2);
        if (!result) {
            std::cout << "Could not jog" << std::endl;
            return -1;
        }
        std::cout << "jog() = " << result.value() << std::endl;
        sleep(1);
    }

    getTIMStatus(tim);


    std::cout << "-------------------- Start Charging --------------------" << std::endl;
    result = tim->startCharging();
    if (!result) {
        std::cout << "Could not start charging" << std::endl;
    } else {
        std::cout << "startCharging() = " << result.value() << std::endl;
    }

    getTIMStatus(tim);


    std::cout << "--------------------- Stop Charging --------------------" << std::endl;
    result = tim->stopCharging();
    if (!result) {
        std::cout << "Could not stop charging" << std::endl;
        return -1;
    }
    std::cout << "stopCharging() = " << result.value() << std::endl;

    getTIMStatus(tim);


    std::cout << "------------------ Enable Economy Mode ------------------" << std::endl;
    result = tim->enableEconomyMode();
    if (!result) {
        std::cout << "Could not enable economy mode" << std::endl;
        return -1;
    }
    std::cout << "enableEconomyMode() = " << result.value() << std::endl;

    getTIMStatus(tim);


    std::cout << "----------------- Disable Economy Mode -----------------" << std::endl;
    result = tim->disableEconomyMode();
    if (!result) {
        std::cout << "Could not disable economy mode" << std::endl;
        return -1;
    }
    std::cout << "disableEconomyMode() = " << result.value() << std::endl;

    getTIMStatus(tim);

    std::cout << "-------------------- Emergency Stop --------------------" << std::endl;
    result = tim->emergencyStop();
    if (!result) {
        std::cout << "Could not send emergency stop" << std::endl;
        return -1;
    }
    std::cout << "emergencyStop() = " << result.value() << std::endl;

    getTIMStatus(tim);


    std::cout << "------------------ Acknowledge Alarms ------------------" << std::endl;
    result = tim->acknowledgeAlarms();
    if (!result) {
        std::cout << "Could not acknowledge alarms" << std::endl;
        // return -1;
    }
    std::cout << "acknowledgeAlarms() = " << result.value() << std::endl;

    getTIMStatus(tim);


    std::cout << "-------------------- Move to Target --------------------" << std::endl;
    result = tim->moveToTarget(5000, 0.1);
    if (!result) {
        std::cout << "Could not move to target" << std::endl;
        return -1;
    }
    std::cout << "moveToTarget() = " << result.value() << std::endl;

    getTIMStatus(tim);


    std::cout << "------------------------- Stop -------------------------" << std::endl;
    result = tim->stop();
    if (!result) {
        std::cout << "Could not stop" << std::endl;
        return -1;
    }
    std::cout << "stop() = " << result.value() << std::endl;

    getTIMStatus(tim);

    std::cout << "-------------------- Move to Target --------------------" << std::endl;
    result = tim->moveToTarget(4998, 0.3);
    if (!result) {
        std::cout << "Could not move to target" << std::endl;
        return -1;
    }
    std::cout << "moveToTarget() = " << result.value() << std::endl;

    getTIMStatus(tim);

    sleep(10);
    return 0;
}
