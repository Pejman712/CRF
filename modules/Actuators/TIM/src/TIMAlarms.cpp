/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <map>
#include <utility>
#include <mutex>
#include <shared_mutex>

#include "TIM/TIMAlarms.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

namespace crf::actuators::tim {

using crf::devices::siemensplc::SiemensPLCTypeConverter;

TIMAlarms::TIMAlarms() :
    logger_("TIMAlarms"),
    isEmpty_(true),
    barcodeReaderError_(true),
    batteryError_(true),
    chargingArmMotorError_(true),
    chargingArmRequiresAcknowledgement_(true),
    frontBumperPressed_(true),
    frontLaserScannerError_(true),
    frontProtectiveFieldReading_(true),
    backBumperPressed_(true),
    backLaserScannerError_(true),
    backProtectiveFieldReading_(true),
    mainMotorError_(true),
    mainMotorRequiresAcknowledgement_(true),
    positionEncoderError_(true),
    positionEncoderReadingError_(true),
    velocityEncoderError_(true),
    velocityEncoderReadingError_(true),
    emergencyStop_(true) {
    logger_->debug("CTor");
}

TIMAlarms::TIMAlarms(const TIMAlarms& input) :
    logger_("TIMAlarms") {
    logger_->debug("CTor");
    if (input.isEmpty()) {
        isEmpty_ = true;
        return;
    }
    isEmpty_ = false;
    barcodeReaderError_ = input.barcodeReaderError();
    batteryError_ = input.batteryError();
    chargingArmMotorError_ = input.chargingArmMotorError();
    chargingArmRequiresAcknowledgement_ = input.chargingArmRequiresAcknowledgement();
    frontBumperPressed_ = input.frontBumperPressed();
    frontLaserScannerError_ = input.frontLaserScannerError();
    frontProtectiveFieldReading_ = input.frontProtectiveFieldReading();
    backBumperPressed_ = input.backBumperPressed();
    backLaserScannerError_ = input.backLaserScannerError();
    backProtectiveFieldReading_ = input.backProtectiveFieldReading();
    mainMotorError_ = input.mainMotorError();
    mainMotorRequiresAcknowledgement_ = input.mainMotorRequiresAcknowledgement();
    positionEncoderError_ = input.positionEncoderError();
    positionEncoderReadingError_ = input.positionEncoderReadingError();
    velocityEncoderError_ = input.velocityEncoderError();
    velocityEncoderReadingError_ = input.velocityEncoderReadingError();
    emergencyStop_ = input.emergencyStop();
}

bool TIMAlarms::parseSiemensPLCBuffer(const std::string& buffer,
    std::map<std::string, std::array<unsigned int, 2>> variablesDBLocation) {
    logger_->debug("parseSiemensPLCBuffer");
    try {
        barcodeReaderError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["BarcodeReaderError"][1],
            variablesDBLocation["BarcodeReaderError"][0]);
        batteryError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["BatteryError"][1],
            variablesDBLocation["BatteryError"][0]);
        chargingArmMotorError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ChargingArmMotorError"][1],
            variablesDBLocation["ChargingArmMotorError"][0]);
        chargingArmRequiresAcknowledgement_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ChargingArmRequiresAcknowledgement"][1],
            variablesDBLocation["ChargingArmRequiresAcknowledgement"][0]);
        frontBumperPressed_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["FrontBumperPressed"][1],
            variablesDBLocation["FrontBumperPressed"][0]);
        frontLaserScannerError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["FrontLaserScannerError"][1],
            variablesDBLocation["FrontLaserScannerError"][0]);
        frontProtectiveFieldReading_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["FrontProtectiveFieldReading"][1],
            variablesDBLocation["FrontProtectiveFieldReading"][0]);
        backBumperPressed_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["BackBumperPressed"][1],
            variablesDBLocation["BackBumperPressed"][0]);
        backLaserScannerError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["BackLaserScannerError"][1],
            variablesDBLocation["BackLaserScannerError"][0]);
        backProtectiveFieldReading_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["BackProtectiveFieldReading"][1],
            variablesDBLocation["BackProtectiveFieldReading"][0]);
        mainMotorError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["MainMotorError"][1],
            variablesDBLocation["MainMotorError"][0]);
        mainMotorRequiresAcknowledgement_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["MainMotorRequiresAcknowledgement"][1],
            variablesDBLocation["MainMotorRequiresAcknowledgement"][0]);
        positionEncoderError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["PositionEncoderError"][1],
            variablesDBLocation["PositionEncoderError"][0]);
        positionEncoderReadingError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["PositionEncoderReadingError"][1],
            variablesDBLocation["PositionEncoderReadingError"][0]);
        velocityEncoderError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["VelocityEncoderError"][1],
            variablesDBLocation["VelocityEncoderError"][0]);
        velocityEncoderReadingError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["VelocityEncoderReadingError"][1],
            variablesDBLocation["VelocityEncoderReadingError"][0]);
        emergencyStop_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["EmergencyStop"][1],
            variablesDBLocation["EmergencyStop"][0]);
    } catch (std::invalid_argument& e) {
        logger_->error("Failed to parse TIM alarms - {}", e.what());
        return false;
    }
    isEmpty_ = false;
    return true;
}

void TIMAlarms::clear() {
    isEmpty_ = true;
    barcodeReaderError_ = true;
    batteryError_ = true;
    chargingArmMotorError_ = true;
    chargingArmRequiresAcknowledgement_ = true;
    frontBumperPressed_ = true;
    frontLaserScannerError_ = true;
    frontProtectiveFieldReading_ = true;
    backBumperPressed_ = true;
    backLaserScannerError_ = true;
    backProtectiveFieldReading_ = true;
    mainMotorError_ = true;
    mainMotorRequiresAcknowledgement_ = true;
    positionEncoderError_ = true;
    positionEncoderReadingError_ = true;
    velocityEncoderError_ = true;
    velocityEncoderReadingError_ = true;
    emergencyStop_ = true;
    return;
}

bool TIMAlarms::isEmpty() const {
    return isEmpty_;
}

bool TIMAlarms::barcodeReaderError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return barcodeReaderError_;
}
void TIMAlarms::barcodeReaderError(bool input) {
    isEmpty_ = false;
    barcodeReaderError_ = input;
}

bool TIMAlarms::batteryError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return batteryError_;
}
void TIMAlarms::batteryError(bool input) {
    isEmpty_ = false;
    batteryError_ = input;
}

bool TIMAlarms::chargingArmMotorError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return chargingArmMotorError_;
}
void TIMAlarms::chargingArmMotorError(bool input) {
    isEmpty_ = false;
    chargingArmMotorError_ = input;
}

bool TIMAlarms::chargingArmRequiresAcknowledgement() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return chargingArmRequiresAcknowledgement_;
}
void TIMAlarms::chargingArmRequiresAcknowledgement(bool input) {
    isEmpty_ = false;
    chargingArmRequiresAcknowledgement_ = input;
}

bool TIMAlarms::frontBumperPressed() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return frontBumperPressed_;
}
void TIMAlarms::frontBumperPressed(bool input) {
    isEmpty_ = false;
    frontBumperPressed_ = input;
}

bool TIMAlarms::frontLaserScannerError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return frontLaserScannerError_;
}
void TIMAlarms::frontLaserScannerError(bool input) {
    isEmpty_ = false;
    frontLaserScannerError_ = input;
}

bool TIMAlarms::frontProtectiveFieldReading() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return frontProtectiveFieldReading_;
}
void TIMAlarms::frontProtectiveFieldReading(bool input) {
    isEmpty_ = false;
    frontProtectiveFieldReading_ = input;
}

bool TIMAlarms::backBumperPressed() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return backBumperPressed_;
}
void TIMAlarms::backBumperPressed(bool input) {
    isEmpty_ = false;
    backBumperPressed_ = input;
}

bool TIMAlarms::backLaserScannerError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return backLaserScannerError_;
}
void TIMAlarms::backLaserScannerError(bool input) {
    isEmpty_ = false;
    backLaserScannerError_ = input;
}

bool TIMAlarms::backProtectiveFieldReading() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return backProtectiveFieldReading_;
}
void TIMAlarms::backProtectiveFieldReading(bool input) {
    isEmpty_ = false;
    backProtectiveFieldReading_ = input;
}

bool TIMAlarms::mainMotorError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return mainMotorError_;
}
void TIMAlarms::mainMotorError(bool input) {
    isEmpty_ = false;
    mainMotorError_ = input;
}

bool TIMAlarms::mainMotorRequiresAcknowledgement() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return mainMotorRequiresAcknowledgement_;
}
void TIMAlarms::mainMotorRequiresAcknowledgement(bool input) {
    isEmpty_ = false;
    mainMotorRequiresAcknowledgement_ = input;
}

bool TIMAlarms::positionEncoderError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return positionEncoderError_;
}
void TIMAlarms::positionEncoderError(bool input) {
    isEmpty_ = false;
    positionEncoderError_ = input;
}

bool TIMAlarms::positionEncoderReadingError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return positionEncoderReadingError_;
}
void TIMAlarms::positionEncoderReadingError(bool input) {
    isEmpty_ = false;
    positionEncoderReadingError_ = input;
}

bool TIMAlarms::velocityEncoderError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return velocityEncoderError_;
}
void TIMAlarms::velocityEncoderError(bool input) {
    isEmpty_ = false;
    velocityEncoderError_ = input;
}

bool TIMAlarms::velocityEncoderReadingError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return velocityEncoderReadingError_;
}
void TIMAlarms::velocityEncoderReadingError(bool input) {
    isEmpty_ = false;
    velocityEncoderReadingError_ = input;
}

bool TIMAlarms::emergencyStop() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Alarms is empty");
    }
    return emergencyStop_;
}
void TIMAlarms::emergencyStop(bool input) {
    isEmpty_ = false;
    emergencyStop_ = input;
}

}  // namespace crf::actuators::tim
