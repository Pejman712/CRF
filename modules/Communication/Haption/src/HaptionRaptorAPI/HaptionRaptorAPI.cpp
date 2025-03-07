/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <string>
#include <chrono>
#include <thread>

#include <RaptorAPI.hpp>

#include "Haption/HaptionRaptorAPI/HaptionRaptorAPI.hpp"
#include "crf/ResponseDefinitions.hpp"

namespace crf::devices::haption {

HaptionRaptorAPI::HaptionRaptorAPI(unsigned int localPort, const std::string& remoteIP,
    unsigned int remotePort, const std::string& parametersFile) :
    parametersFile_(parametersFile),
    logger_("HaptionRaptorAPI"),
    raptorHandle_(),
    connected_(false),
    calibTimeoutSec_(30),
    communicationLoopStopped_(true),
    communicationLoopThread_() {
    uint8_t versionMajor = 0;
    uint8_t versionMinor = 0;
    std::string build;
    raptorHandle_.GetVersion(versionMajor, versionMinor, build);
    logger_->info("RaptorAPI version {}.{} and build {}", versionMajor, versionMinor, build);

    std::string fields = "channel=SimpleChannelUDP:";
    fields = fields + "localport=" + std::to_string(localPort) + ":";
    fields = fields + "remoteip=" + remoteIP + ":";
    fields = fields + "remoteport=" + std::to_string(remotePort);
    logger_->info("Connection parameters {}", fields);
    HAPTION::ErrorCode result = raptorHandle_.DefineSpecificFields(fields);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to configure the connection - Code {}", errorParser(result));
    }
}

HaptionRaptorAPI::HaptionRaptorAPI(unsigned int card, unsigned int slot, unsigned int offset,
    const std::string& parametersFile) :
    parametersFile_(parametersFile),
    logger_("HaptionRaptorAPI"),
    raptorHandle_(),
    connected_(false) {
    uint8_t versionMajor = 0;
    uint8_t versionMinor = 0;
    std::string build;
    raptorHandle_.GetVersion(versionMajor, versionMinor, build);
    logger_->info("RaptorAPI version {}.{} and build {}", versionMajor, versionMinor, build);

    std::string fields = "channel=SimpleChannelCIFX:";
    fields = fields + "card=" + std::to_string(card) + ":";
    fields = fields + "slot=" + std::to_string(slot) + ":";
    fields = fields + "offset=" + std::to_string(offset) + ":";
    logger_->info("Connection parameters {}", fields);
    HAPTION::ErrorCode result = raptorHandle_.DefineSpecificFields(fields);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to configure the connection - Code {}", errorParser(result));
    }
}

HaptionRaptorAPI::~HaptionRaptorAPI() {
    communicationLoopStopped_ = true;
    if (communicationLoopThread_.joinable()) communicationLoopThread_.join();
}

crf::Code HaptionRaptorAPI::startConnection() {
    if (connected_) {
        logger_->error("The haption device is already connected");
        return crf::Code::OK;
    }

    HAPTION::ErrorCode result;
    result = raptorHandle_.Init(parametersFile_);
    logger_->info("Haption parameters file {}", parametersFile_);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to initialize the device");
        return errorParser(result);
    }

    std::string name;
    result = raptorHandle_.GetFullName(name);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to get robot name");
        return errorParser(result);
    }
    logger_->info("Connected to Haption device {}", name);

    crf::Code code = calibrate();
    if (code != crf::Code::OK) {
        logger_->error("Failed to calibrate - Haption Error {}", code);
        return code;
    }
    logger_->info("The haption device is calibrated correctly");

    result = raptorHandle_.ActivateForceFeedback(true);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to enable force feedback");
        return errorParser(result);
    }
    logger_->info("Force feedback activated");

    communicationLoopStopped_ = false;
    communicationLoopThread_ = std::thread(&HaptionRaptorAPI::communicationLoop, this);
    return crf::Code::OK;
}

crf::Code HaptionRaptorAPI::stopConnection() {
    communicationLoopStopped_ = true;
    if (communicationLoopThread_.joinable()) {
        communicationLoopThread_.join();
    }
    HAPTION::ErrorCode result;
    result = raptorHandle_.ActivateForceFeedback(false);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to disable force feedback");
        return errorParser(result);
    }
    result = raptorHandle_.Close();
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to close the connection");
        return errorParser(result);
    }
    return crf::Code::OK;
}

crf::Code HaptionRaptorAPI::calibrate() {
    HAPTION::ErrorCode result;
    HAPTION::CalibrationStatus calibrationStatus;
    result = raptorHandle_.GetCalibrationStatus(calibrationStatus);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to get robot calibration status");
        return errorParser(result);
    }
    if (calibrationStatus == HAPTION::CalibrationStatus::C_CALIBRATED) {
        return crf::Code::OK;
    }

    // The activation of the force-feeback is only needed for automatic calibration.
    result = raptorHandle_.ActivateForceFeedback(true);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to activate force feedback");
        return errorParser(result);
    }

    std::cout <<
        " _____________________________________________________ \n" <<
        "|                                                     |\n" <<
        "|             HAPTION DEVICE CALIBRATION              |\n" <<
        "|                                                     |\n" <<
        "| The Haption device needs to be calibrated. You have |\n" <<
        "|                " << calibTimeoutSec_ << " seconds to do it:                 |\n"  <<
        "|                                                     |" << std::endl;

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now()-start <= std::chrono::seconds(calibTimeoutSec_)) {
        result = raptorHandle_.DoCalibration();
        if (result != HAPTION::ErrorCode::E_NOERROR) {
            logger_->error("Failed to do calibration");
            return errorParser(result);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        HAPTION::CalibrationStatus newCalibrationStatus;
        result = raptorHandle_.GetCalibrationStatus(newCalibrationStatus);
        if (result != HAPTION::ErrorCode::E_NOERROR) {
            logger_->error("Failed to get the calibration status");
            return errorParser(result);
        }
        if (newCalibrationStatus == calibrationStatus) {
            continue;
        }
        if (newCalibrationStatus == HAPTION::CalibrationStatus::C_WAITINGFORPOWER) {
            std::cout << "|                Press the power button               |" << std::endl;
        } else if (newCalibrationStatus == HAPTION::CalibrationStatus::C_WAITINGFORUSER) {
            std::cout << "|    Move the joints to their calibration positions   |" << std::endl;
        } else if (newCalibrationStatus == HAPTION::CalibrationStatus::C_MOVING) {
            std::cout << "|            Warning: the device is moving!           |" << std::endl;
        } else if (newCalibrationStatus == HAPTION::CalibrationStatus::C_CALIBRATIONFAILED) {
            std::cout << "|     Calibration has failed, please restart again    |" << std::endl;
            calibrationStatus = newCalibrationStatus;
            break;
        } else if (newCalibrationStatus == HAPTION::CalibrationStatus::C_CALIBRATED) {
            std::cout <<
                "|                                                     |\n" <<
                "|               Calibration finished                  |\n" <<
                "|_____________________________________________________|" << std::endl;
            calibrationStatus = newCalibrationStatus;
            break;
        }
        calibrationStatus = newCalibrationStatus;
    }

    result = raptorHandle_.ActivateForceFeedback(false);
    if (result != HAPTION::ErrorCode::E_NOERROR) {
        logger_->error("Failed to deactivate force feedback");
        return errorParser(result);
    }

    if (calibrationStatus != HAPTION::CalibrationStatus::C_CALIBRATED) {
        std::cout << "|                                                     |\n" <<
            "|        The Haption device failed to calibrate       |\n" <<
            "|_____________________________________________________|" << std::endl;
        return crf::Code::GatewayTimeout;
    }
    return crf::Code::OK;
}

crf::Code HaptionRaptorAPI::getErrorStatus() {
    return errorParser(raptorHandle_.GetErrorStatus());
}

crf::Code HaptionRaptorAPI::changeBase(const HAPTION::Displacement& iDisp) {
    return errorParser(raptorHandle_.ChangeBase(iDisp));
}

crf::Code HaptionRaptorAPI::changeBase(const HAPTION::Transformation& iTransformation) {
    return errorParser(raptorHandle_.ChangeBase(iTransformation));
}

crf::Code HaptionRaptorAPI::changeViewpoint(const HAPTION::Displacement& iDisp) {
    return errorParser(raptorHandle_.ChangeViewpoint(iDisp));
}

crf::Code HaptionRaptorAPI::changeViewpoint(const HAPTION::Transformation& iTransformation) {
    return errorParser(raptorHandle_.ChangeViewpoint(iTransformation));
}

crf::Code HaptionRaptorAPI::moveViewpoint(const HAPTION::Displacement& iDisp,
    const HAPTION::CartesianVector& iSpeed) {
    return errorParser(raptorHandle_.MoveViewpoint(iDisp, iSpeed));
}

crf::Code HaptionRaptorAPI::moveViewpoint(const HAPTION::Transformation& iTransformation,
    const HAPTION::CartesianVector& iSpeed) {
    return errorParser(raptorHandle_.MoveViewpoint(iTransformation, iSpeed));
}

crf::Code HaptionRaptorAPI::changeMovementScale(const HAPTION::float32_t& iScaleTrans,
    const HAPTION::float32_t& iScaleRot) {
    return errorParser(raptorHandle_.ChangeMovementScale(iScaleTrans, iScaleRot));
}

crf::Code HaptionRaptorAPI::getCartesianPose(HAPTION::Displacement& oPose) {
    return errorParser(raptorHandle_.GetCartesianPose(oPose));
}

crf::Code HaptionRaptorAPI::getCartesianPose(HAPTION::Transformation& oPose) {
    return errorParser(raptorHandle_.GetCartesianPose(oPose));
}

crf::Code HaptionRaptorAPI::getRawCartesianPose(HAPTION::Displacement& oPose) {
    return errorParser(raptorHandle_.GetRawCartesianPose(oPose));
}

crf::Code HaptionRaptorAPI::getRawCartesianPose(HAPTION::Transformation& oPose) {
    return errorParser(raptorHandle_.GetRawCartesianPose(oPose));
}

crf::Code HaptionRaptorAPI::getCartesianSpeed(HAPTION::CartesianVector& oSpeed) {
    return errorParser(raptorHandle_.GetCartesianSpeed(oSpeed));
}

crf::Code HaptionRaptorAPI::getJointAngles(HAPTION::JointVector& oAngles) {
    return errorParser(raptorHandle_.GetJointAngles(oAngles));
}

crf::Code HaptionRaptorAPI::getJointSpeeds(HAPTION::JointVector& oSpeeds) {
    return errorParser(raptorHandle_.GetJointSpeeds(oSpeeds));
}

crf::Code HaptionRaptorAPI::getJointTorques(HAPTION::JointVector& oTorques) {
    return errorParser(raptorHandle_.GetJointTorques(oTorques));
}

crf::Code HaptionRaptorAPI::getMotorCurrents(HAPTION::JointVector& oCurrents) {
    return errorParser(raptorHandle_.GetMotorCurrents(oCurrents));
}

crf::Code HaptionRaptorAPI::getMotorStatus(
    std::array<HAPTION::MotorStatus, HAPTION::MAX_NB_JOINTS> &oStatus) {
    return errorParser(raptorHandle_.GetMotorStatus(oStatus));
}

crf::Code HaptionRaptorAPI::changeCartesianGains(HAPTION::float32_t iKT,
    HAPTION::float32_t iBT, HAPTION::float32_t iKR, HAPTION::float32_t iBR) {
    return errorParser(raptorHandle_.ChangeCartesianGains(iKT, iBT, iKR, iBR));
}

crf::Code HaptionRaptorAPI::getMaxCartesianGains(HAPTION::float32_t& oMaxKT,
    HAPTION::float32_t& oMaxBT, HAPTION::float32_t& oMaxKR, HAPTION::float32_t& oMaxBR) {
    return errorParser(raptorHandle_.GetMaxCartesianGains(oMaxKT, oMaxBT, oMaxKR, oMaxBR));
}

crf::Code HaptionRaptorAPI::startCartesianPositionMode() {
    return errorParser(raptorHandle_.StartCartesianPositionMode());
}

crf::Code HaptionRaptorAPI::changeJointGains(const HAPTION::JointVector& iKs,
    const HAPTION::JointVector& iBs) {
    return errorParser(raptorHandle_.ChangeJointGains(iKs, iBs));
}

crf::Code HaptionRaptorAPI::getMaxJointGains(HAPTION::JointVector& oMaxKs,
    HAPTION::JointVector& oMaxBs) {
    return errorParser(raptorHandle_.GetMaxJointGains(oMaxKs, oMaxBs));
}

crf::Code HaptionRaptorAPI::startJointPositionMode() {
    return errorParser(raptorHandle_.StartJointPositionMode());
}

crf::Code HaptionRaptorAPI::activateForceFeedback(const bool& iActivate) {
    return errorParser(raptorHandle_.ActivateForceFeedback(iActivate));
}

crf::Code HaptionRaptorAPI::changeBrakeStatus(const HAPTION::BrakeStatus& iStatus) {
    return errorParser(raptorHandle_.ChangeBrakeStatus(iStatus));
}

crf::Code HaptionRaptorAPI::getBrakeStatus(HAPTION::BrakeStatus& oStatus) {
    return errorParser(raptorHandle_.GetBrakeStatus(oStatus));
}

crf::Code HaptionRaptorAPI::getAutomatonStatus(HAPTION::AutomatonStatus& oStatus) {
    return errorParser(raptorHandle_.GetAutomatonStatus(oStatus));
}

crf::Code HaptionRaptorAPI::getPowerStatus(HAPTION::PowerStatus& oPowerStatus) {
    return errorParser(raptorHandle_.GetPowerStatus(oPowerStatus));
}

crf::Code HaptionRaptorAPI::forceCartesianPose(const HAPTION::Displacement& iPose) {
    return errorParser(raptorHandle_.ForceCartesianPose(iPose));
}

crf::Code HaptionRaptorAPI::forceCartesianPose(const HAPTION::Transformation& iPose) {
    return errorParser(raptorHandle_.ForceCartesianPose(iPose));
}

crf::Code HaptionRaptorAPI::getClutchOffset(HAPTION::Displacement& oOffset) {
    return errorParser(raptorHandle_.GetClutchOffset(oOffset));
}

crf::Code HaptionRaptorAPI::activateClutch(bool iActivation) {
    return errorParser(raptorHandle_.ActivateClutch(iActivation));
}

crf::Code HaptionRaptorAPI::getClutchOffset(HAPTION::Transformation& oOffset) {
    return errorParser(raptorHandle_.GetClutchOffset(oOffset));
}

crf::Code HaptionRaptorAPI::setCartesianPose(const HAPTION::Displacement& iPose) {
    return errorParser(raptorHandle_.SetCartesianPose(iPose));
}

crf::Code HaptionRaptorAPI::setCartesianPose(const HAPTION::Transformation& iPose) {
    return errorParser(raptorHandle_.SetCartesianPose(iPose));
}

crf::Code HaptionRaptorAPI::setCartesianPoseWithClutchOffset(
    const HAPTION::Displacement& iPose, const HAPTION::Displacement& iOffset) {
    return errorParser(raptorHandle_.SetCartesianPoseWithClutchOffset(iPose, iOffset));
}

crf::Code HaptionRaptorAPI::setCartesianSpeed(const HAPTION::CartesianVector& iSpeed) {
    return errorParser(raptorHandle_.SetCartesianSpeed(iSpeed));
}

crf::Code HaptionRaptorAPI::addCartesianForceOverlay(
    const HAPTION::CartesianVector& iForce) {
    return errorParser(raptorHandle_.AddCartesianForceOverlay(iForce));
}

crf::Code HaptionRaptorAPI::getCartesianForce(HAPTION::CartesianVector& oForce) {
    return errorParser(raptorHandle_.GetCartesianForce(oForce));
}

crf::Code HaptionRaptorAPI::setJointAngles(const HAPTION::JointVector& iJointAngles) {
    return errorParser(raptorHandle_.SetJointAngles(iJointAngles));
}

crf::Code HaptionRaptorAPI::setJointSpeeds(const HAPTION::JointVector& iJointSpeeds) {
    return errorParser(raptorHandle_.SetJointSpeeds(iJointSpeeds));
}

crf::Code HaptionRaptorAPI::addJointTorqueOverlay(
    const HAPTION::JointVector& iJointTorques) {
    return errorParser(raptorHandle_.AddJointTorqueOverlay(iJointTorques));
}

crf::Code HaptionRaptorAPI::getOperatorButton(const HAPTION::OperatorButton& iButton,
    bool& oButtonState) {
    return errorParser(raptorHandle_.GetOperatorButton(iButton, oButtonState));
}

crf::Code HaptionRaptorAPI::getTriggerValue(HAPTION::float32_t& oValue) {
    return errorParser(raptorHandle_.GetTriggerValue(oValue));
}

crf::Code HaptionRaptorAPI::clearError() {
    return errorParser(raptorHandle_.ClearError());
}

crf::Code HaptionRaptorAPI::switchPowerOnOff(const bool& iEnableSupply) {
    return errorParser(raptorHandle_.SwitchPowerOnOff(iEnableSupply));
}

crf::Code HaptionRaptorAPI::getTCPOffset(HAPTION::Displacement& oTcpOffset) {
    return errorParser(raptorHandle_.GetTCPOffset(oTcpOffset));
}

crf::Code HaptionRaptorAPI::changeTCPOffset(const HAPTION::Displacement& iTcpOffset) {
    return errorParser(raptorHandle_.ChangeTCPOffset(iTcpOffset));
}

crf::Code HaptionRaptorAPI::setTriggerLockLed(const bool& iStatus) {
    return errorParser(raptorHandle_.SetTriggerLockLed(iStatus));
}

crf::Code HaptionRaptorAPI::isJointLimitReached(
    std::array<bool, HAPTION::MAX_NB_JOINTS>& oLimitsReached) {  // NOLINT
    return errorParser(raptorHandle_.IsJointLimitReached(oLimitsReached));
}

void HaptionRaptorAPI::communicationLoop() {
    HAPTION::ErrorCode result;
    std::chrono::time_point<std::chrono::system_clock> timeStamp;
    auto start = std::chrono::high_resolution_clock::now();
    while (!communicationLoopStopped_) {
        result = raptorHandle_.ReadState(timeStamp);
        if (result != HAPTION::ErrorCode::E_NOERROR) {
            logger_->critical("Failed to read state");
        }
        result = raptorHandle_.SendSetpoints();
        if (result != HAPTION::ErrorCode::E_NOERROR) {
            logger_->critical("Failed to send state");
        }
        if (std::chrono::high_resolution_clock::now() - start > loopTime_) {
            logger_->critical("Real time violation");
        }
        std::this_thread::sleep_until(start + loopTime_);
        start = std::chrono::high_resolution_clock::now();
    }
}

crf::Code HaptionRaptorAPI::errorParser(HAPTION::ErrorCode code) {
    if (code == HAPTION::ErrorCode::E_NOERROR) {
        return crf::Code::OK;
    }
    std::string message;
    raptorHandle_.GetLastError(code, message);
    logger_->error("{} (Code {})", message, code);
    if (code == HAPTION::ErrorCode::E_NOTCONNECTED) {
        logger_->critical("Not connected to the device.");
        return crf::Code::Disconnected;
    } else if (code == HAPTION::ErrorCode::E_WATCHDOG) {
        logger_->critical("Software watchdog on the connection raised.");
        return crf::Code::ConnectionWatchdog;
    } else if (code == HAPTION::ErrorCode::E_MEMORYALLOCATION) {
        logger_->critical("Problems with memory allocation.");
        return crf::Code::MemoryAllocationError;
    } else if (code == HAPTION::ErrorCode::E_BREAKDOWN) {
        logger_->critical("Hardware breakdown.");
        return crf::Code::FaultState;
    } else if (code == HAPTION::ErrorCode::E_SAFETYSTOP_DEADMAN) {
        logger_->error("Movement stopped due to deadman switch released.");
        return crf::Code::DeadmanSwitchReleased;
    } else if (code == HAPTION::ErrorCode::E_SAFETYSTOP_POWERBUTTON) {
        logger_->error("Movement stopped due to power button depressed.");
        return crf::Code::PoweredOff;
    } else if (code == HAPTION::ErrorCode::E_EMERGENCYSTOP) {
        logger_->error("Movement stopped due to emergency stop pressed.");
        return crf::Code::EmergencyStop;
    } else if (code == HAPTION::ErrorCode::E_TEMPERATURE) {
        logger_->error("Motor torques reduced because of exceeded rotor temperature threshold.");
        return crf::Code::TemperatureError;
    } else if (code == HAPTION::ErrorCode::E_SPEEDEXCESS) {
        logger_->error("Movement stopped due to excessive speed.");
        return crf::Code::HardwareLimit;
    } else if (code == HAPTION::ErrorCode::E_TIMEOUT) {
        logger_->error("Timeout on receiving an update message.");
        return crf::Code::GatewayTimeout;
    } else if (code == HAPTION::ErrorCode::E_BEYONDUPPER) {
        logger_->error("No movement since the current position is higher than upper limit.");
        return crf::Code::HardwareLimit;
    } else if (code == HAPTION::ErrorCode::E_BEYONDLOWER) {
        logger_->error("No movement since the current position is lower than lower limit.");
        return crf::Code::HardwareLimit;
    } else if (code == HAPTION::ErrorCode::E_NOTAVAILABLE) {
        logger_->error("Function not available in this context.");
        return crf::Code::MethodNotAllowed;
    } else if (code == HAPTION::ErrorCode::E_WRONGPARAMETER) {
        logger_->error("Wrong parameter given.");
        return crf::Code::BadRequest;
    } else if (code == HAPTION::ErrorCode::E_WRONGSERIALNUMBER) {
        logger_->error("Wrong serial number given.");
        return crf::Code::BadRequest;
    } else if (code == HAPTION::ErrorCode::E_FILENOTFOUND) {
        logger_->error("File not found.");
        return crf::Code::BadRequest;
    } else if (code == HAPTION::ErrorCode::E_PINGERROR) {
        logger_->error("There is a ping error with the device.");
        return crf::Code::UnstableConnection;
    } else if (code == HAPTION::ErrorCode::E_LICENSEERROR) {
        logger_->error("There is a license error.");
        return crf::Code::Unauthorized;
    } else if (code == HAPTION::ErrorCode::E_WRONGJOINTGAINS) {
        logger_->error("Wrong joint gain given.");
        return crf::Code::BadRequest;
    } else if (code == HAPTION::ErrorCode::E_WRONGCARTESIANGAINS) {
        logger_->error("wrong cartesian gain given.");
        return crf::Code::BadRequest;
    } else if (code == HAPTION::ErrorCode::E_FIRMWARENOTREADY) {
        logger_->critical("Device firmware not ready.");
        return crf::Code::FaultState;
    } else if (code == HAPTION::ErrorCode::E_OTHER) {
        return crf::Code::InternalServerError;
    }
    return  crf::Code::Empty;
}

}  // namespace crf::devices::haption
