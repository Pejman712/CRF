/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <vector>
#include <string>
#include <optional>
#include <cstdint>

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CiA402DriverConfiguration.hpp"

namespace crf::devices::canopendrivers {

CiA402DriverConfiguration::CiA402DriverConfiguration(const nlohmann::json& json):
    checkSupportedModes_(true),
    posUnits_(1),
    velUnits_(1),
    accUnits_(1),
    jrkUnits_(1),
    tqeUnits_(1),
    crnUnits_(1),
    logger_("CiA402DriverConfiguration") {
    parse(json);
}

uint64_t CiA402DriverConfiguration::getSlaveID() const {
    return slaveID_;
}

bool CiA402DriverConfiguration::checkSupportedModes() const {
    return checkSupportedModes_;
}

double CiA402DriverConfiguration::getPositionUnitConversion() const {
    return posUnits_;
}

double CiA402DriverConfiguration::getVelocityUnitConversion() const {
    return velUnits_;
}

double CiA402DriverConfiguration::getAccelerationUnitConversion() const {
    return accUnits_;
}

double CiA402DriverConfiguration::getJerkUnitConversion() const {
    return jrkUnits_;
}

double CiA402DriverConfiguration::getTorqueUnitConversion() const {
    return tqeUnits_;
}

double CiA402DriverConfiguration::getCurrentUnitConversion() const {
    return crnUnits_;
}

std::optional<Polarity> CiA402DriverConfiguration::getPolarity() const {
    return polarity_;
}

std::optional<uint32_t> CiA402DriverConfiguration::getMaxMotorSpeed() const {
    return maxMotorSpeed_;
}

std::optional<uint16_t> CiA402DriverConfiguration::getMaxMotorTorque() const {
    return maxMotorTorque_;
}

std::optional<uint16_t> CiA402DriverConfiguration::getMaxMotorCurrent() const {
    return maxMotorCurrent_;
}

std::optional<uint32_t> CiA402DriverConfiguration::getMotorRatedTorque() const {
    return motorRatedTorque_;
}

std::optional<uint32_t> CiA402DriverConfiguration::getMotorRatedCurrent() const {
    return motorRatedCurrent_;
}

std::optional<QuickStopOptionCode> CiA402DriverConfiguration::getQuickStopOptionCode() const {
    return quickStopCode_;
}

std::optional<ShutdownOptionCode> CiA402DriverConfiguration::getShutdownOptionCode() const {
    return shutdownCode_;
}

std::optional<DisableOperationOptionCode> CiA402DriverConfiguration::getDisableOperationOptionCode() const {  // NOLINT
    return disableOperationCode_;
}

std::optional<HaltOptionCode> CiA402DriverConfiguration::getHaltOptionCode() const {
    return haltCode_;
}

std::optional<FaultOptionCode> CiA402DriverConfiguration::getFaultOptionCode() const {
    return faultCode_;
}

nlohmann::json CiA402DriverConfiguration::getProfilePositionConfig() const {
    return ppmConfig_;
}

nlohmann::json CiA402DriverConfiguration::getProfileVelocityConfig() const {
    return pvmConfig_;
}

nlohmann::json CiA402DriverConfiguration::getProfileTorqueConfig() const {
    return ptmConfig_;
}

nlohmann::json CiA402DriverConfiguration::getVelocityModeConfig() const {
    return vomConfig_;
}

nlohmann::json CiA402DriverConfiguration::getInterpolatedPositionConfig() const {
    return ipmConfig_;
}

nlohmann::json CiA402DriverConfiguration::getHomingConfig() const {
    return homConfig_;
}

nlohmann::json CiA402DriverConfiguration::getCyclicPositionConfig() const {
    return cspConfig_;
}

nlohmann::json CiA402DriverConfiguration::getCyclicVelocityConfig() const {
    return csvConfig_;
}

nlohmann::json CiA402DriverConfiguration::getCyclicTorqueConfig() const {
    return cstConfig_;
}

// private

void CiA402DriverConfiguration::parse(const nlohmann::json& json) {
    try {
        // Mandatory
        if (!json.contains("SlaveID")) {
            throw std::invalid_argument("Slave ID not present");
        }

        slaveID_ = json.at("SlaveID").get<uint64_t>();

        // Optional
        // Check supported modes of operation
        if (json.contains("CheckSupportedModes")) {
            checkSupportedModes_ = json.at("CheckSupportedModes").get<bool>();
        }

        // Optional
        // Unit conversion
        if (json.contains("PositionUnitConversion")) {
            posUnits_ = json.at("PositionUnitConversion").get<double>();
        } else {
            logger_->warn(
                "Position units not present, driver position will be in non standard units!");
        }
        if (json.contains("VelocityUnitConversion")) {
            velUnits_ = json.at("VelocityUnitConversion").get<double>();
        } else {
            logger_->warn(
                "Velocity units not present, driver velocity will be in non standard units!");
        }
        if (json.contains("AccelerationUnitConversion")) {
            accUnits_ = json.at("AccelerationUnitConversion").get<double>();
        } else {
            logger_->warn(
                "Acceleration units not present, driver acceleration will be in non standard units!");  // NOLINT
        }
        if (json.contains("JerkUnitConversion")) {
            jrkUnits_ = json.at("JerkUnitConversion").get<double>();
        } else {
            logger_->warn(
                "Jerk units not present, driver jerk will be in non standard units!");
        }
        if (json.contains("TorqueUnitConversion")) {
            tqeUnits_ = json.at("TorqueUnitConversion").get<double>();
        } else {
            logger_->warn(
                "Torque units not present, driver torque will be in non standard units!");
        }
        if (json.contains("CurrentUnitConversion")) {
            crnUnits_ = json.at("CurrentUnitConversion").get<double>();
        } else {
            logger_->warn(
                "Current units not present, driver current will be in non standard units!");
        }

        nlohmann::json units;
        units["PositionUnitConversion"] = posUnits_;
        units["VelocityUnitConversion"] = velUnits_;
        units["AccelerationUnitConversion"] = accUnits_;
        units["JerkUnitConversion"] = jrkUnits_;
        units["TorqueUnitConversion"] = tqeUnits_;
        units["CurrentUnitConversion"] = crnUnits_;

        ppmConfig_ = units;
        pvmConfig_ = units;
        ptmConfig_ = units;
        vomConfig_ = units;
        homConfig_ = units;
        ipmConfig_ = units;
        cspConfig_ = units;
        csvConfig_ = units;
        cstConfig_ = units;

        // General registries
        if (json.contains("Polarity")) {
            polarity_ = polarityMap_.at(json.at("Polarity").get<std::string>());
        }
        if (json.contains("MaxMotorSpeed")) {
            maxMotorSpeed_ = rpmToRadPerSec_ * json.at("MaxMotorSpeed").get<double>();
        }
        if (json.contains("MaxMotorTorque")) {
            maxMotorTorque_ = tqeUnits_ * json.at("MaxMotorTorque").get<double>();
        }
        if (json.contains("MaxMotorCurrent")) {
            maxMotorCurrent_ = crnUnits_ * json.at("MaxMotorCurrent").get<double>();
        }
        if (json.contains("MotorRatedTorque")) {
            // You have to input it in milli Nm
            motorRatedTorque_ = newtmToMilliNewtm * json.at("MotorRatedTorque").get<double>();
        }
        if (json.contains("MotorRatedCurrent")) {
            // You have to input it in milli Amp
            motorRatedCurrent_ = ampsToMilliAmps * json.at("MotorRatedCurrent").get<double>();
        }

        // Option codes
        if (json.contains("QuickStopOptionCode")) {
            quickStopCode_ = quickStopCodeMap_.at(
                json.at("QuickStopOptionCode").get<std::string>());
        }
        if (json.contains("ShutdownOptionCode")) {
            shutdownCode_ = shutdownCodeMap_.at(json.at("ShutdownOptionCode").get<std::string>());
        }
        if (json.contains("DisableOperationOptionCode")) {
            disableOperationCode_ = disableOperationCodeMap_.at(
                json.at("DisableOperationOptionCode").get<std::string>());
        }
        if (json.contains("HaltOptionCode")) {
            haltCode_ = haltCodeMap_.at(json.at("HaltOptionCode").get<std::string>());
        }
        if (json.contains("FaultOptionCode")) {
            faultCode_ = faultCodeMap_.at(json.at("FaultOptionCode").get<std::string>());
        }

        if (!json.contains("ModesOfOperation")) return;

        // Mode configurations
        nlohmann::json modes = json.at("ModesOfOperation");

        if (modes.contains("ProfilePositionMode")) {
            ppmConfig_.merge_patch(modes.at("ProfilePositionMode"));
        }
        if (modes.contains("ProfileVelocityMode")) {
            pvmConfig_.merge_patch(modes.at("ProfileVelocityMode"));
        }
        if (modes.contains("ProfileTorqueMode")) {
            ptmConfig_.merge_patch(modes.at("ProfileTorqueMode"));
        }
        if (modes.contains("VelocityMode")) {
            vomConfig_.merge_patch(modes.at("VelocityMode"));
        }
        if (modes.contains("InterpolatedPositionMode")) {
            ipmConfig_.merge_patch(modes.at("InterpolatedPositionMode"));
        }
        if (modes.contains("HomingMode")) {
            homConfig_.merge_patch(modes.at("HomingMode"));
        }
        if (modes.contains("CyclicSynchronousPositionMode")) {
            cspConfig_.merge_patch(modes.at("CyclicSynchronousPositionMode"));
        }
        if (modes.contains("CyclicSynchronousVelocityMode")) {
            csvConfig_.merge_patch(modes.at("CyclicSynchronousVelocityMode"));
        }
        if (modes.contains("CyclicSynchronousTorqueMode")) {
            cstConfig_.merge_patch(modes.at("CyclicSynchronousTorqueMode"));
        }
    } catch (const std::exception& e) {
        throw std::invalid_argument(
            "CiA402DriverConfiguration - Parse - Error parsing: " + std::string(e.what()));
    }
}

}  // namespace crf::devices::canopendrivers
