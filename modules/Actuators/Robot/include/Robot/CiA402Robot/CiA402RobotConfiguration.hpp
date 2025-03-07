/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <string>
#include <vector>
#include <map>

#include "Robot/RobotConfiguration.hpp"
#include "Robot/CiA402Robot/CiA402RobotDefinitions.hpp"

namespace crf::actuators::robot {

/**
 * @brief Class to parse the specifics within the cnfiguration file and
 * store them so the CiA402 robot class can use them.
 *
 */
class CiA402RobotConfiguration : public RobotConfiguration {
 public:
    explicit CiA402RobotConfiguration(const nlohmann::json& robotConfig);
    explicit CiA402RobotConfiguration(const std::string&) = delete;
    ~CiA402RobotConfiguration() override = default;

    /**
     * @brief Get the Position Mode in which the motors will be controlled
     *
     * @return PositionMode to use
     */
    PositionMode getPositionMode() const;

    /**
     * @brief Get the Velocity Mode in which the motors will be controlled
     *
     * @return VelocityMode to use
     */
    VelocityMode getVelocityMode() const;

    /**
     * @brief Get the Torque Mode in which the motors will be controlled
     *
     * @return TorqueMode to use
     */
    TorqueMode getTorqueMode() const;

    /**
     * @brief Get the Motor Config Files
     *
     * @return std::vector<nlohmann::json>
     */
    std::vector<nlohmann::json> getMotorConfigFiles() const;

    /**
     * @brief Get the Number Of Motors
     *
     * @return uint64_t number of motors
     */
    uint64_t getNumberOfMotors() const;

 private:
    /**
     * @brief Method that parses the full JSON object. It is called in the
     * constructor of the class.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param robotJSON JSON object to parse
     */
    void parse(const nlohmann::json& robotJSON);
    void parse(const std::string&) = delete;

    uint64_t numberOfMotors_;
    PositionMode posMode_;
    VelocityMode velMode_;
    TorqueMode tqeMode_;

    std::vector<nlohmann::json> motorJSONs_;

    crf::utility::logger::EventLogger logger_;

    const std::map<std::string, PositionMode> positionMap_ = {
        {"ProfilePosition", PositionMode::ProfilePositionMode},
        {"CyclicSyncPosition", PositionMode::CyclicSyncPositionMode},
        {"InterpolatedPosition", PositionMode::InterpolatedPositionMode}
    };

    const std::map<std::string, VelocityMode> velocityMap_ = {
        {"ProfileVelocity", VelocityMode::ProfileVelocityMode},
        {"CyclicSyncVelocity", VelocityMode::CyclicSyncVelocityMode},
        {"VelocityMode", VelocityMode::VelocityMode}
    };

    const std::map<std::string, TorqueMode> torqueMap_ = {
        {"ProfileTorque", TorqueMode::ProfileTorqueMode},
        {"CyclicSyncTorque", TorqueMode::CyclicSyncTorqueMode}
    };
};

}  // namespace crf::actuators::robot
