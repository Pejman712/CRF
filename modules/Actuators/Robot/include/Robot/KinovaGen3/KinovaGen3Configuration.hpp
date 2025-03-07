/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

#include <string>
#include <vector>

#include "Robot/RobotConfiguration.hpp"

namespace crf::actuators::robot {

class KinovaGen3Configuration : public RobotConfiguration {
 public:
    explicit KinovaGen3Configuration(const nlohmann::json& robotJSON);
    explicit KinovaGen3Configuration(const std::string&) = delete;
    ~KinovaGen3Configuration() override = default;

    /**
     * @brief Get the robot User Name
     *
     * @return std::string
     */
    std::string getUsername() const;

    /**
     * @brief Get the robot Password
     *
     * @return std::string
     */
    std::string getPassword() const;

    /**
     * @brief Get IP Address
     *
     * @return std::string
     */
    std::string getIPAddress() const;

    /**
     * @brief get TCP port
     *
     * @return uint32_t
     */
    uint32_t getTCPPort() const;

    /**
     * @brief Get UDP Port
     *
     * @return uint32_t
     */
    uint32_t getUDPPort() const;

    /**
     * @brief Get the Session Timeout value
     *
     * @return uint32_t
     */
    uint32_t getSessionTimeout() const;

    /**
     * @brief Get the Connection Timeout value
     *
     * @return uint32_t
     */
    uint32_t getConnectionTimeout() const;

 private:
    void parse(const nlohmann::json& robotJSON);
    void parse(const std::string&) = delete;

    std::string username_;
    std::string password_;
    std::string ipAddress_;
    uint32_t tcpPort_;
    uint32_t udpPort_;
    uint32_t sessionTimeout_;
    uint32_t connectionTimeout_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::robot
