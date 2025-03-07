/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>

#include "Robot/RobotConfiguration.hpp"

namespace crf::actuators::robot {

class CombinedRobotConfiguration : public RobotConfiguration {
 public:
    explicit CombinedRobotConfiguration(const nlohmann::json& robotConfig);
    explicit CombinedRobotConfiguration(const std::string&) = delete;
    ~CombinedRobotConfiguration() override = default;

    /**
     * @brief Get the Robot Config Files object
     *
     * @return std::vector<nlohmann::json>
     */
    std::vector<nlohmann::json> getRobotConfigFiles() const;

    /**
     * @brief Get the Number Of Robots
     *
     * @return uint64_t
     */
    uint64_t getNumberOfRobots() const;

    /**
     * @brief Get the Joint Dimensions Of the Robots
     *
     * @return std::vector<uint64_t>
     */
    std::vector<uint64_t> getJointDimensionsOfRobots() const;

    /**
     * @brief Get the Task Dimensions Of the Robots
     *
     * @return std::vector<uint64_t>
     */
    std::vector<uint64_t> getTaskDimensionsOfRobots() const;

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

    /**
     * @brief Small function to simplify code. It generated a sub vector from a bigger
     * vector.
     * The subvector will have the structure as follows:
     *     vec[start], vec[start+1 ], ...., vec[start + count]
     *
     * @param vec vector from which create the subvector
     * @param start left index where the vector starts
     * @param count subvector size to add from start
     * @return std::vector<double>
     */
    std::vector<double> getSubVector(
        const std::vector<double>& vec, const uint64_t& start, const uint64_t& count);

    std::vector<nlohmann::json> robotJSONs_;
    uint64_t numberOfRobots_;
    std::vector<uint64_t> jointDimensions_;
    std::vector<uint64_t> taskDimensions_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::robot
