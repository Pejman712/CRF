/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/

#pragma once

#include <string>
#include <atomic>
#include <memory>
#include <deque>
#include <mutex>
#include <vector>

#include <nlohmann/json.hpp>
#include "Types/JsonConverters.hpp"

#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "MissionUtility/IDeployableDevice.hpp"

namespace crf::applications::missionmanager::sciencegateway {

class Actions: public utility::commoninterfaces::IInitializable {
 public:
    Actions() = delete;
    explicit Actions(
        std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
        const nlohmann::json& configFile);
    ~Actions();

    bool deinitialize() override;
    bool initialize() override;

    /**
     * @brief Deploys the deployable device with the neccessary checks.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool deploy();
    /**
     * @brief Retracts the deployable device with the neccessary checks.
     *
     * @return true if succeded
     * @return false otherwise
     */
    bool retract();
    /**
     * @brief Get the Status of mission
     *
     * @return nlohmann::json with several fields to send to the operator monitoring the robots
     */
    nlohmann::json getStatus();

 private:
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable_;
    bool initialized_;
    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::applications::missionmanager::sciencegateway
