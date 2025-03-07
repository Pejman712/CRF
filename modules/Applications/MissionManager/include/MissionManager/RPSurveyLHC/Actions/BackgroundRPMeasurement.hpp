/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

#include <condition_variable>
#include <string>
#include <atomic>
#include <thread>
#include <memory>

#include <nlohmann/json.hpp>

#include "CommonInterfaces/IInitializable.hpp"
#include "MissionUtility/IDeployableDevice.hpp"
#include "RPSensor/IRPSensor.hpp"
#include "TIM/ITIM.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/JsonConverters.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

class BackgroundRPMeasurement: public utility::commoninterfaces::IInitializable {
 public:
    explicit BackgroundRPMeasurement(
        std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor,
        std::shared_ptr<crf::actuators::tim::ITIM> tim,
        std::shared_ptr<crf::utility::missionutility::IDeployableDevice> delpoyableArm,
        const nlohmann::json& configFile);
    ~BackgroundRPMeasurement();

    bool deinitialize() override;
    bool initialize() override;

    /**
     * @brief Start the measurement of radiation asynchronously and save them.
     *
     */
    void start();
    /**
     * @brief Stop the measurement of radiation.
     *
     */
    void stop();

 private:
    void execute();

    std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor_;
    std::shared_ptr<crf::actuators::tim::ITIM> tim_;
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployableDevice_;

    std::string pathForResults_;
    std::atomic<bool> stopThreads_;
    std::atomic<bool> measuring_;
    std::thread measurementThread_;
    std::condition_variable cv_;

    utility::logger::EventLogger logger_;
};

}  // namespace crf::applications::missionmanager::rpsurveylhc
