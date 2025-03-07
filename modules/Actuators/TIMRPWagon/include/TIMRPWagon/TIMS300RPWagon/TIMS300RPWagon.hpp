/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <optional>
#include <map>
#include <tuple>
#include <chrono>
#include <thread>
#include <condition_variable>

#include <nlohmann/json.hpp>

#include "TIMRPWagon/ITIMRPWagon.hpp"
#include "SiemensPLC/ISiemensPLC.hpp"
#include "TIMRPWagon/TIMRPWagonCommands.hpp"
#include "TIMRPWagon/TIMRPWagonStatus.hpp"
#include "TIMRPWagon/TIMRPWagonConfiguration.hpp"

namespace crf::actuators::timrpwagon {

class TIMS300RPWagon: public ITIMRPWagon {
 public:
    explicit TIMS300RPWagon(const nlohmann::json& configFile,
        std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc = nullptr);
    TIMS300RPWagon(const std::string& timConfigFile,
        std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc) = delete;
    TIMS300RPWagon(const TIMS300RPWagon&) = delete;
    TIMS300RPWagon(TIMS300RPWagon&&) = delete;
    ~TIMS300RPWagon() override;

    bool initialize() override;
    bool deinitialize() override;

    bool isConnected() override;

    RPArmPosition getRPArmPosition() override;
    std::optional<bool> retractRPArm() override;
    std::optional<bool> deployRPArm() override;
    std::optional<bool> moveRPArmUp() override;
    std::optional<bool> moveRPArmDown() override;
    std::optional<bool> stopRPArm() override;
    std::optional<bool> lockRPArm() override;
    std::optional<bool> unlockRPArm() override;
    std::optional<bool> isRPArmInError() override;
    std::optional<bool> resetRPArmDriver() override;
    std::optional<bool> acknowledgeErrors() override;
    std::shared_ptr<TIMRPWagonConfiguration> getConfiguration() override;

 private:
    nlohmann::json configFile_;
    std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc_;
    crf::utility::logger::EventLogger logger_;
    bool isInitialized_;
    std::shared_ptr<crf::actuators::timrpwagon::TIMRPWagonConfiguration> configuration_;
    std::chrono::microseconds commandTimeout_;
    crf::actuators::timrpwagon::TIMRPWagonCommands commandsDB_;
    crf::actuators::timrpwagon::TIMRPWagonStatus statusDB_;

    std::thread updatePLCValuesThread_;
    std::mutex firstGrabberLoopMutex_;
    std::condition_variable firstGrabberLoop_;
    std::atomic<bool> stopUpdatePCLValuesThread_;

    bool updatePLCValues();
    void grabber();
};

}  // namespace crf::actuators::timrpwagon
