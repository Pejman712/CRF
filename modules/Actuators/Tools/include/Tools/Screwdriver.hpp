#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include <boost/any.hpp>
#include <boost/optional.hpp>

#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Tools/ITool.hpp"

namespace crf {
namespace devices {
namespace tools {

class Screwdriver : public ITool {
 public:
    Screwdriver() = delete;
    Screwdriver(std::shared_ptr<ethercatdevices::IEtherCATMotor> motor, int motorIndex);
    Screwdriver(const Screwdriver&) = delete;
    Screwdriver(Screwdriver&&) = delete;
    ~Screwdriver() override;

    bool initialize() override;
    bool deinitialize() override;

    bool setValue(const std::string& name, bool value) override;
    bool setValue(const std::string& name, int value) override;
    bool setValue(const std::string& name, float value) override;

    boost::optional<boost::any> getValue(const std::string& name) const override;
    boost::optional<ToolValueTypes> getValueType(const std::string& name) override;
    std::vector<std::string> getValueNames() override;

 private:
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<ethercatdevices::IEtherCATMotor> ecMotor_;
    int motorIndex_;
    std::atomic<bool> initialized_;
    std::atomic<bool> startWatchdog_;
    std::atomic<bool> resetWatchdog_;
    std::thread threadWatchdog_;

    void watchdog();
};

}  // namespace tools
}  // namespace devices
}  // namespace crf
