#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <vector>

#include "CanOpenDevices/CanOpenIOs/ICanOpenIOModule.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Tools/ITool.hpp"

namespace crf {
namespace devices {
namespace tools {

class CuttingScissors : public ITool {
 public:
    CuttingScissors() = delete;
    CuttingScissors(std::shared_ptr<canopendevices::ICanOpenIOModule> ioModule, int relayIndex);
    CuttingScissors(const CuttingScissors&) = delete;
    CuttingScissors(CuttingScissors&&) = delete;
    ~CuttingScissors() override;

    bool initialize() override;
    bool deinitialize() override;

    bool setValue(const std::string& name, bool value) override;
    bool setValue(const std::string& name, int value) override;
    bool setValue(const std::string& name, float value) override;

    boost::optional<boost::any> getValue(const std::string& name) const override;
    boost::optional<ToolValueTypes> getValueType(const std::string& name) override;
    std::vector<std::string> getValueNames() override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<canopendevices::ICanOpenIOModule> ioModule_;
    int relayIndex_;
};

}  // namespace tools
}  // namespace devices
}  // namespace crf
