/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include "Tools/CuttingScissors.hpp"

namespace crf {
namespace devices {
namespace tools {

CuttingScissors::CuttingScissors(
    std::shared_ptr<canopendevices::ICanOpenIOModule> ioModule, int relayIndex) :
        logger_("CuttingScissors"),
        ioModule_(ioModule),
        relayIndex_(relayIndex) {
            logger_->debug("CTor");
}

CuttingScissors::~CuttingScissors() {
    logger_->debug("DTor");
    deinitialize();
}

bool CuttingScissors::initialize() {
    logger_->debug("initialize");
    return ioModule_->initialize();
}

bool CuttingScissors::deinitialize() {
    logger_->debug("deinitialize");
    return ioModule_->deinitialize();
}

bool CuttingScissors::setValue(const std::string& name, bool value) {
    if (name != "close") {
        return false;
    }

    return ioModule_->setDigitalOutputState(relayIndex_, value);
}

bool CuttingScissors::setValue(const std::string& name, int value) {
    // Int values are not supported for this tool
    return false;
}

bool CuttingScissors::setValue(const std::string& name, float value) {
    // Float values are not supported for this tool
    return false;
}

boost::optional<boost::any> CuttingScissors::getValue(const std::string& name) const {
    if (name != "closing") {
        return boost::none;
    }
    auto value = ioModule_->getDigitalOutputState(relayIndex_);
    if (!value) {
        return boost::none;
    } else {
        return boost::any(value.value());
    }
}

boost::optional<ToolValueTypes> CuttingScissors::getValueType(const std::string& name) {
    if (name == "closing") {
        return ToolValueTypes::BOOL;
    } else if (name == "close") {
        return ToolValueTypes::BOOL;
    }

    return boost::none;
}

std::vector<std::string> CuttingScissors::getValueNames() {
    return std::vector<std::string>({"closing"});
}

}  // namespace tools
}  // namespace devices
}  // namespace crf
