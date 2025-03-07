/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include "ThermalCamera/OptrisDeviceFactory.hpp"

#include <memory>
#include <string>

#include <libirimager/IRLogger.h>
#include <libirimager/IRImager.h>

namespace crf {
namespace sensors {
namespace thermalcamera {

OptrisDeviceFactory::OptrisDeviceFactory(const std::string& configFile):
    logger_("OptrisDeviceFactory") {
    evo::IRLogger::setVerbosity(evo::IRLOG_ERROR, evo::IRLOG_OFF);
    /**
     * TODO!!!: Do not use evo::IRDeviceParamsReader::readXML, because it has a memory leak
     * (mismatched delete / delete []) upon file access. Better to write our own
     * xml --> params reader
     */
    if (!evo::IRDeviceParamsReader::readXML(configFile.c_str(), params_)) {
        logger_->error("Failed to read params from configFile: {}", configFile);
    }
}

std::shared_ptr<evo::IRDevice> OptrisDeviceFactory::createDevice() {
    return std::shared_ptr<evo::IRDevice>(evo::IRDeviceUVC::createInstance(
        NULL, params_.serial, params_.videoFormatIndex));
}

std::shared_ptr<evo::IRImager> OptrisDeviceFactory::createImager(unsigned int frequency,
    unsigned int width, unsigned int height) {
    auto imager = std::make_shared<evo::IRImager>();
    bool success = imager->init(&params_, frequency, width, height);
    logger_->debug("Imager init finished with {}", success);
    if (!success) {
        return nullptr;
    }
    return imager;
}

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
