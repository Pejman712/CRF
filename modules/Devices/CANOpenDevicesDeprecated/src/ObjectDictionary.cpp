/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <fstream>
#include <nlohmann/json.hpp>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "CANOpenDevices/CANOpenDefinitions.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

ObjectDictionary::ObjectDictionary(const std::string& configurationDictionary) :
    logger_("ObjectDictionary"),
    registers_(),
    registersByName_(),
    pdoMappings_(),
    nmtState_(),
    canFrameHandlerMap_(),
    sdoCvFlag_(false),
    sdoResponseResults_(),
    sdoMutex_(),
    sdoConditionVariable_() {
    if (!parseConfiguration(configurationDictionary)) {
        throw std::runtime_error("Failed to parse configuration dictionary");
    }
    handlersInizialization();
}

bool ObjectDictionary::addRegister(const ObjectDictionaryRegister& reg) {
    uint32_t registerNumber = (reg.getIndex() << 16) + reg.getSubindex();
    if (registers_.find(registerNumber) != registers_.end()) {
        logger_->warn("Register already added");
        return false;
    }
    if (registersByName_.find(reg.getName()) != registersByName_.end()) {
        logger_->warn("Register already added");
        return false;
    }
    registers_.insert({registerNumber, reg});
    registersByName_.insert({reg.getName(), registerNumber});
    return true;
}

bool ObjectDictionary::addPDOMapping(uint8_t pdo,
    const std::vector<ObjectDictionaryRegister>& registers) {
    int totalSize = 0;
    for (unsigned int i=0; i < registers.size(); i++) {
        totalSize += registers[i].getSize();
    }
    if (totalSize > 8) {
        logger_->warn("PDO mapping can't be bigger than 8 bytes");
        return false;
    }
    if (pdoMappings_.find(pdo) != pdoMappings_.end()) {
        logger_->info("PDO mapping already inserted, it will be replaced");
        pdoMappings_[pdo] = registers;
    } else {
        pdoMappings_.insert({pdo, registers});
    }
    for (unsigned int i=0; i < registers.size(); i++) {
        uint32_t registerNumber = (registers[i].getIndex() << 16) + registers[i].getSubindex();
        auto iterator = registers_.find(registerNumber);

        if (iterator == registers_.end()) {
            registers_.insert({registerNumber, registers[i]});
        }
    }
    return true;
}

bool ObjectDictionary::setData(const can_frame& cframe) {
    uint8_t functionCode = getFunctionCode(cframe);
    std::map<uint8_t, std::function<bool(ObjectDictionary*, const can_frame&)>>::iterator it;
    it = canFrameHandlerMap_.find(functionCode);
    if (it != canFrameHandlerMap_.end()) {
        return canFrameHandlerMap_[functionCode](this, cframe);
    } else {
        logger_->warn("CAN't handle the frame. Function Code not recognized: {}", functionCode);
        return false;
    }
}

boost::optional<ObjectDictionaryRegister&> ObjectDictionary::getRegister(
    uint16_t registerIndex, uint8_t subRegisterIndex) {
    uint32_t registerNumber = (registerIndex << 16) + subRegisterIndex;

    const std::map<uint32_t, ObjectDictionaryRegister>::iterator& iterator = registers_.find(
        registerNumber);
    if (iterator == registers_.end()) {
        logger_->warn("Requested register does not exists in the object dictionary");
        return boost::none;
    }
    return iterator->second;
}

boost::optional<ObjectDictionaryRegister&> ObjectDictionary::getRegister(const std::string& name) {
    auto iteratorReg = registersByName_.find(name);
    if (iteratorReg == registersByName_.end()) {
        logger_->warn("Requested register does not exists in the object dictionary");
        return boost::none;
    }
    auto iterator = registers_.find(iteratorReg->second);
    return iterator->second;
}

boost::optional<std::pair<ObjectDictionaryRegister&, bool> > ObjectDictionary::waitForSdoResponse(
    const ObjectDictionaryRegister& reg, const std::chrono::milliseconds& timeout) {
    std::unique_lock<std::mutex> lock(sdoMutex_);
    sdoCvFlag_ = false;
    if (sdoResponseResults_.empty()) {
        if (sdoConditionVariable_.wait_for(lock, timeout,
            [=]() {return sdoCvFlag_.load(); }) == false) {
            logger_->warn("Timeout while waiting for SDO");
            return boost::none;
        }
    }
    for (int i = sdoResponseResults_.size() - 1; i >= 0; i--) {
        if (sdoResponseResults_[i].first.getName() == reg.getName()) {
            auto retval = sdoResponseResults_[i];
            sdoResponseResults_.erase(sdoResponseResults_.begin(),
                sdoResponseResults_.begin() + i + 1);
            return retval;
        }
    }
    logger_->error("The requested register was not found in the queue");
    return boost::none;
}

std::vector<ObjectDictionaryRegister> ObjectDictionary::getRegistersVector() {
    std::vector<ObjectDictionaryRegister> returnVector;
    for (auto it = registers_.begin(); it != registers_.end(); ++it) {
        returnVector.push_back(it->second);
    }
    return returnVector;
}

std::pair<uint8_t, std::chrono::high_resolution_clock::time_point> ObjectDictionary::getNMTState() {
    return nmtState_;
}

bool ObjectDictionary::parseConfiguration(const std::string& configurationDictionary) {
    std::ifstream configuration(configurationDictionary);
    if ((configuration.rdstate() & std::ifstream::failbit) != 0) {
        return false;
    }
    nlohmann::json configurationJSON;
    try {
        configuration >> configurationJSON;
        for (auto it = configurationJSON.begin(); it != configurationJSON.end(); ++it) {
            auto name = it->at("name").get<std::string>();
            auto index = static_cast<uint16_t>(std::stoi(
                it->at("index").get<std::string>(), nullptr, 0));
            if (it->find("subregisters") == it->end()) {
                auto size = it->at("size").get<uint8_t>();
                if (!addRegister(ObjectDictionaryRegister(name, size, index))) {
                    return false;
                }
            } else {
                for (auto subIt = it->at("subregisters").begin(); subIt != it->at("subregisters").end(); ++subIt) {  // NOLINT
                    name = it->at("name").get<std::string>() + "/" + subIt->at("name").get<std::string>(); // NOLINT
                    auto subindex = static_cast<uint8_t>(std::stoi(
                        subIt->at("subindex").get<std::string>(), nullptr, 0));
                    auto size = subIt->at("size").get<uint8_t>();
                    if (!addRegister(ObjectDictionaryRegister(name, size, index, subindex))) {
                        logger_->warn("Failed to add {}", name);
                        return false;
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        return false;
    }

    return true;
}

void ObjectDictionary::handlersInizialization() {
    canFrameHandlerMap_.insert({MessageFilters::SDO_RESPONSE, &ObjectDictionary::sdoHandler});
    canFrameHandlerMap_.insert({MessageFilters::PDO1_RX,  &ObjectDictionary::pdoHandler});
    canFrameHandlerMap_.insert({MessageFilters::PDO2_RX, &ObjectDictionary::pdoHandler});
    canFrameHandlerMap_.insert({MessageFilters::PDO3_RX, &ObjectDictionary::pdoHandler});
    canFrameHandlerMap_.insert({MessageFilters::PDO4_RX, &ObjectDictionary::pdoHandler});
    canFrameHandlerMap_.insert({MessageFilters::NMT_HEARTBEAT, &ObjectDictionary::nmtHandler});
}

uint8_t ObjectDictionary::getFunctionCode(const can_frame& cframe) {
    return (cframe.can_id & FrameManagement::FUNCTION_CODE_MASK) >> 7;
}

uint16_t ObjectDictionary::getIndex(const can_frame& cframe) {
    return (cframe.data[1] + (cframe.data[2] << 8));
}

uint8_t ObjectDictionary::getSubIndex(const can_frame& cframe) {
    return cframe.data[3];
}

bool ObjectDictionary::sdoHandler(const can_frame& cframe) {
    uint32_t registerNumber = (getIndex(cframe) << 16) + getSubIndex(cframe);
    const std::map<uint32_t, ObjectDictionaryRegister>::iterator& iterator = registers_.find(
        registerNumber);
    if (iterator == registers_.end()) {
        logger_->warn("Received register does not exists in the object dictionary {}:{}",
            getIndex(cframe), getSubIndex(cframe));
        return false;
    }
    if (cframe.data[0] == 0x80) {
        {
            std::unique_lock<std::mutex> lock(sdoMutex_);
            sdoCvFlag_ = true;
            sdoResponseResults_.push_back({ iterator->second, false });
        }
        sdoConditionVariable_.notify_all();
        return true;
    }
    if (cframe.data[0] != 0x60) {
        int size = iterator->second.getSize();
        std::string valueStr(reinterpret_cast<const char*>(cframe.data + 4), size);
        if (!iterator->second.setValue(valueStr)) {
            logger_->warn("Error setting value");
            return false;
        }
    }
    {
        std::unique_lock<std::mutex> lock(sdoMutex_);
        sdoCvFlag_ = true;
        sdoResponseResults_.push_back({ iterator->second, true });
    }
    sdoConditionVariable_.notify_all();
    return true;
}

bool ObjectDictionary::pdoHandler(const can_frame& cframe) {
    uint8_t functionCode = getFunctionCode(cframe);
    if (pdoMappings_.find(functionCode) == pdoMappings_.end()) {
        logger_->warn("PDO mapping not mapped {}", functionCode);
        return false;
    }
    auto mapping = pdoMappings_[functionCode];
    int dataIndex = 0;
    for (unsigned int i=0; i < mapping.size(); i++) {
        uint32_t registerNumber = (mapping[i].getIndex() << 16) + mapping[i].getSubindex();
        auto iterator = registers_.find(registerNumber);
        int size = iterator->second.getSize();
        std::string valueStr(reinterpret_cast<const char*>(cframe.data + dataIndex), size);
        iterator->second.setValue(valueStr);
        dataIndex += size;
    }
    return true;
}

bool ObjectDictionary::nmtHandler(const can_frame& frame) {
    nmtState_.first = frame.data[0];
    nmtState_.second = std::chrono::high_resolution_clock::now();
    return true;
}

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
