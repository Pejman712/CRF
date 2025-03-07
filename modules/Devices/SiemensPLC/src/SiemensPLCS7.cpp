/* � Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Krzysztof Szczurek CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <Snap7/s7.hpp>
#include <boost/any.hpp>

#include "SiemensPLC/SiemensPLCS7.hpp"

namespace crf {
namespace devices {
namespace siemensplc {

SiemensPLCS7::SiemensPLCS7(const std::string& plcHostname, unsigned int rack, unsigned int slot) :
    plcHostname_(plcHostname),
    rack_(rack),
    slot_(slot),
    logger_("SiemensPLCS7"),
    client_(),
    mutex_() {
    logger_->debug("CTor");
}

SiemensPLCS7::~SiemensPLCS7() {
    logger_->debug("DTor");
    deinitialize();
}

bool SiemensPLCS7::initialize() {
    logger_->debug("initialize");
    if (!isConnected()) {
        std::unique_lock<std::mutex> lock(mutex_);
        int result = client_.ConnectTo(plcHostname_.c_str(), rack_, slot_);
        lock.unlock();
        if (result != 0) {
            logger_->error(CliErrorText(result));
            return false;
        }
    }
    logger_->info("Connection to PLC {} successful!", plcHostname_);
    return true;
}

bool SiemensPLCS7::deinitialize() {
    logger_->debug("deinitialize");
    std::unique_lock<std::mutex> lock(mutex_);
    bool disconnectionResult = client_.Disconnect();
    lock.unlock();
    if (!isConnected() || disconnectionResult == 0) {
        logger_->info("Disconnection successful");
        return true;
    }
    logger_->info("Disconnection error");
    return false;
}

bool SiemensPLCS7::isConnected() {
    std::unique_lock<std::mutex> lock(mutex_);
    bool result = client_.Connected();
    lock.unlock();
    return result;
}

bool SiemensPLCS7::writeRegister(crf::devices::siemensplc::RegisterType registerType,
    const boost::any& value, unsigned int dbNumber, unsigned int registerOffset,
    unsigned int bitNumber) {
    logger_->debug("writeRegister");
    boost::any valueTemp = value;
    if (!isConnected()) {
        logger_->error("Client not connected");
        return true;
    }
    int size = getTypeSize(registerType);
    std::unique_ptr<byte[]> buffer{new byte[size]};
    for (int i = 0; i < size; ++i) {
        buffer[i] = 0;
    }
    switch (registerType) {
        case crf::devices::siemensplc::RegisterType::R_BOOL: {
            std::unique_lock<std::mutex> lock(mutex_);
            client_.DBRead(dbNumber, registerOffset, size, buffer.get());
            lock.unlock();
            S7_SetBitAt(buffer.get(), 0, bitNumber, boost::any_cast<bool>(value));
        } break;
        case crf::devices::siemensplc::RegisterType::R_BYTE:  {
            try {
                S7_SetByteAt(buffer.get(), 0, boost::any_cast<uint8_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_BYTE: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_SINT:  {
            try {
                S7_SetSIntAt(buffer.get(), 0, boost::any_cast<int8_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_SINT: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_WORD:  {
            try {
                S7_SetWordAt(buffer.get(), 0, boost::any_cast<uint16_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_WORD: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_UINT:  {
            try {
                S7_SetUIntAt(buffer.get(), 0, boost::any_cast<uint16_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_UINT: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_INT:  {
            try {
                S7_SetIntAt(buffer.get(), 0, boost::any_cast<int16_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_INT: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_DWORD:  {
            try {
                S7_SetDWordAt(buffer.get(), 0, boost::any_cast<uint32_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_DWORD: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_UDINT:  {
            try {
                S7_SetUDIntAt(buffer.get(), 0, boost::any_cast<uint32_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_UDINT: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_DINT:  {
            try {
                S7_SetDIntAt(buffer.get(), 0, boost::any_cast<int32_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_DINT: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_LWORD:  {
            try {
                S7_SetLWordAt(buffer.get(), 0, boost::any_cast<uint64_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_LWORD: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_ULINT:  {
            try {
                S7_SetULIntAt(buffer.get(), 0, boost::any_cast<uint64_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_ULINT: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_LINT:  {
            try {
                S7_SetLIntAt(buffer.get(), 0, boost::any_cast<int64_t>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_LINT: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_REAL:  {
            try {
                float aux = boost::any_cast<float>(valueTemp);
                S7_SetRealAt(buffer.get(), 0, aux);
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_REAL: {}", e.what());
                return false;
            }
        } break;
        case crf::devices::siemensplc::RegisterType::R_LREAL:  {
            try {
                S7_SetLRealAt(buffer.get(), 0, boost::any_cast<double>(value));
            } catch (const std::exception& e) {
                logger_->error("Exception thrown R_LREAL: {}", e.what());
                return false;
            }
        } break;
        default:
            logger_->error("Register type problem");
            return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    int result = client_.DBWrite(dbNumber, registerOffset, size, buffer.get());
    if (result != 0) {
        logger_->error("DB writing problem: {}", CliErrorText(result));
        client_.Disconnect();
        lock.unlock();
        return false;
    }
    lock.unlock();
    return true;
}

boost::any SiemensPLCS7::readRegister(crf::devices::siemensplc::RegisterType registerType,
    unsigned int dbNumber, unsigned int registerOffset, unsigned int bitNumber) {
    logger_->debug("readRegister");
    if (!isConnected()) {
        logger_->error("Client not connected");
        return boost::any();
    }
    int size = getTypeSize(registerType);
    std::unique_ptr<byte[]> buffer{new byte[size]};
    for (int i = 0; i < size; ++i) {
        buffer[i] = 0;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    int result = client_.DBRead(dbNumber, registerOffset, size, buffer.get());
    if (result != 0) {
        logger_->error("DB reading problem: {}", CliErrorText(result));
        client_.Disconnect();
        lock.unlock();
        return boost::any();
    }
    lock.unlock();

    switch (registerType) {
        case crf::devices::siemensplc::RegisterType::R_BOOL:
            return S7_GetBitAt(buffer.get(), 0, bitNumber);
        case crf::devices::siemensplc::RegisterType::R_BYTE:
            return S7_GetByteAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_SINT:
            return S7_GetSIntAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_WORD:
            return S7_GetWordAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_UINT:
            return S7_GetUIntAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_INT:
            return S7_GetIntAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_DWORD:
            return S7_GetDWordAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_UDINT:
            return S7_GetUDIntAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_DINT:
            return S7_GetDIntAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_LWORD:
            return S7_GetLWordAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_ULINT:
            return S7_GetULIntAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_LINT:
            return S7_GetLIntAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_REAL:
            return S7_GetRealAt(buffer.get(), 0);
        case crf::devices::siemensplc::RegisterType::R_LREAL:
            return S7_GetLRealAt(buffer.get(), 0);
        default:
            logger_->error("Register type problem");
            return boost::any();
    }
}

std::string SiemensPLCS7::readDB(const unsigned int dbNumber, const size_t length,
    const unsigned int registerOffset) {
    logger_->debug("readDB");

    if (!isConnected()) {
        logger_->error("Client not connected");
        return std::string();
    }
    std::unique_ptr<byte[]> buffer{new byte[length]};
    std::memset(buffer.get(), 0, length);

    std::unique_lock<std::mutex> lock(mutex_);
    int ret = client_.DBRead(dbNumber, registerOffset, length, buffer.get());
    if (ret != 0) {
        logger_->error("DB reading problem: {}", CliErrorText(ret));
        client_.Disconnect();
        lock.unlock();
        return std::string();
    }
    lock.unlock();
    return std::string(reinterpret_cast<char*>(buffer.get()), length);
}

int SiemensPLCS7::getTypeSize(RegisterType registerType) const {
    switch (registerType) {
        case crf::devices::siemensplc::RegisterType::R_BOOL:
            return 1;
        case crf::devices::siemensplc::RegisterType::R_BYTE:
            return 1;
        case crf::devices::siemensplc::RegisterType::R_SINT:
            return 1;
        case crf::devices::siemensplc::RegisterType::R_WORD:
            return 2;
        case crf::devices::siemensplc::RegisterType::R_UINT:
            return 2;
        case crf::devices::siemensplc::RegisterType::R_INT:
            return 2;
        case crf::devices::siemensplc::RegisterType::R_DWORD:
            return 4;
        case crf::devices::siemensplc::RegisterType::R_UDINT:
            return 4;
        case crf::devices::siemensplc::RegisterType::R_DINT:
            return 4;
        case crf::devices::siemensplc::RegisterType::R_LWORD:
            return 8;
        case crf::devices::siemensplc::RegisterType::R_ULINT:
            return 8;
        case crf::devices::siemensplc::RegisterType::R_LINT:
            return 8;
        case crf::devices::siemensplc::RegisterType::R_REAL:
            return 4;
        case crf::devices::siemensplc::RegisterType::R_LREAL:
            return 8;
        default:
            logger_->error("Register type problem");
            return 0;
    }
}

}  // namespace siemensplc
}  // namespace devices
}  // namespace crf
