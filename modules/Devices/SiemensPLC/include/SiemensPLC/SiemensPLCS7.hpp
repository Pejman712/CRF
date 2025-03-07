/* � Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Krzysztof Szczurek CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <mutex>

#include <boost/any.hpp>
#include <Snap7/s7.hpp>

#include "EventLogger/EventLogger.hpp"
#include "SiemensPLC/ISiemensPLC.hpp"

namespace crf {
namespace devices {
namespace siemensplc {

/*
* @brief Class for data exchange with Siemens PLCs: 300 and 1500 using S7 protocol. The basic
*        features are:
*            - Connect/disconnect to/from PLC on the given IP, rack and slot.
*            - Read/write DB registers.
*            - Types supported for S7-300: Bool, Byte, Word, Int, DWord, DInt, Real.
*            - Types supported for S7-1500: Bool, Byte, SInt, Word, UInt, Int, DWord, UDInt,
*              DInt, LWord, ULInt, LInt, Real, LReal.
*/
class SiemensPLCS7 : public ISiemensPLC {
 public:
    SiemensPLCS7(const std::string& plcHostname, unsigned int rack, unsigned int slot);
    ~SiemensPLCS7() override;

    bool initialize() override;
    bool deinitialize() override;
    bool isConnected() override;
    bool writeRegister(crf::devices::siemensplc::RegisterType registerType,
        const boost::any& value, unsigned int dbNumber, unsigned int registerOffset,
        unsigned int bitNumber = 0) override;
    boost::any readRegister(crf::devices::siemensplc::RegisterType registerType,
        unsigned int dbNumber, unsigned int registerOffset, unsigned int bitNumber = 0) override;
    std::string readDB(const unsigned int dbNumber, const size_t length,
        const unsigned int registerOffset = 0) override;

 private:
    std::string plcHostname_;
    unsigned int rack_;
    unsigned int slot_;
    utility::logger::EventLogger logger_;
    TS7Client client_;
    std::mutex mutex_;

    int getTypeSize(crf::devices::siemensplc::RegisterType registerType) const;
};

}  // namespace siemensplc
}  // namespace devices
}  // namespace crf
