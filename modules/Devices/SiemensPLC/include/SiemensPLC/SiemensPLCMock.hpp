/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "gmock/gmock.h"
#include <boost/any.hpp>

#include "SiemensPLC/ISiemensPLC.hpp"
#include "SiemensPLC/RegisterType.hpp"

namespace crf {
namespace devices {
namespace siemensplc {

class SiemensPLCMock : public ISiemensPLC {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));
    MOCK_METHOD(bool, isConnected, (), (override));
    MOCK_METHOD(bool, writeRegister, (crf::devices::siemensplc::RegisterType registerType,
        const boost::any& value, unsigned int dbNumber, unsigned int registerOffset,
        unsigned int bitNumber), (override));
    MOCK_METHOD(boost::any, readRegister, (crf::devices::siemensplc::RegisterType registerType,
        unsigned int dbNumber, unsigned int registerOffset, unsigned int bitNumber), (override));
    MOCK_METHOD(std::string, readDB, (const unsigned int dbNumber, const size_t length,
        const unsigned int registerOffset), (override));
};

}  // namespace siemensplc
}  // namespace devices
}  // namespace crf
