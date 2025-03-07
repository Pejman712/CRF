/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <string>

#include "Snap7/s7.hpp"

#include "EventLogger/EventLogger.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

using crf::devices::siemensplc::SiemensPLCTypeConverter;

class SiemensPLCTypeConverterShould: public ::testing::Test {
 protected:
    SiemensPLCTypeConverterShould() :
        logger_("SiemensPLCTypeConverterShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~SiemensPLCTypeConverterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(SiemensPLCTypeConverterShould, throwExceptionOnBadBuffer) {
    std::string buffer;
    std::string bufferOffset;
    bufferOffset.resize(12);
    ASSERT_THROW(SiemensPLCTypeConverter::getBit(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getBit(bufferOffset, 0, 13), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getBit(bufferOffset, 8, 1), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getByte(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getByte(bufferOffset, 12), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getSInt(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getSInt(bufferOffset, 12), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getWord(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getWord(bufferOffset, 11), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getShort(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getShort(bufferOffset, 11), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getUShort(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getUShort(bufferOffset, 11), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getDWord(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getDWord(bufferOffset, 9), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getUInt(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getUInt(bufferOffset, 9), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getInt(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getInt(bufferOffset, 9), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getLWord(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getLWord(bufferOffset, 5), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getULong(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getULong(bufferOffset, 5), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getLong(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getLong(bufferOffset, 5), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getFloat(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getFloat(bufferOffset, 9), std::invalid_argument);

    ASSERT_THROW(SiemensPLCTypeConverter::getDouble(buffer, 0), std::invalid_argument);
    ASSERT_THROW(SiemensPLCTypeConverter::getDouble(bufferOffset, 5), std::invalid_argument);
}

TEST_F(SiemensPLCTypeConverterShould, correctlyConvertAllTypes) {
    char buffer[8];
    S7_SetBitAt(reinterpret_cast<byte*>(buffer), 0, 0, true);
    std::string bufferStr(buffer, 8);
    ASSERT_TRUE(SiemensPLCTypeConverter::getBit(bufferStr, 0));

    S7_SetByteAt(reinterpret_cast<byte*>(buffer), 0, 0xFF);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getByte(bufferStr), 0xFF);

    S7_SetSIntAt(reinterpret_cast<byte*>(buffer), 0, 16);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getSInt(bufferStr), 16);

    S7_SetWordAt(reinterpret_cast<byte*>(buffer), 0, 213);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getWord(bufferStr), 213);

    S7_SetIntAt(reinterpret_cast<byte*>(buffer), 0, -542);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getShort(bufferStr), -542);

    S7_SetUIntAt(reinterpret_cast<byte*>(buffer), 0, 1234);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getUShort(bufferStr), 1234);

    S7_SetDWordAt(reinterpret_cast<byte*>(buffer), 0, 0xff34);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getDWord(bufferStr), 0xff34);

    S7_SetUDIntAt(reinterpret_cast<byte*>(buffer), 0, 2138);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getUInt(bufferStr), 2138);

    S7_SetDIntAt(reinterpret_cast<byte*>(buffer), 0, -2138);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getInt(bufferStr), -2138);

    S7_SetLWordAt(reinterpret_cast<byte*>(buffer), 0, 0xff272391);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getLWord(bufferStr), 0xff272391);

    S7_SetULIntAt(reinterpret_cast<byte*>(buffer), 0, 2348273);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getULong(bufferStr), 2348273);

    S7_SetLIntAt(reinterpret_cast<byte*>(buffer), 0, -2348273);
    bufferStr = std::string(buffer, 8);
    ASSERT_EQ(SiemensPLCTypeConverter::getLong(bufferStr), -2348273);

    S7_SetRealAt(reinterpret_cast<byte*>(buffer), 0, 0.123841);
    bufferStr = std::string(buffer, 8);
    ASSERT_NEAR(SiemensPLCTypeConverter::getFloat(bufferStr), 0.123841, 1e-5);

    S7_SetLRealAt(reinterpret_cast<byte*>(buffer), 0, 12.3841);
    bufferStr = std::string(buffer, 8);
    ASSERT_NEAR(SiemensPLCTypeConverter::getDouble(bufferStr), 12.3841, 1e-5);
}
