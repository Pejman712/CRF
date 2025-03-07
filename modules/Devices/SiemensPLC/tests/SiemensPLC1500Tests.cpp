/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Krzysztof Szczurek CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <string>
#include <typeinfo>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "SiemensPLC/SiemensPLCS7.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

using crf::devices::siemensplc::SiemensPLCTypeConverter;

/* Please configure the S7-1500 PLC:
    IP address: 128.141.94.122, rack 0, slot 1
    Create DB 2 with direct addressing and registers:
    Type    Address:
    Bool    0.0
    Bool    0.1
    Bool    0.2
    Bool    0.3
    Bool    0.4
    Bool    0.5
    Bool    0.6
    Bool    0.7
    Byte    1.0
    SInt    2.0
    Word    4.0
    UInt    6.0
    Int     8.0
    DWord   10.0
    UDInt   14.0
    DInt    18.0
    LWord   22.0
    ULInt   30.0
    LInt    38.0
    Real    46.0
    LReal   50.0
*/

class SiemensPLC1500Should: public ::testing::Test {
 protected:
    SiemensPLC1500Should() :
        logger_("SiemensPLC1500Should"),
        ipAddress_("128.141.94.122"),
        rack_(0),
        slot_(1),
        dbNumber_(2),
        plc_(ipAddress_, rack_, slot_) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
     }

    ~SiemensPLC1500Should() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    template<typename T>
    bool isDifferent(T a, T b) {
        if (a != b) {
            logger_->error("Value problem, {} != {}", a, b);
            return true;
        }
        return false;
    }

    template<typename T>
    bool writeReadCompare(crf::devices::siemensplc::RegisterType registerType, T value,
        unsigned int dbNumber_, unsigned int registerOffset) {
        plc_.writeRegister(registerType, value, dbNumber_, registerOffset);
        return !isDifferent(value, boost::any_cast<T>(plc_.readRegister(registerType, dbNumber_,
            registerOffset)));
    }

    crf::utility::logger::EventLogger logger_;
    std::string ipAddress_;
    int rack_;
    int slot_;
    int dbNumber_;
    crf::devices::siemensplc::SiemensPLCS7 plc_;
};

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareBools) {
    ASSERT_TRUE(plc_.initialize());
    bool bools[8] = {true, false, false, true, true, true, true, false};
    for (unsigned int i = 0; i < 7; ++i) {
        plc_.writeRegister(
            crf::devices::siemensplc::RegisterType::R_BOOL, bools[i], dbNumber_, 0, i);
        auto valueBool = plc_.readRegister(
            crf::devices::siemensplc::RegisterType::R_BOOL, dbNumber_, 0, i);
        EXPECT_FALSE(isDifferent(bools[i], boost::any_cast<bool>(valueBool)));
    }
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareByte) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<uint8_t>(crf::devices::siemensplc::RegisterType::R_BYTE,
        0b11100101, dbNumber_, 1));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareSInt) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<int8_t>(crf::devices::siemensplc::RegisterType::R_SINT,
        96, dbNumber_, 2));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareWord) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<uint16_t>(crf::devices::siemensplc::RegisterType::R_WORD,
        0b0111010101110001, dbNumber_, 4));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareUInt) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<uint16_t>(crf::devices::siemensplc::RegisterType::R_UINT,
        280, dbNumber_, 6));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareInt) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<int16_t>(crf::devices::siemensplc::RegisterType::R_INT,
        -558, dbNumber_, 8));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareDWord) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<uint32_t>(crf::devices::siemensplc::RegisterType::R_DWORD,
        0b01110101011100010110010101100101, dbNumber_, 10));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareUDInt) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<uint32_t>(crf::devices::siemensplc::RegisterType::R_UDINT,
        65463, dbNumber_, 14));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareDInt) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<int32_t>(crf::devices::siemensplc::RegisterType::R_DINT,
        -2556, dbNumber_, 18));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareLWord) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<uint64_t>(crf::devices::siemensplc::RegisterType::R_LWORD,
        0b0111010101110001011001010110111010101110001011001010110010100101, dbNumber_, 22));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareULInt) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<uint64_t>(crf::devices::siemensplc::RegisterType::R_ULINT,
        454664, dbNumber_, 30));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareLInt) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<int64_t>(crf::devices::siemensplc::RegisterType::R_LINT,
        -46464, dbNumber_, 38));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareReal) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<float>(crf::devices::siemensplc::RegisterType::R_REAL,
        546.3434, dbNumber_, 46));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareWriteReadCompareLReal) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(writeReadCompare<double>(crf::devices::siemensplc::RegisterType::R_LREAL,
        15353534.3534535, dbNumber_, 50));
}

TEST_F(SiemensPLC1500Should, DISABLED_HardwareCorrectlyReadFromDb) {
    ASSERT_TRUE(plc_.initialize());
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, true,
        2, 0, 0));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, true,
        2, 0, 1));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, true,
        2, 0, 2));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, true,
        2, 0, 3));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, true,
        2, 0, 4));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, false,
        2, 0, 5));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, false,
        2, 0, 6));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BOOL, false,
        2, 0, 7));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_BYTE, 0b11100101,
        2, 1, 0));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_WORD, 0,
        2, 4, 0));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_INT, 0,
        2, 8, 25));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_DWORD, 0,
        2, 10, 25));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_DINT, 0,
        2, 18, 25));
    EXPECT_TRUE(plc_.writeRegister(crf::devices::siemensplc::RegisterType::R_REAL, 0,
        2, 46, 25.356));

    std::string dbBytes = plc_.readDB(2, 18, 0);

    ASSERT_TRUE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 0));
    ASSERT_TRUE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 1));
    ASSERT_TRUE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 2));
    ASSERT_TRUE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 3));
    ASSERT_TRUE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 4));
    ASSERT_FALSE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 5));
    ASSERT_FALSE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 6));
    ASSERT_FALSE(SiemensPLCTypeConverter::getBit(dbBytes, 0, 7));
    ASSERT_EQ(SiemensPLCTypeConverter::getByte(dbBytes, 1), 0b11100101);
    ASSERT_EQ(SiemensPLCTypeConverter::getWord(dbBytes, 2), 0);
    ASSERT_EQ(SiemensPLCTypeConverter::getShort(dbBytes, 4), 25);
    ASSERT_EQ(SiemensPLCTypeConverter::getDWord(dbBytes, 6), 25);
    ASSERT_EQ(SiemensPLCTypeConverter::getInt(dbBytes, 10), 25);
    ASSERT_NEAR(SiemensPLCTypeConverter::getFloat(dbBytes, 14), 25.356, 1e-3);
}
