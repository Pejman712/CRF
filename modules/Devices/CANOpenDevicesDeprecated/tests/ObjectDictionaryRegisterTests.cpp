/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "CANOpenDevices/ObjectDictionaryRegister.hpp"

using testing::_;
using testing::Return;

using crf::devices::canopendevices::ObjectDictionaryRegister;

class ObjectDictionaryRegisterShould : public ::testing::Test {
 protected:
    ObjectDictionaryRegisterShould() :
        logger_("ObjectDictionaryRegisterShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~ObjectDictionaryRegisterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(ObjectDictionaryRegisterShould, correctlySetConstructorValues) {
    ObjectDictionaryRegister reg("targetVelocity", 4, 0x4FB3, 0x12);

    ASSERT_EQ(reg.getName(), "targetVelocity");
    ASSERT_EQ(reg.getSize(), 4);
    ASSERT_EQ(reg.getIndex(), 0x4FB3);
    ASSERT_EQ(reg.getSubindex(), 0x12);
}

TEST_F(ObjectDictionaryRegisterShould, correctlyCopyObject) {
    ObjectDictionaryRegister reg("targetVelocity", 4, 0x4FB3, 0x12);
    reg.setValue<int32_t>(250);

    ObjectDictionaryRegister reg2(reg);

    ASSERT_EQ(reg2.getName(), reg.getName());
    ASSERT_EQ(reg2.getSize(), reg.getSize());
    ASSERT_EQ(reg2.getIndex(), reg.getIndex());
    ASSERT_EQ(reg2.getSubindex(), reg.getSubindex());
    ASSERT_EQ(reg2.getValue<int32_t>(), reg.getValue<int32_t>());
}

TEST_F(ObjectDictionaryRegisterShould, failsToBuildWithWrongSizes) {
    ASSERT_THROW(ObjectDictionaryRegister reg("targetVelocity", 3, 0x4FB3, 0x12),
        std::runtime_error);
    ASSERT_THROW(ObjectDictionaryRegister reg("targetVelocity", 5, 0x4FB3, 0x12),
        std::runtime_error);

    ASSERT_NO_THROW(ObjectDictionaryRegister reg("targetVelocity", 1, 0x4FB3, 0x12));
    ASSERT_NO_THROW(ObjectDictionaryRegister reg("targetVelocity", 2, 0x4FB3, 0x12));
    ASSERT_NO_THROW(ObjectDictionaryRegister reg("targetVelocity", 4, 0x4FB3, 0x12));
}

TEST_F(ObjectDictionaryRegisterShould, failsWhenTryingToGetAndSetWrongSizeValue) {
    ObjectDictionaryRegister reg("targetVelocity", 4, 0x4FB3, 0x12);
    ASSERT_THROW(reg.getValue<int16_t>(), std::runtime_error);
    ASSERT_THROW(reg.getValue<int8_t>(), std::runtime_error);

    ASSERT_FALSE(reg.setValue<int16_t>(123));
    ASSERT_FALSE(reg.setValue<int8_t>(12));

    std::string buffer("12");
    ASSERT_FALSE(reg.setValue(buffer));
}

TEST_F(ObjectDictionaryRegisterShould, correctlySetAndGetValues) {
    ObjectDictionaryRegister reg("targetVelocity", 4, 0x4FB3, 0x12);

    ASSERT_EQ(reg.getValue<int32_t>(), 0);
    ASSERT_TRUE(reg.setValue<int32_t>(123));
    ASSERT_EQ(reg.getValue<int32_t>(), 123);

    std::string buffer;
    buffer.resize(4);
    int32_t value = 3214;
    std::memcpy(&value, (void*)buffer.c_str(), sizeof(value)); // NOLINT
    ASSERT_TRUE(reg.setValue(buffer));
    ASSERT_EQ(reg.getValue<int32_t>(), value);
}
