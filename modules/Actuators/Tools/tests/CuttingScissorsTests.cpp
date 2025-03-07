/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "Tools/CuttingScissors.hpp"

#include "Mocks/Devices/CanOpenIOModuleMock.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::Invoke;

using crf::devices::tools::CuttingScissors;
using crf::devices::canopendevices::CanOpenIOModuleMock;

class CuttingScissorsShould : public ::testing::Test {
 protected:
    CuttingScissorsShould() :
        logger_("CuttingScissorsShould"),
        ioInitialized_(false),
        ioModule_(new NiceMock<CanOpenIOModuleMock>) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~CuttingScissorsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        ON_CALL(*ioModule_, initialize()).WillByDefault(Invoke([this]() {
            if (!ioInitialized_) {
                ioInitialized_ = true;
                return true;
            } else {
                return false;
            }
        }));

        ON_CALL(*ioModule_, deinitialize()).WillByDefault(Invoke([this]() {
            if (ioInitialized_) {
                ioInitialized_ = false;
                return true;
            } else {
                return false;
            }
        }));
    }

    crf::utility::logger::EventLogger logger_;

    bool ioInitialized_;
    std::shared_ptr<CanOpenIOModuleMock> ioModule_;
    std::unique_ptr<CuttingScissors> sut_;
};

TEST_F(CuttingScissorsShould, initializeDeinitializeSequence) {
    sut_.reset(new CuttingScissors(ioModule_, 0));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}


TEST_F(CuttingScissorsShould, failsToSetAndGetUnknownValues) {
    sut_.reset(new CuttingScissors(ioModule_, 0));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setValue("ciao", false));
    ASSERT_FALSE(sut_->setValue("ciao", 1));
    ASSERT_FALSE(sut_->setValue("ciao", 1.0f));

    ASSERT_FALSE(sut_->getValue("ciao"));
    ASSERT_FALSE(sut_->getValueType("ciao"));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CuttingScissorsShould, correctlyGetValueTypeAndNames) {
    sut_.reset(new CuttingScissors(ioModule_, 0));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getValueType("closing"), crf::devices::tools::ToolValueTypes::BOOL);
    ASSERT_EQ(sut_->getValueType("close"), crf::devices::tools::ToolValueTypes::BOOL);
    ASSERT_EQ(sut_->getValueNames().size(), 1);
    ASSERT_EQ(sut_->getValueNames()[0], "closing");
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CuttingScissorsShould, correctlyGetAndSetValue) {
    EXPECT_CALL(*ioModule_, getDigitalOutputState(0)).WillRepeatedly(Return(true));
    EXPECT_CALL(*ioModule_, setDigitalOutputState(0, _)).WillRepeatedly(Return(true));

    sut_.reset(new CuttingScissors(ioModule_, 0));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setValue("close", true));
    ASSERT_TRUE(boost::any_cast<bool>(sut_->getValue("closing").get()));
    ASSERT_TRUE(sut_->deinitialize());
}
