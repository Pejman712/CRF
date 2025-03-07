/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/SoemApiMock.hpp"
#include "EtherCATDevices/SoemSimulator.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::SoemApiMock;

class EtherCATManagerShould : public ::testing::Test {
 protected:
    EtherCATManagerShould() :
        logger_("EtherCATManagerShould"),
        soemMock_(new NiceMock<SoemApiMock>),
        soem_(new SoemSimulator(soemMock_, 2)),
        sut_(),
        numberOfSlaves_(2),
        failMotorStateId_(-1),
        ioMapSize_(4096) {
            logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~EtherCATManagerShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<SoemApiMock> soemMock_;
    std::unique_ptr<SoemSimulator> soem_;
    std::unique_ptr<EtherCATManager> sut_;

    int numberOfSlaves_;
    int failMotorStateId_;
    int ioMapSize_;
};

TEST_F(EtherCATManagerShould, throwsExceptionIfSlavesLessThan1) {
    ASSERT_THROW(sut_.reset(new EtherCATManager("ifname", 0, 4096, 0, soemMock_)),
                std::invalid_argument);
}

TEST_F(EtherCATManagerShould, initializeDeinitializeSequence) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(EtherCATManagerShould, initializeFailsIfCantInitSoem) {
    ON_CALL(*soemMock_, config_init(false)).WillByDefault(Return(0));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(EtherCATManagerShould, initializeFailsIfCantConfigInit) {
    ON_CALL(*soemMock_, config_init(false)).WillByDefault(Return(0));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(EtherCATManagerShould, initializeFailsIfDifferentSlavesWhereFound) {
    ON_CALL(*soemMock_, ec_slavecount()).WillByDefault(Return(numberOfSlaves_+1));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(EtherCATManagerShould, initializeFailsIfOneSlaveDoesNotGoPreOperational) {
    ON_CALL(*soemMock_, statecheck(_, _, _)).WillByDefault(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[0].state = 1;
                return 1;
    }));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(EtherCATManagerShould, correctlyConfigureIoMap) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->configureIOMap());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(EtherCATManagerShould, failsConfigureIoMapBiggerSize) {
    ON_CALL(*soemMock_, config_map(_)).WillByDefault(Return(8192));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->configureIOMap());
}

TEST_F(EtherCATManagerShould, failsToSetInSafeOperationalState) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_TRUE(sut_->initialize());
    ON_CALL(*soemMock_, statecheck(_, _, _)).WillByDefault(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[0].state = 2;
                return 2;
    }));
    ASSERT_FALSE(sut_->configureIOMap());
}

TEST_F(EtherCATManagerShould, failsSomeOperationsIfIOMapNotConfiguredAndOthers) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_FALSE(sut_->retrieveOutputs(1));
    ASSERT_FALSE(sut_->retrieveInputs(1));
    ASSERT_EQ(sut_->getSlaveState(1), 0);
    ASSERT_FALSE(sut_->enterInit(1));
    ASSERT_FALSE(sut_->enterPreOp(1));
    ASSERT_FALSE(sut_->enterSafeOp(1));
    ASSERT_FALSE(sut_->enterOp(1));
    ASSERT_FALSE(sut_->slaveCommunicationCheck(1));
    ASSERT_FALSE(sut_->checkInit(1));
    ASSERT_FALSE(sut_->checkPreOp(1));
    ASSERT_FALSE(sut_->checkSafeOp(1));
    ASSERT_FALSE(sut_->checkOp(1));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->retrieveOutputs(1));
    ASSERT_FALSE(sut_->retrieveInputs(1));
    ASSERT_FALSE(sut_->retrieveOutputs(3));
    ASSERT_FALSE(sut_->retrieveInputs(3));
    ASSERT_FALSE(sut_->retrieveOutputs(0));
    ASSERT_FALSE(sut_->retrieveInputs(0));
    ASSERT_EQ(sut_->getSlaveState(3), 0);
    ASSERT_FALSE(sut_->enterInit(3));
    ASSERT_FALSE(sut_->enterPreOp(3));
    ASSERT_FALSE(sut_->enterSafeOp(3));
    ASSERT_FALSE(sut_->enterOp(3));
    ASSERT_FALSE(sut_->slaveCommunicationCheck(3));
    ASSERT_FALSE(sut_->slaveCommunicationCheck(0));
    ASSERT_FALSE(sut_->checkInit(3));
    ASSERT_FALSE(sut_->checkPreOp(3));
    ASSERT_FALSE(sut_->checkSafeOp(3));
    ASSERT_FALSE(sut_->checkOp(3));
}

TEST_F(EtherCATManagerShould, readSdoFails) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ON_CALL(*soemMock_, SDOread(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->readSDO<int32_t>(0x01, 0x6000, 0x00));
}

TEST_F(EtherCATManagerShould, writeSdoFails) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->writeSDO<int32_t>(1, 0x6000, 0x00, 32));
}

TEST_F(EtherCATManagerShould, enterInitWorksAndFails) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->enterInit(1));
    ASSERT_TRUE(sut_->enterInit(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillRepeatedly(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = 0;
                return 0;
    }));
    ASSERT_FALSE(sut_->enterInit(1));
    ASSERT_FALSE(sut_->enterInit(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillOnce(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = 0;
                return 0;
    })).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->enterInit(1));
    ASSERT_TRUE(sut_->enterInit(1));
}

TEST_F(EtherCATManagerShould, enterPreOpWorksAndFails) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->enterPreOp(1));
    ASSERT_TRUE(sut_->enterPreOp(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillRepeatedly(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = 0;
                return 0;
    }));
    ASSERT_FALSE(sut_->enterPreOp(1));
    ASSERT_FALSE(sut_->enterPreOp(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillOnce(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = 0;
                return 0;
    })).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->enterPreOp(1));
    ASSERT_TRUE(sut_->enterPreOp(1));
}

TEST_F(EtherCATManagerShould, enterSafeOpWorksAndFails) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->enterSafeOp(1));
    ASSERT_TRUE(sut_->enterSafeOp(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillRepeatedly(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = 0;
                return 0;
    }));
    ASSERT_FALSE(sut_->enterSafeOp(1));
    ASSERT_FALSE(sut_->enterSafeOp(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillOnce(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = 0;
                return 0;
    })).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->enterSafeOp(1));
    ASSERT_TRUE(sut_->enterSafeOp(1));
}

TEST_F(EtherCATManagerShould, enterOpWorksAndFails) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->enterOp(1));
    ASSERT_TRUE(sut_->enterOp(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillRepeatedly(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = EC_STATE_SAFE_OP;
                return 0;
    }));
    ASSERT_FALSE(sut_->enterOp(1));
    ASSERT_FALSE(sut_->enterOp(1));
    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillOnce(Invoke([this]
    (uint16_t slave, uint16_t reqstate, int timeout) {
                soem_->ec_slave_[1].state = EC_STATE_SAFE_OP;
                return 0;
    })).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->enterOp(1));
    ASSERT_TRUE(sut_->enterOp(1));
}

TEST_F(EtherCATManagerShould, allCheckStateTests) {
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->enterInit(1));
    ASSERT_TRUE(sut_->checkInit(1));
    ASSERT_FALSE(sut_->checkPreOp(1));
    ASSERT_FALSE(sut_->checkSafeOp(1));
    ASSERT_FALSE(sut_->checkOp(1));

    ASSERT_TRUE(sut_->enterPreOp(1));
    ASSERT_FALSE(sut_->checkInit(1));
    ASSERT_TRUE(sut_->checkPreOp(1));
    ASSERT_FALSE(sut_->checkSafeOp(1));
    ASSERT_FALSE(sut_->checkOp(1));

    ASSERT_TRUE(sut_->enterSafeOp(1));
    ASSERT_FALSE(sut_->checkInit(1));
    ASSERT_FALSE(sut_->checkPreOp(1));
    ASSERT_TRUE(sut_->checkSafeOp(1));
    ASSERT_FALSE(sut_->checkOp(1));

    ASSERT_TRUE(sut_->enterOp(1));
    ASSERT_FALSE(sut_->checkInit(1));
    ASSERT_FALSE(sut_->checkPreOp(1));
    ASSERT_FALSE(sut_->checkSafeOp(1));
    ASSERT_TRUE(sut_->checkOp(1));
}

TEST_F(EtherCATManagerShould, correcltyDetectLostSlave) {
    std::atomic<int> state(0);
    EXPECT_CALL(*soemMock_, receive_processdata(_)).WillRepeatedly(
            Invoke([this, &state](int) {
                if (state == 1) {
                    soem_->ec_slave_[1].state = EC_STATE_NONE;
                }
                return 3;
    }));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->configureIOMap());
    ASSERT_TRUE(sut_->enterOp());
    // This sleep has to stay even if it's a test.
    // If it fails, it means that the realtime thread is not working correctly.
    state = 1;

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_CALL(*soemMock_, recover_slave(_, _)).Times(AtLeast(1));
    ASSERT_TRUE(soem_->ec_slave_[1].islost);
}

TEST_F(EtherCATManagerShould, correctlyAcknowledgeError) {
    std::atomic<int> state(0);
    EXPECT_CALL(*soemMock_, receive_processdata(_)).WillRepeatedly(
            Invoke([this, &state](int) {
                if (state == 1) {
                    soem_->ec_slave_[1].state = EC_STATE_SAFE_OP + EC_STATE_ERROR;
                }
                return 3;
    }));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->configureIOMap());
    ASSERT_TRUE(sut_->enterOp());
    state = 1;
    // This sleep has to stay even if it's a test.
    // If it fails, it means that the realtime thread is not working correctly.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_CALL(*soemMock_, writestate(1)).Times(AtLeast(1)).WillOnce(Invoke([this](int){
                soem_->ec_slave_[1].state = EC_STATE_SAFE_OP;
                return 3;
    })).WillRepeatedly(DoDefault());
    EXPECT_CALL(*soemMock_, receive_processdata(_)).WillRepeatedly(DoDefault());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_EQ(soem_->ec_slave_[1].state, EC_STATE_OPERATIONAL);
}

// Test disabled due to a constant file. Needs to be checked
TEST_F(EtherCATManagerShould, DISABLED_correctlyBringSlaveToOperational) {
    std::atomic<int> state(0);
    EXPECT_CALL(*soemMock_, receive_processdata(_)).WillRepeatedly(
            Invoke([this, &state](int) {
                if (state == 1) {
                    soem_->ec_slave_[1].state = EC_STATE_SAFE_OP;
                }
                return 3;
    }));
    ASSERT_NO_THROW(sut_.reset(new EtherCATManager("ifname", 2, 4096, 0, soemMock_)));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->configureIOMap());
    ASSERT_TRUE(sut_->enterOp());
    state = 1;

    // This sleep has to stay even if it's a test.
    // If it fails, it means that the realtime thread is not working correctly.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_CALL(*soemMock_, receive_processdata(_)).WillRepeatedly(DoDefault());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(soem_->ec_slave_[1].state, EC_STATE_OPERATIONAL);
}
