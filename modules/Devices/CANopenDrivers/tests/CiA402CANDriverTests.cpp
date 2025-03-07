/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <future>
#include <atomic>
#include <thread>

#include <nlohmann/json.hpp>

#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/vcan.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/coapp/slave.hpp>

#include "CANopenDrivers/CiA301Registers.hpp"
#include "CANopenDrivers/CiA402/CiA402Definitions.hpp"
#include "CANopenDrivers/CiA402/CiA402Registers.hpp"
#include "CANopenDrivers/CiA402/CANDrivers/CiA402CANDriver/CiA402CANDriver.hpp"
#include "CANopenDrivers/CANopenMaster/CANopenMaster.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;

using crf::devices::canopendrivers::CiA301;
using crf::devices::canopendrivers::CiA402;
using crf::devices::canopendrivers::ControlWord;
using crf::devices::canopendrivers::StatusWord;
using crf::devices::canopendrivers::ModeOfOperation;
using crf::devices::canopendrivers::StatusWordMask;
using crf::devices::canopendrivers::ControlWordMask;
using crf::devices::canopendrivers::CiA402CANDriver;
using crf::devices::canopendrivers::ControlWordIPM;
using crf::devices::canopendrivers::StatusWordIPM;
using crf::devices::canopendrivers::CANopenMaster;

class MockSlave : public lely::canopen::BasicSlave {
 public:
    MockSlave(
        lely::io::TimerBase& timer, // NOLINT
        lely::io::CanChannelBase& chan, // NOLINT
        const ::std::string& dcf_txt,
        const ::std::string& dcf_bin,
        uint8_t id):
        lely::canopen::BasicSlave(timer, chan, dcf_txt, dcf_bin, id),
        automaticTransitionFault_(false),
        shutdownFault_(false),
        switchOnFault_(false),
        disableVoltageFault_(false),
        quickStopFault_(false),
        disableOperationFault_(false),
        enableOperationFault_(false),
        logger_("MockSlave - " + std::to_string(id)) {
        logger_->debug("CTor");
    }

    void goIntoFault() {
        (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::FaultReactionActive);
        (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::Fault);
    }

    void goIntoFaultShutdown() {
        shutdownFault_ = true;
    }

    void goIntoFaultSwitchOn() {
        switchOnFault_ = true;
    }

    void goIntoFaultDisableVoltage() {
        disableVoltageFault_ = true;
    }

    void goIntoFaultQuickStop() {
        quickStopFault_ = true;
    }

    void goIntoFaultDisableOperation() {
        disableOperationFault_ = true;
    }

    void goIntoFaultEnableOperation() {
        enableOperationFault_ = true;
    }

    void goIntoFaultAutomaticTransition() {
        automaticTransitionFault_ = true;
    }

    void Config() noexcept {
        if (automaticTransitionFault_) {
            goIntoFault();
            return;
        }
        (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::SwitchONDisabled);
        (*this)[CiA402::SupportedDriveModes][0] = static_cast<uint32_t>(0x03EF);
    }

 protected :
    // This function gets called every time a value is written to the local object
    // dictionary by an SDO or RPDO.
    void OnWrite(uint16_t idx, uint8_t subidx) noexcept override {
        if (idx == CiA402::ControlWord) {
            uint16_t cw = (*this)[CiA402::ControlWord][0];

            if ((cw & ControlWordMask::Shutdown) == ControlWord::Shutdown) {
                if (shutdownFault_) {
                    goIntoFault();
                    return;
                }
                (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::Ready);
            }

            if ((cw & ControlWordMask::SwitchON) == ControlWord::SwitchON) {
                if (switchOnFault_) {
                    goIntoFault();
                    return;
                }
                (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::SwitchedON);
            }

            if ((cw & ControlWordMask::DisableVoltage) == ControlWord::DisableVoltage) {
                if (disableVoltageFault_) {
                    goIntoFault();
                    return;
                }
                (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::SwitchONDisabled);  // NOLINT
            }

            if ((cw & ControlWordMask::QuickStop) == ControlWord::QuickStop) {
                if (quickStopFault_) {
                    goIntoFault();
                    return;
                }
                (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::QuickStopActive);  // NOLINT
            }

            if ((cw & ControlWordMask::DisableOperation) == ControlWord::DisableOperation) {
                if (disableOperationFault_) {
                    goIntoFault();
                    return;
                }
                (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::SwitchedON);
            }
            if ((cw & ControlWordMask::EnableOperation) == ControlWord::EnableOperation) {
                if (enableOperationFault_) {
                    goIntoFault();
                    return;
                }
                (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::OperationEnabled);  // NOLINT
            }

            if ((cw & ControlWordMask::FaultResetActive) == ControlWord::FaultResetActive) {
                (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWord::SwitchONDisabled);  // NOLINT
            }

            int8_t mo = (*this)[CiA402::ModesOfOperationDisplay][0];
            if (mo == ModeOfOperation::InterpolatedPositionMode) {
                if (cw == ControlWordIPM::EnableIPM) {
                    (*this)[CiA402::StatusWord][0] = static_cast<uint16_t>(StatusWordIPM::IPMModeActive);  // NOLINT
                }
            }
        }

        if (idx == CiA402::ModesOfOperation) {
            int8_t mo = (*this)[CiA402::ModesOfOperation][0];
            (*this)[CiA402::ModesOfOperationDisplay][0] = static_cast<int8_t>(mo);
        }

        // CST mode
        if (idx == CiA402::TargetTorque) {
            int16_t tor = (*this)[CiA402::TargetTorque][0];
            int16_t off = (*this)[CiA402::TorqueOffset][0];
            (*this)[CiA402::TorqueActualValue][0] = static_cast<int16_t>(tor + off);
        }

        // CSV mode
        if (idx == CiA402::TargetVelocity) {
            int32_t vel = (*this)[CiA402::TargetVelocity][0];
            int32_t veloff = (*this)[CiA402::VelocityOffset][0];
            (*this)[CiA402::VelocityActualValue][0] = vel + veloff;
        }

        // CSP mode
        if (idx == CiA402::TargetPosition) {
            int32_t pos = (*this)[CiA402::TargetPosition][0];
            int32_t posoff = (*this)[CiA402::PositionOffset][0];
            (*this)[CiA402::PositionActualValue][0] = pos + posoff;
        }
     }

 private:
    std::atomic<bool> automaticTransitionFault_;
    std::atomic<bool> shutdownFault_;
    std::atomic<bool> switchOnFault_;
    std::atomic<bool> disableVoltageFault_;
    std::atomic<bool> quickStopFault_;
    std::atomic<bool> disableOperationFault_;
    std::atomic<bool> enableOperationFault_;

    crf::utility::logger::EventLogger logger_;
};

class CiA402CANDriverShould: public ::testing::Test {
 protected:
    CiA402CANDriverShould():
        io_guard_(),
        ctx_(),
        poll_(ctx_),
        loop_(poll_.get_poll()),
        exec_(loop_.get_executor()),
        ctrl_(lely::io::clock_monotonic),
        mtimer_(poll_, exec_, CLOCK_MONOTONIC),
        mchan_(ctx_, exec_),
        stimer_(poll_, exec_, CLOCK_MONOTONIC),
        schan_(ctx_, exec_),
        logger_("CiA402CANDriverShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~CiA402CANDriverShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        logger_->info("SetUp");

        mchan_.open(ctrl_);
        std::string path_master = __FILE__;
        path_master = path_master.substr(0, path_master.find("modules/"));
        path_master += "modules/Devices/CANopenDrivers/tests/config/CAN/master.dcf";
        master_ = std::make_shared<CANopenMaster>(mtimer_, mchan_, path_master, "", 1);

        std::string testFileDirName = __FILE__;
        testFileDirName = testFileDirName.substr(0, testFileDirName.find("modules/"));
        testFileDirName += "modules/Devices/CANopenDrivers/tests/config/CAN/ExampleDriver.json";
        std::ifstream f(testFileDirName);
        nlohmann::json data = nlohmann::json::parse(f);
        sut_ = std::make_unique<CiA402CANDriver>(master_, 3, data);


        schan_.open(ctrl_);
        std::string path_slave = __FILE__;
        path_slave = path_slave.substr(0, path_slave.find("modules/"));
        std::string bin_slave =
            path_slave + "modules/Devices/CANopenDrivers/tests/config/CAN/slave1.bin";
        path_slave += "modules/Devices/CANopenDrivers/tests/config/CAN/ExampleDriver.eds";
        slave_ = std::make_unique<MockSlave>(stimer_, schan_, path_slave, bin_slave, 3);

        slave_->Reset();
        master_->Reset();

        res_ = std::async(std::launch::async, [this] () {
            loop_.run();
            return true;
        });

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        slave_->Config();
    }

    void TearDown() override {
        logger_->info("TearDown");
        sut_->deinitialize();
        loop_.stop();
        res_.get();
    }

    lely::io::IoGuard io_guard_;
    lely::io::Context ctx_;
    lely::io::Poll poll_;
    lely::ev::Loop loop_;
    lely::ev::Executor exec_;
    lely::io::VirtualCanController ctrl_;
    lely::io::Timer mtimer_;
    lely::io::VirtualCanChannel mchan_;
    lely::io::Timer stimer_;
    lely::io::VirtualCanChannel schan_;
    std::future<bool> res_;

    std::shared_ptr<CANopenMaster> master_;

    std::unique_ptr<CiA402CANDriver> sut_;
    std::unique_ptr<MockSlave> slave_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(CiA402CANDriverShould, initalizeTheSlaveAndMoveItToOperationEnabled) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);
}

TEST_F(CiA402CANDriverShould, goIntoFaultAtReadyToSwitchOn) {
    slave_->goIntoFaultShutdown();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CANDriverShould, goIntoFaultAtSwitchOn) {
    slave_->goIntoFaultSwitchOn();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CANDriverShould, goIntoFaultAtEnableOperation) {
    slave_->goIntoFaultEnableOperation();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CANDriverShould, goIntoFaultDeinitialize) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    slave_->goIntoFaultDisableVoltage();
    ASSERT_FALSE(sut_->deinitialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CANDriverShould, goIntoQuickStop) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    ASSERT_TRUE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::QuickStopActive);
}

TEST_F(CiA402CANDriverShould, goIntoResetQuickStop) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);


    ASSERT_TRUE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::QuickStopActive);


    ASSERT_TRUE(sut_->resetQuickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);
}

TEST_F(CiA402CANDriverShould, goIntoFaultAtQuickStop) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    slave_->goIntoFaultQuickStop();
    ASSERT_FALSE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CANDriverShould, goIntoResetFaultAtReadyToSwitchOn) {
    slave_->goIntoFaultShutdown();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);

    ASSERT_TRUE(sut_->resetFault());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::SwitchONDisabled);
}

TEST_F(CiA402CANDriverShould, goIntoResetFaultAtSwitchOn) {
    slave_->goIntoFaultSwitchOn();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);


    ASSERT_TRUE(sut_->resetFault());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::SwitchONDisabled);
}

TEST_F(CiA402CANDriverShould, goIntoResetFaultAtEnableOperation) {
    slave_->goIntoFaultEnableOperation();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);

    ASSERT_TRUE(sut_->resetFault());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::SwitchONDisabled);
}

TEST_F(CiA402CANDriverShould, goIntoResetFaultAtQuickStop) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    slave_->goIntoFaultQuickStop();
    ASSERT_FALSE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);

    ASSERT_TRUE(sut_->resetFault());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::SwitchONDisabled);
}

TEST_F(CiA402CANDriverShould, goIntoInitializeAfterQuikcStop) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    ASSERT_TRUE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::QuickStopActive);

    ASSERT_TRUE(sut_->resetQuickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);
}

TEST_F(CiA402CANDriverShould, goIntoCyclicSyncTorqueMode) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    ASSERT_TRUE(sut_->setCyclicTorque(1000));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getModeOfOperation(), ModeOfOperation::CyclicSyncTorqueMode);
    // We lose precision because of the cast to int
    ASSERT_NEAR(sut_->getTorque().value(), 1000, 50);
}

TEST_F(CiA402CANDriverShould, goIntoCyclicSyncVelocityMode) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    ASSERT_TRUE(sut_->setCyclicVelocity(2));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getModeOfOperation(), ModeOfOperation::CyclicSyncVelocityMode);
    // We lose precision because of the cast to int
    ASSERT_NEAR(sut_->getVelocity().value(), 2, 0.01);
}

TEST_F(CiA402CANDriverShould, goIntoCyclicSyncPositionMode) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    ASSERT_TRUE(sut_->setCyclicPosition(3.14));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getModeOfOperation(), ModeOfOperation::CyclicSyncPositionMode);
    // We lose precision because of the cast to int
    ASSERT_NEAR(sut_->getPosition().value(), 3.14, 0.01);
}
