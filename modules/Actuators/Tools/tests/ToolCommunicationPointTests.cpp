/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <condition_variable>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "Mocks/Communication/IpcMock.hpp"
#include "Mocks/Devices/ToolMock.hpp"
#include "Tools/ToolCommunicationPoint.hpp"
#include "ComponentAccessControl/SimpleAccessControl.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::NiceMock;
using testing::Matcher;
using testing::AtLeast;


using crf::communication::componentaccesscontrol::SimpleAccessControl;
using crf::devices::tools::ToolMock;
using crf::devices::tools::ToolCommunicationPoint;

class ToolCommunicationPointShould : public ::testing::Test {
 protected:
    ToolCommunicationPointShould() :
        logger_("ToolCommunicationPointShould"),
        sut_(),
        accessControl_(new SimpleAccessControl),
        tool_(new NiceMock<ToolMock>),
        ipcInput1_(new NiceMock<IpcMock>),
        ipcOutput1_(new NiceMock<IpcMock>),
        ipcInput2_(new NiceMock<IpcMock>),
        ipcOutput2_(new NiceMock<IpcMock>) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        ON_CALL(*tool_, setValue(_, Matcher<bool>(_))).WillByDefault(
            Invoke([](const std::string& name, bool value) {
                if (name == "boolean") {
                    return true;
                }
                return false;
        }));

        ON_CALL(*tool_, setValue(_, Matcher<float>(_))).WillByDefault(
            Invoke([](const std::string& name, float value) {
                if (name == "velocity") {
                    return true;
                }
                return false;
        }));

        ON_CALL(*tool_, setValue(_, Matcher<int>(_))).WillByDefault(
            Invoke([](const std::string& name, int value) {
                if (name == "integer") {
                    return true;
                }
                return false;
        }));

        ON_CALL(*tool_, getValue(_)).WillByDefault(
            Invoke([](const std::string& name) -> boost::optional<boost::any> {
                if (name == "velocity") {
                    return boost::optional<boost::any>(10.2f);
                } else if (name == "integer") {
                    return boost::optional<boost::any>(static_cast<int32_t>(10));
                } else if (name == "boolean") {
                    return boost::optional<boost::any>(static_cast<bool>(false));
                }
                return boost::none;
        }));

        ON_CALL(*tool_, getValueType(_)).WillByDefault(
            Invoke([](const std::string& name) -> boost::optional<crf::devices::tools::ToolValueTypes> {  // NOLINT
                if (name == "velocity") {
                    return crf::devices::tools::ToolValueTypes::FLOAT;
                } else if (name == "boolean") {
                    return crf::devices::tools::ToolValueTypes::BOOL;
                } else if (name == "integer") {
                    return crf::devices::tools::ToolValueTypes::INT;
                }
                return boost::none;
        }));

        ON_CALL(*tool_, getValueNames()).WillByDefault(
            Invoke([]() {
                std::vector<std::string> retval({
                    "velocity"
                });
                return retval;
        }));

        startControlPacket_.data_["cmd"] = "start";
        releaseControlPacket_.data_["cmd"] = "stop";

        setVelocityPacket_.data_["cmd"] = "set";
        setVelocityPacket_.data_["name"] = "velocity";
        setVelocityPacket_.data_["value"] = 0.3f;

        setBooleanPacket_.data_["cmd"] = "set";
        setBooleanPacket_.data_["name"] = "boolean";
        setBooleanPacket_.data_["value"] = true;

        setIntegerPacket_.data_["cmd"] = "set";
        setIntegerPacket_.data_["name"] = "integer";
        setIntegerPacket_.data_["value"] = 10;

        getVelocityPacket_.data_["cmd"] = "get";
        getVelocityPacket_.data_["name"] = "velocity";

        getNotExistingVariablePacket_.data_["cmd"] = "get";
        getNotExistingVariablePacket_.data_["name"] = "position";

        unknownCmdPacket_.data_["cmd"] = "else";
        missingCmdPacket_.data_["cmd2"] = "ciao";

        missingNamePacket_.data_["cmd"] = "set";
        missingNamePacket_.data_["value"] = 0.3f;

        missingValuePacket_.data_["cmd"] = "set";
        missingValuePacket_.data_["name"] = "velocity";

        unknownNamePacket_.data_["cmd"] = "set";
        unknownNamePacket_.data_["name"] = "velocityd";
        unknownNamePacket_.data_["value"] = 0.3f;
    }

    ~ToolCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<ToolCommunicationPoint> sut_;
    std::unique_ptr<ToolCommunicationPoint> sut2_;

    std::shared_ptr<SimpleAccessControl> accessControl_;
    std::shared_ptr<ToolMock> tool_;
    std::shared_ptr<IpcMock> ipcInput1_;
    std::shared_ptr<IpcMock> ipcOutput1_;
    std::shared_ptr<IpcMock> ipcInput2_;
    std::shared_ptr<IpcMock> ipcOutput2_;

    communication::datapackets::JSONPacket startControlPacket_;
    communication::datapackets::JSONPacket releaseControlPacket_;
    communication::datapackets::JSONPacket setVelocityPacket_;
    communication::datapackets::JSONPacket setBooleanPacket_;
    communication::datapackets::JSONPacket setIntegerPacket_;
    communication::datapackets::JSONPacket getVelocityPacket_;
    communication::datapackets::JSONPacket getNotExistingVariablePacket_;
    communication::datapackets::JSONPacket unknownCmdPacket_;
    communication::datapackets::JSONPacket missingCmdPacket_;
    communication::datapackets::JSONPacket missingNamePacket_;
    communication::datapackets::JSONPacket missingValuePacket_;
    communication::datapackets::JSONPacket unknownNamePacket_;
};

TEST_F(ToolCommunicationPointShould, initializeDeinitializeSequence) {
    ON_CALL(*ipcInput1_, read(_, _)).WillByDefault(Return(false));
    ON_CALL(*ipcOutput1_, write(_, _)).WillByDefault(Return(false));

    sut_.reset(new ToolCommunicationPoint(tool_, ipcInput1_, ipcOutput1_, 1, accessControl_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(ToolCommunicationPointShould, correctlyReturnBoostNoneOnNonExistingPacket) {
    ON_CALL(*ipcOutput1_, write(_, _)).WillByDefault(Return(false));

    std::mutex mtx;
    std::condition_variable cv;
    bool called = false;

    EXPECT_CALL(*ipcInput1_, read(_, _)).WillOnce(
        Invoke([this](std::string& bytes, Packets::PacketHeader& header) {
            bytes = getNotExistingVariablePacket_.serialize();
            header = getNotExistingVariablePacket_.getHeader();
            return true;
    })).WillRepeatedly(Return(false));

    EXPECT_CALL(*tool_, getValueType(_)).WillRepeatedly(
        Invoke([&mtx, &cv, &called](const std::string& name) -> boost::optional<crf::devices::tools::ToolValueTypes> {  // NOLINT
            std::unique_lock<std::mutex> lock(mtx);

            if (name == "velocity") {
                return crf::devices::tools::ToolValueTypes::FLOAT;
            }

            cv.notify_all();
            called = true;
            return boost::none;
    }));

    sut_.reset(new ToolCommunicationPoint(tool_, ipcInput1_, ipcOutput1_, 1, accessControl_));

    {
        std::unique_lock<std::mutex> lock(mtx);
        ASSERT_TRUE(sut_->initialize());

        ASSERT_TRUE(cv.wait_for(lock, std::chrono::milliseconds(250), [&called]() {
            return called;
        }));
    }

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ToolCommunicationPointShould, correctlySetValueAndHandlePriority) {
    ON_CALL(*ipcOutput1_, write(_, _)).WillByDefault(Return(false));
    ON_CALL(*ipcOutput2_, write(_, _)).WillByDefault(Return(false));

    std::mutex mtx;
    std::condition_variable cv;
    bool called = false;
    bool ipc1read = false;
    bool ipc2read = false;

    bool startStopController1 = true;
    int controller2status = 1;

    EXPECT_CALL(*ipcInput1_, read(_, _)).WillRepeatedly(
        Invoke([this, &startStopController1, &mtx, &cv, &ipc1read](std::string& bytes, Packets::PacketHeader& header) {  // NOLINT
            std::unique_lock<std::mutex> lock(mtx);
            if (startStopController1) {
                bytes = startControlPacket_.serialize();
                header = startControlPacket_.getHeader();
            } else {
                bytes = releaseControlPacket_.serialize();
                header = releaseControlPacket_.getHeader();
            }
            cv.notify_all();
            ipc1read = true;
            return true;
    }));

    EXPECT_CALL(*ipcInput2_, read(_, _)).WillRepeatedly(
        Invoke([this, &controller2status, &mtx, &cv, &ipc2read](std::string& bytes, Packets::PacketHeader& header) {  // NOLINT
            std::unique_lock<std::mutex> lock(mtx);
            if (controller2status == 1) {
                bytes = startControlPacket_.serialize();
                header = startControlPacket_.getHeader();
            } else if (controller2status == 2) {
                bytes = setVelocityPacket_.serialize();
                header = setVelocityPacket_.getHeader();
            } else {
                bytes = releaseControlPacket_.serialize();
                header = releaseControlPacket_.getHeader();
            }
            cv.notify_all();
            ipc2read = true;
            return true;
    }));

    EXPECT_CALL(*tool_, setValue(_, Matcher<float>(_))).WillRepeatedly(
            Invoke([&mtx, &cv, &called](const std::string& name, float value) {
                std::unique_lock<std::mutex> lock(mtx);
                if (name == "velocity") {
                    cv.notify_all();
                    called = true;
                    return true;
                }
                return false;
        }));

    sut_.reset(new ToolCommunicationPoint(tool_, ipcInput1_, ipcOutput1_, 2, accessControl_));
    sut2_.reset(new ToolCommunicationPoint(tool_, ipcInput2_, ipcOutput2_, 1, accessControl_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut2_->initialize());
    {
        std::unique_lock<std::mutex> lock(mtx);

        controller2status = 2;
        ASSERT_FALSE(cv.wait_for(lock, std::chrono::milliseconds(250), [&called]() {
            return called;
        }));

        startStopController1 = false;
        ipc1read = false;
        ASSERT_TRUE(cv.wait_for(lock, std::chrono::milliseconds(250), [&ipc1read]() {
            return ipc1read;
        }));

        controller2status = 1;
        ipc2read = false;
        ASSERT_TRUE(cv.wait_for(lock, std::chrono::milliseconds(250), [&ipc2read]() {
            return ipc2read;
        }));

        controller2status = 2;
        called = false;
        ASSERT_TRUE(cv.wait_for(lock, std::chrono::milliseconds(250), [&called]() {
            return called;
        }));
    }

    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut2_->deinitialize());
}

TEST_F(ToolCommunicationPointShould, correctlyHandleWrongPackets) {
    ON_CALL(*ipcOutput1_, write(_, _)).WillByDefault(Return(false));

    std::mutex mtx;
    std::condition_variable cv;
    bool called = false;
    int state = -1;

    EXPECT_CALL(*ipcInput1_, read(_, _)).WillRepeatedly(
        Invoke([this, &mtx, &cv, &called, &state](std::string& bytes, Packets::PacketHeader& header) { // NOLINT
            std::unique_lock<std::mutex> lock(mtx);
            if (state == 0) {
                bytes = startControlPacket_.serialize();
                header = startControlPacket_.getHeader();
                header.type = 13;
            } else if (state == 1) {
                bytes = "startControlPacket_.serialize()";
                header = startControlPacket_.getHeader();
                header.length = bytes.size();
            } else if (state == 2) {
                bytes = missingCmdPacket_.serialize();
                header = missingCmdPacket_.getHeader();
            } else if (state == 3) {
                bytes = unknownCmdPacket_.serialize();
                header = unknownCmdPacket_.getHeader();
            } else if (state == 4) {
                bytes = startControlPacket_.serialize();
                header = startControlPacket_.getHeader();
            } else if (state == 5) {
                bytes = setVelocityPacket_.serialize();
                header = setVelocityPacket_.getHeader();
            } else if (state == 6) {
                bytes = missingNamePacket_.serialize();
                header = missingNamePacket_.getHeader();
            } else if (state == 7) {
                bytes = missingValuePacket_.serialize();
                header = missingValuePacket_.getHeader();
            } else if (state == 8) {
                bytes = unknownNamePacket_.serialize();
                header = unknownNamePacket_.getHeader();
            } else if (state == 9) {
                bytes = setBooleanPacket_.serialize();
                header = setBooleanPacket_.getHeader();
            } else if (state == 10) {
                bytes = setIntegerPacket_.serialize();
                header = setIntegerPacket_.getHeader();
            } else {
                return false;
            }

            called = true;
            cv.notify_all();
            state = -1;
            return true;
    }));

    EXPECT_CALL(*tool_, setValue(_, Matcher<bool>(_))).Times(1);
    EXPECT_CALL(*tool_, setValue(_, Matcher<float>(_))).Times(1);
    EXPECT_CALL(*tool_, setValue(_, Matcher<int>(_))).Times(1);

    sut_.reset(new ToolCommunicationPoint(tool_, ipcInput1_, ipcOutput1_, 1, accessControl_));

    {
        std::unique_lock<std::mutex> lock(mtx);
        ASSERT_TRUE(sut_->initialize());

        for (int i = 0; i < 11; i++) {
            called = false;
            state = i;
            ASSERT_TRUE(cv.wait_for(lock, std::chrono::milliseconds(250), [&called]() {
                return called;
            }));
        }
    }

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ToolCommunicationPointShould, correctlyGetValues) {
    ON_CALL(*ipcOutput1_, write(_, _)).WillByDefault(Return(false));
    ON_CALL(*ipcOutput1_, write(_, _)).WillByDefault(Return(false));
    ON_CALL(*tool_, getValueNames()).WillByDefault(Return(std::vector<std::string>()));

    std::mutex mtx;
    std::condition_variable cv;
    bool called = false;
    int state = -1;

    EXPECT_CALL(*ipcInput1_, read(_, _)).WillRepeatedly(
        Invoke([this, &mtx, &cv, &called, &state](std::string& bytes, Packets::PacketHeader& header) { // NOLINT
            std::unique_lock<std::mutex> lock(mtx);
            if (state == 0) {
                bytes = getVelocityPacket_.serialize();
                header = getVelocityPacket_.getHeader();
            } else if (state == 1) {
                getVelocityPacket_.data_["name"] = "integer";
                bytes = getVelocityPacket_.serialize();
                header = getVelocityPacket_.getHeader();
            } else if (state == 2) {
                getVelocityPacket_.data_["name"] = "boolean";
                bytes = getVelocityPacket_.serialize();
                header = getVelocityPacket_.getHeader();
            } else {
                return false;
            }

            called = true;
            cv.notify_all();
            state = -1;
            return true;
    }));

    EXPECT_CALL(*tool_, getValue(_)).Times(3);
    sut_.reset(new ToolCommunicationPoint(tool_, ipcInput1_, ipcOutput1_, 1, accessControl_));

    {
        std::unique_lock<std::mutex> lock(mtx);
        ASSERT_TRUE(sut_->initialize());

        for (int i = 0; i < 3; i++) {
            called = false;
            state = i;
            ASSERT_TRUE(cv.wait_for(lock, std::chrono::milliseconds(250), [&called]() {
                return called;
            }));
        }
    }

    ASSERT_TRUE(sut_->deinitialize());
}
