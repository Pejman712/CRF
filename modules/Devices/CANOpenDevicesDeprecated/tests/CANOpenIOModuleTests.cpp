/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <condition_variable>
#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "CANSocket/CANSocketMock.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/CANOpenContext.hpp"
#include "CANOpenDevices/CANOpenIOs/ICANOpenIOModule.hpp"
#include "CANOpenDevices/CANOpenIOs/CANOpenIOModule.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::communication::cansocket::ICANSocket;
using crf::communication::cansocket::CANSocketMock;
using crf::devices::canopendevices::CANOpenContext;
using crf::devices::canopendevices::CANOpenIOModule;
using crf::devices::canopendevices::ICANOpenIOModule;
using crf::devices::canopendevices::ObjectDictionary;

MATCHER_P(canFramesAreEqual, other, "Equality matcher for can_frame") {
    if (arg->can_id != other.can_id) return false;
    if (arg->can_dlc != other.can_dlc) return false;
    for (int i = 0; i < 8; i++) {
        if (arg->data[i] != other.data[i]) return false;
    }
    return true;
}

class ICANOpenIOModuleFactory {
 public:
    virtual std::shared_ptr<ICANOpenIOModule> create(std::shared_ptr<ICANSocket>) = 0;  // NOLINT
};

class CANOpenIOModuleFactory : public ICANOpenIOModuleFactory {
 public:
    std::shared_ptr<ICANOpenIOModule> create(std::shared_ptr<ICANSocket> socket) override { // NOLINT
        return std::make_shared<CANOpenIOModule>(0x01, socket);
    }
};

struct CANOpenIOFactoryStruct {
    std::shared_ptr<ICANOpenIOModuleFactory> factory;
};

class CANOpenIOModuleShould : public ::testing::TestWithParam<CANOpenIOFactoryStruct> {
 protected:
    CANOpenIOModuleShould() :
        logger_("CANOpenIOModuleShould"),
        testDirName_(),
        framesQueue_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("modules/"));
        testDirName_ += "modules/Devices/CANOpenDevicesDeprecated/tests/configurations/";
        socket_.reset(new NiceMock<CANSocketMock>);
        context_.reset(new CANOpenContext(socket_));

        auto factoryStruct = GetParam();
        device_ = factoryStruct.factory->create(socket_);
        dictionary_ = device_->getObjectDictionary();
    }

    virtual ~CANOpenIOModuleShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        setValue<uint32_t>(0x1000, 0x00, 0x00030191);
        setValue<uint8_t>(0x6000, 0x00, 1);
        setValue<uint8_t>(0x6000, 0x01, 0x02);
        setValue<uint8_t>(0x6200, 0x00, 1);
        setValue<uint8_t>(0x6202, 0x01, 0);

        ON_CALL(*socket_, initialize()).WillByDefault(Return(true));
        ON_CALL(*socket_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*socket_, getName()).WillByDefault(Return("can0"));
        ON_CALL(*socket_, write(_)).WillByDefault(Invoke([this](can_frame* frame) {  // NOLINT
            std::unique_lock<std::mutex> lck(frameMutex_);
            if ((frame->can_id == 0x601) && (frame->data[0] == 0x40)) {
                framesQueue_.push(buildSdoReadRepsonse(frame));
            } else if (frame->can_id == 0x601) {
                framesQueue_.push(buildSdoWriteResponse(frame));
            }

            frameCv_.notify_all();
            return sizeof(can_frame);
        }));

        ON_CALL(*socket_, read(_)).WillByDefault(Invoke([this](can_frame* frame) {  // NOLINT
            std::unique_lock<std::mutex> lck(frameMutex_);
            if (framesQueue_.empty()) {
                if (!frameCv_.wait_for(lck, std::chrono::milliseconds(1), [=]() { return !framesQueue_.empty(); })) { // NOLINT
                    return -1;
                }
            }

            can_frame poppedFrame = framesQueue_.front();
            framesQueue_.pop();

            std::memcpy(frame, &poppedFrame, sizeof(can_frame));
            return static_cast<int>(sizeof(can_frame));
        }));
    }

    can_frame buildSdoReadRepsonse(can_frame* frame) {
        can_frame response = {};
        response.can_id = 0x581;
        response.can_dlc = 0x08;

        auto reg = dictionary_->getRegister(getIndex(*frame), getSubIndex(*frame)).value();
        uint32_t registerNumber = (getIndex(*frame) << 16) + getSubIndex(*frame);

        response.data[1] = frame->data[1];
        response.data[2] = frame->data[2];
        response.data[3] = frame->data[3];

        if ((values_.find(registerNumber) == values_.end()) ||
            (failToReadThisRegister_ == getIndex(*frame))) {
                response.data[0] = 0x80;
                return response;
        }

        auto valueStr = values_[registerNumber];

        if (reg.getSize() == 1) {
            response.data[0] = 0x2F;
            response.data[4] = valueStr[0];
            response.data[5] = 0x00;
            response.data[6] = 0x00;
            response.data[7] = 0x00;
        } else if (reg.getSize() == 2) {
            response.data[0] = 0x2B;
            response.data[4] = valueStr[0];
            response.data[5] = valueStr[1];
            response.data[6] = 0x00;
            response.data[7] = 0x00;
        } else if (reg.getSize() == 4) {
            response.data[0] = 0x23;
            response.data[4] = valueStr[0];
            response.data[5] = valueStr[1];
            response.data[6] = valueStr[2];
            response.data[7] = valueStr[3];
        }

        return response;
    }

    can_frame buildSdoWriteResponse(can_frame* frame) {
        can_frame response = {};
        response.can_id = 0x581;
        response.can_dlc = 0x08;

        response.data[0] = 0x60;
        response.data[1] = frame->data[1];
        response.data[2] = frame->data[2];
        response.data[3] = frame->data[3];

        auto reg = dictionary_->getRegister(getIndex(*frame), getSubIndex(*frame)).value();
        uint32_t registerNumber = (getIndex(*frame) << 16) + getSubIndex(*frame);

        if (getIndex(*frame) == failToWriteThisRegister_) {
            response.data[0] = 0x80;
            return response;
        }

        std::string value(reinterpret_cast<char*>((frame->data)+4), reg.getSize());

        if (values_.find(registerNumber) == values_.end()) {
            values_.insert({registerNumber, value});
        } else {
            values_[registerNumber] = value;
        }

        return response;
    }

    uint16_t getIndex(const can_frame& cframe) {
        return (cframe.data[1] + (cframe.data[2] << 8));
    }

    uint8_t getSubIndex(const can_frame& cframe) {
        return cframe.data[3];
    }

    template<typename T>
    void setValue(uint16_t index, uint8_t subindex, T value) {
        uint32_t registerIndex =  (index << 16) + subindex;
        char buff[4];
        std::memcpy(buff, &value, sizeof(value));
        std::string valueStr(buff, sizeof(value));
        if (values_.find(registerIndex) == values_.end()) {
            values_.insert({registerIndex, valueStr});
        } else {
            values_[registerIndex] = valueStr;
        }
    }

    template<typename T>
    T getValue(uint16_t index, uint8_t subindex) {
        uint32_t registerIndex =  (index << 16) + subindex;
        T value;
        std::memcpy(&value, values_[registerIndex].c_str(), sizeof(value));
        return value;
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::mutex frameMutex_;
    std::condition_variable frameCv_;
    std::queue<can_frame> framesQueue_;
    std::shared_ptr<CANSocketMock> socket_;

    std::shared_ptr<ICANOpenIOModule> device_;
    std::unique_ptr<CANOpenContext> context_;
    std::shared_ptr<ObjectDictionary> dictionary_;

    uint32_t failToWriteThisRegister_ = 0x0000;
    uint32_t failToReadThisRegister_ = 0x0000;

    std::map<uint32_t, std::string> values_;
};

INSTANTIATE_TEST_CASE_P(instantiation_one, CANOpenIOModuleShould,
    ::testing::Values(
        CANOpenIOFactoryStruct{
            std::make_shared<CANOpenIOModuleFactory>()
        }
));

TEST_P(CANOpenIOModuleShould, failToInitializeIfNotAddedToContext) {
    ASSERT_FALSE(device_->initialize());
    ASSERT_FALSE(device_->deinitialize());
}

TEST_P(CANOpenIOModuleShould, initializationDeinitializationSequence) {
    context_->addDevice(device_);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(device_->initialize());
    ASSERT_FALSE(device_->initialize());
    ASSERT_TRUE(device_->deinitialize());
    ASSERT_FALSE(device_->deinitialize());

    ASSERT_TRUE(device_->initialize());
    ASSERT_FALSE(device_->initialize());
    ASSERT_TRUE(device_->deinitialize());
    ASSERT_FALSE(device_->deinitialize());

     ASSERT_TRUE(context_->deinitialize());
}

TEST_P(CANOpenIOModuleShould, allOperationsFailIfNotInitialized) {
    context_->addDevice(device_);
    ASSERT_TRUE(context_->initialize());
    ASSERT_EQ(device_->getCANID(), 0x01);

    ASSERT_FALSE(device_->isAlive());
    ASSERT_FALSE(device_->hasDigitalInputs());
    ASSERT_FALSE(device_->hasDigitalOutputs());
    ASSERT_FALSE(device_->hasAnalogueInputs());
    ASSERT_FALSE(device_->hasAnalogueOutputs());

    ASSERT_FALSE(device_->getDigitalInputState(0));
    ASSERT_FALSE(device_->getDigitalInputPolarity(0));
    ASSERT_FALSE(device_->setDigitalInputPolarity(0, true));
    ASSERT_FALSE(device_->setDigitalOutputState(0, true));
    ASSERT_FALSE(device_->getDigitalOutputState(0));
    ASSERT_FALSE(device_->setDigitalOutputPolarity(0, false));
    ASSERT_FALSE(device_->getDigitalOutputPolarity(0));
    ASSERT_FALSE(device_->getAnalogueInput(0));
    ASSERT_FALSE(device_->setAnalogueOutput(0, 12));
    ASSERT_FALSE(device_->getAnalogueOutput(0));

    ASSERT_EQ(device_->getDigitalInputCount(), 0);
    ASSERT_EQ(device_->getDigitalOutputCount(), 0);
    ASSERT_EQ(device_->getAnalogueInputCount(), 0);
    ASSERT_EQ(device_->getAnalogueOutputCount(), 0);

    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(CANOpenIOModuleShould, correctlyGetNumberOfIO) {
    context_->addDevice(device_);
    ASSERT_TRUE(context_->initialize());
    ASSERT_EQ(device_->getCANID(), 0x01);

    ASSERT_TRUE(device_->initialize());
    ASSERT_EQ(device_->getDigitalInputCount(), 8);
    ASSERT_EQ(device_->getDigitalOutputCount(), 8);

    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(CANOpenIOModuleShould, failsOperationOnOutOfRangeIO) {
    context_->addDevice(device_);
    ASSERT_TRUE(context_->initialize());
    ASSERT_EQ(device_->getCANID(), 0x01);

    ASSERT_TRUE(device_->initialize());
    ASSERT_FALSE(device_->getDigitalInputState(17));
    ASSERT_FALSE(device_->getDigitalInputPolarity(17));
    ASSERT_FALSE(device_->setDigitalInputPolarity(17, true));

    ASSERT_FALSE(device_->setDigitalOutputState(17, true));
    ASSERT_FALSE(device_->getDigitalOutputState(17));
    ASSERT_FALSE(device_->setDigitalOutputPolarity(17, true));
    ASSERT_FALSE(device_->getDigitalOutputPolarity(17));

    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(CANOpenIOModuleShould, correctlyGetDigitalInputsAndSetoutputs) {
    context_->addDevice(device_);
    ASSERT_TRUE(context_->initialize());
    ASSERT_EQ(device_->getCANID(), 0x01);

    ASSERT_TRUE(device_->initialize());
    ASSERT_TRUE(device_->getDigitalInputState(1));
    ASSERT_TRUE(device_->getDigitalInputState(1).value());
    /*ASSERT_TRUE(device_->getDigitalInputPolarity(1));
    ASSERT_FALSE(device_->getDigitalInputPolarity(1).value());
    ASSERT_TRUE(device_->setDigitalInputPolarity(1, true));*/

    ASSERT_TRUE(device_->getDigitalOutputState(1));
    ASSERT_FALSE(device_->getDigitalOutputState(1).value());
    ASSERT_TRUE(device_->getDigitalOutputPolarity(1));
    ASSERT_FALSE(device_->getDigitalOutputPolarity(1).value());
    ASSERT_TRUE(device_->setDigitalOutputPolarity(1, true));
    ASSERT_TRUE(device_->getDigitalOutputPolarity(1));
    ASSERT_TRUE(device_->getDigitalOutputPolarity(1).value());
    ASSERT_TRUE(device_->setDigitalOutputState(2, true));
    ASSERT_TRUE(device_->getDigitalOutputState(2));
    ASSERT_TRUE(device_->getDigitalOutputState(2).value());

    ASSERT_TRUE(context_->deinitialize());
}
