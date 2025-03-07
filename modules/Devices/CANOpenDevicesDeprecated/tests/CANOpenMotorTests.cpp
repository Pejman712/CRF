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
#include "CANOpenDevices/CANOpenMotors/ICANOpenMotor.hpp"
#include "CANOpenDevices/CANOpenMotors/MaxonEPOS4.hpp"
#include "CANOpenDevices/CANOpenMotors/MaxonEPOS2.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::communication::cansocket::ICANSocket;
using crf::communication::cansocket::CANSocketMock;
using crf::devices::canopendevices::CANOpenContext;
using crf::devices::canopendevices::ICANOpenMotor;
using crf::devices::canopendevices::MaxonEPOS2;
using crf::devices::canopendevices::MaxonEPOS4;
using crf::devices::canopendevices::ObjectDictionary;

MATCHER_P(canFramesAreEqual, other, "Equality matcher for can_frame") {
    if (arg->can_id != other.can_id) return false;
    if (arg->can_dlc != other.can_dlc) return false;
    for (int i = 0; i < 8; i++) {
        if (arg->data[i] != other.data[i]) return false;
    }
    return true;
}

class CANOpenMotorFactory {
 public:
    virtual std::shared_ptr<ICANOpenMotor> create(std::shared_ptr<ICANSocket>) = 0;  // NOLINT
};

class MaxonEPOS4Factory : public CANOpenMotorFactory {
 public:
    std::shared_ptr<ICANOpenMotor> create(std::shared_ptr<ICANSocket> socket) override { // NOLINT
        return std::make_shared<MaxonEPOS4>(0x01, socket);
    }
};

class MaxonEPOS2Factory : public CANOpenMotorFactory {
 public:
    std::shared_ptr<ICANOpenMotor> create(std::shared_ptr<ICANSocket> socket) override { // NOLINT
        return std::make_shared<MaxonEPOS2>(0x01, socket);
    }
};

struct CANOpenMotorFactoryStruct {
    std::shared_ptr<CANOpenMotorFactory> factory;
    uint16_t statusWordSwitchOnDisabled;
    uint16_t statusWordReadyToSwitchOn;
    uint16_t statusWordSwitchedOn;
    uint16_t statusWordEnabled;
    uint16_t statusWordFault;
    uint16_t statusWordQuickStop;
};

class ICANOpenMotorShould : public ::testing::TestWithParam<CANOpenMotorFactoryStruct> {
 protected:
    ICANOpenMotorShould() :
        logger_("ICANOpenMotorShould"),
        framesQueue_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        socket_.reset(new NiceMock<CANSocketMock>);
        context_.reset(new CANOpenContext(socket_));
        auto factoryStruct = GetParam();
        motor_ = factoryStruct.factory->create(socket_);
        dictionary_ = motor_->getObjectDictionary();
    }

    virtual ~ICANOpenMotorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        statusWordSwitchOnDisabled = GetParam().statusWordSwitchOnDisabled;
        statusWordReadyToSwitchOn = GetParam().statusWordReadyToSwitchOn;
        statusWordSwitchedOn = GetParam().statusWordSwitchedOn;
        statusWordEnabled = GetParam().statusWordEnabled;
        statusWordFault = GetParam().statusWordFault;
        statusWordQuickStop = GetParam().statusWordQuickStop;

        setValue<uint8_t>(0x2000, 0x00, 0x01);
        setValue<uint16_t>(0x6041, 0x00, statusWordReadyToSwitchOn);
        setValue<uint16_t>(0x6040, 0x00, 0x0000);
        setValue<uint8_t>(0x6061, 0x00, modeOfOperationPPM);
        // Set some motor characteristics
        setValue<uint32_t>(0x60C5, 0x00, 200);      // maxAcceleration
        setValue<uint32_t>(0x607F, 0x00, 250);      // maxProfileVelocity
        setValue<uint32_t>(0x6085, 0x00, 300);      // quickStopDeceleration
        setValue<uint32_t>(0x6084, 0x00, 350);      // profileDeceleration
        setValue<uint32_t>(0x6083, 0x00, 400);      // profileAcceleration
        setValue<uint32_t>(0x6081, 0x00, 450);      // profileVelocity
        setValue<uint32_t>(0x607D, 0x01, 500);      // positionlimit minimum
        setValue<uint32_t>(0x607D, 0x02, 600);      // positionlimit maximum

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

        auto reg = dictionary_->getRegister(getIndex(*frame), getSubIndex(*frame)).get();
        uint32_t registerNumber = (getIndex(*frame) << 16) + getSubIndex(*frame);

        if ((values_.find(registerNumber) == values_.end()) ||
            (failToReadThisRegister_ == getIndex(*frame))) {
                response.data[0] = 0x80;
                return response;
        }

        auto valueStr = values_[registerNumber];

        response.data[1] = frame->data[1];
        response.data[2] = frame->data[2];
        response.data[3] = frame->data[3];

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

        auto reg = dictionary_->getRegister(getIndex(*frame), getSubIndex(*frame)).get();
        uint32_t registerNumber = (getIndex(*frame) << 16) + getSubIndex(*frame);

        if (getIndex(*frame) == failToWriteThisRegister_) {
            response.data[0] = 0x80;
            return response;
        }

        if (getIndex(*frame) == 0x6040) {
            uint16_t controlWord = 0;
            std::memcpy(&controlWord, (reinterpret_cast<char*>(frame->data)+4), 2);
            setValue(0x6041, 0x00, statusWordTransitions(controlWord));

            if (registerNumber == failToWriteThisRegister_) {
                response.data[0] = 0x80;
                return response;
            }
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

    uint16_t statusWordTransitions(uint16_t controlword) {
        uint32_t registerIndex =  (0x6041 << 16) + 0x00;
        uint16_t statusWord;
        std::memcpy(&statusWord, values_[registerIndex].c_str(), 2);

        if (((statusWord & statusWordSwitchOnDisabled) == statusWordSwitchOnDisabled) && ((controlword & 0x0006) == 0x0006)) {  // NOLINT
            statusWord = statusWordReadyToSwitchOn;
        } else if (((statusWord & statusWordReadyToSwitchOn) == statusWordReadyToSwitchOn) && ((controlword & 0x000F) == 0x000F)) {  // NOLINT
            statusWord = statusWordEnabled;
        } else if (((statusWord & statusWordReadyToSwitchOn) == statusWordReadyToSwitchOn) && ((controlword & 0x0007) == 0x0007)) {  // NOLINT
            statusWord = statusWordSwitchedOn;
        } else if (((statusWord & statusWordEnabled) == statusWordEnabled) && ((controlword & 0x0002) == 0x0002)) {  // NOLINT
            statusWord = statusWordQuickStop;
        } else if (((statusWord & statusWordEnabled) == statusWordEnabled) && ((controlword & 0x0007) == 0x0007)) {  // NOLINT
            statusWord = statusWordSwitchedOn;
        } else if (((statusWord & statusWordQuickStop) == statusWordQuickStop) && ((controlword & 0x000F) == 0x000F)) {  // NOLINT
            statusWord = statusWordEnabled;
        } else if (((statusWord & statusWordFault) == statusWordFault) && ((controlword & 0x0080) == 0x0080)) {  // NOLINT
            statusWord = statusWordSwitchOnDisabled;
        }

        return statusWord;
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
    std::mutex frameMutex_;
    std::condition_variable frameCv_;
    std::queue<can_frame> framesQueue_;
    std::shared_ptr<CANSocketMock> socket_;

    std::shared_ptr<ICANOpenMotor> motor_;
    std::unique_ptr<CANOpenContext> context_;
    std::shared_ptr<ObjectDictionary> dictionary_;

    uint16_t statusWordSwitchOnDisabled;
    uint16_t statusWordReadyToSwitchOn;
    uint16_t statusWordSwitchedOn;
    uint16_t statusWordEnabled;
    uint16_t statusWordFault;
    uint16_t statusWordQuickStop;
    uint16_t statusWordTargetReached = 0x0400;

    const uint16_t controlWordSwitchedOn = 0x0001;
    const uint16_t controlWordEnableVoltage = 0x0002;
    const uint16_t controlWordQuickStop = 0x0004;
    const uint16_t controlWordEnableOperation = 0x0008;
    const uint16_t controlWordNewSetPoint = 0x0010;
    const uint16_t controlWordChangeSetImmediately = 0x0020;
    const uint16_t controlWordAbsRel = 0x0040;

    const uint8_t modeOfOperationPPM = 1;
    const uint8_t modeOfOperationPVM = 3;
    const uint8_t modeOfOperationHMM = 6;
    const uint8_t modeOfOperationCSP = 8;
    const uint8_t modeOfOperationCSV = 9;
    const uint8_t modeOfOperationCST = 10;

    uint32_t failToWriteThisRegister_ = 0x0000;
    uint32_t failToReadThisRegister_ = 0x0000;

    const uint16_t controlWordRegister_ = 0x6040;
    const uint16_t statusWordRegister_ = 0x6041;

    std::map<uint32_t, std::string> values_;
};

INSTANTIATE_TEST_CASE_P(instantiation_one, ICANOpenMotorShould,
    ::testing::Values(
        CANOpenMotorFactoryStruct{
            std::make_shared<MaxonEPOS2Factory>(),
            0x0140,
            0x0121,
            0x0123,
            0x0137,
            0x0108,
            0x0117
        },
        CANOpenMotorFactoryStruct{
            std::make_shared<MaxonEPOS4Factory>(),
            0x0040,
            0x0021,
            0x0023,
            0x0027,
            0x0008,
            0x0007
        }
));

TEST_P(ICANOpenMotorShould, failToInitializeIfNotAddedToContext) {
    ASSERT_FALSE(motor_->initialize());
    ASSERT_FALSE(motor_->deinitialize());
}

TEST_P(ICANOpenMotorShould, initializationDeinitializationSequence) {
    context_->addDevice(motor_);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor_->initialize());
    ASSERT_FALSE(motor_->initialize());
    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_FALSE(motor_->deinitialize());

    ASSERT_TRUE(motor_->initialize());
    ASSERT_FALSE(motor_->initialize());
    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_FALSE(motor_->deinitialize());

     ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, allOperationsFailIfNotInitialized) {
    context_->addDevice(motor_);
    ASSERT_TRUE(context_->initialize());
    ASSERT_EQ(motor_->getCANID(), 0x01);

    ASSERT_FALSE(motor_->isAlive());
    ASSERT_FALSE(motor_->inFault());
    ASSERT_FALSE(motor_->inQuickStop());
    ASSERT_FALSE(motor_->isEnabled());
    ASSERT_FALSE(motor_->isReadyToSwitchOn());
    ASSERT_FALSE(motor_->getNMTState());
    ASSERT_FALSE(motor_->getVelocity());
    ASSERT_FALSE(motor_->getPosition());
    ASSERT_FALSE(motor_->getCurrent());
    ASSERT_FALSE(motor_->getStatusWord());
    ASSERT_FALSE(motor_->getModeOfOperation());
    ASSERT_FALSE(motor_->getDigitalInput());

    ASSERT_FALSE(motor_->setDigitalOutput(0));
    ASSERT_FALSE(motor_->resetDigitalOutput(0));
    ASSERT_FALSE(motor_->enableOperation());
    ASSERT_FALSE(motor_->disableOperation());
    ASSERT_FALSE(motor_->stop());
    ASSERT_FALSE(motor_->quickStop());
    ASSERT_FALSE(motor_->shutdown());
    ASSERT_FALSE(motor_->faultReset());
    ASSERT_FALSE(motor_->setPosition(0, false));
    ASSERT_FALSE(motor_->setPosition(0, 1, false));
    ASSERT_FALSE(motor_->setPosition(0, 1, 1, false));
    ASSERT_FALSE(motor_->setPosition(0, 1, 1, 1, false));
    ASSERT_FALSE(motor_->positionReached());
    ASSERT_FALSE(motor_->setVelocity(0));
    ASSERT_FALSE(motor_->setVelocity(0, 1));
    ASSERT_FALSE(motor_->setVelocity(0, 1, 1));
    ASSERT_FALSE(motor_->setTorque(0));
    ASSERT_FALSE(motor_->setCurrent(0));
    ASSERT_FALSE(motor_->setProfileVelocity(0));
    ASSERT_FALSE(motor_->setProfileAcceleration(0));
    ASSERT_FALSE(motor_->setProfileDeceleration(0));
    ASSERT_FALSE(motor_->setQuickstopDeceleration(0));
    ASSERT_FALSE(motor_->setPositionLimits({0, 0}));

    ASSERT_EQ(motor_->getProfileVelocity(), 0);
    ASSERT_EQ(motor_->getProfileAcceleration(), 0);
    ASSERT_EQ(motor_->getProfileDeceleration(), 0);
    ASSERT_EQ(motor_->getQuickstopDeceleration(), 0);
    ASSERT_EQ(motor_->getMaximumVelocity(), 0);
    ASSERT_EQ(motor_->getMaximumAcceleration(), 0);
    ASSERT_EQ(motor_->getPositionLimits().first, 0);
    ASSERT_EQ(motor_->getPositionLimits().second, 0);

     ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, failsToInitializeIfMotorAnswersWithDifferentCANID) {
    setValue<uint8_t>(0x2000, 0x00, 0x02);
    context_->addDevice(motor_);
    ASSERT_TRUE(context_->initialize());

    ASSERT_FALSE(motor_->initialize());

    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, failsToInitializeIfFailsToReadStatusWord) {
    failToReadThisRegister_ = 0x6041;
    context_->addDevice(motor_);
    ASSERT_TRUE(context_->initialize());

    ASSERT_FALSE(motor_->initialize());

    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, failsToInitializeIfFailsToSetAPdo) {
    failToWriteThisRegister_ = 0x1600;
    context_->addDevice(motor_);
    ASSERT_TRUE(context_->initialize());

    ASSERT_FALSE(motor_->initialize());

    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, notPossibleToEnableIfIsInFault) {
    context_->addDevice(motor_);

    setValue(statusWordRegister_, 0x00, statusWordFault);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor_->initialize());

    ASSERT_FALSE(motor_->isEnabled());
    ASSERT_TRUE(motor_->inFault());
    ASSERT_FALSE(motor_->enableOperation());
    ASSERT_FALSE(motor_->isEnabled());

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, correctlyReadAndWriteMotorSettingValues) {
    context_->addDevice(motor_);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor_->initialize());

    ASSERT_EQ(motor_->getPositionLimits().first, 500);
    ASSERT_EQ(motor_->getPositionLimits().second, 600);
    ASSERT_EQ(motor_->getMaximumAcceleration(), 200);
    ASSERT_EQ(motor_->getMaximumVelocity(), 250);
    ASSERT_EQ(motor_->getQuickstopDeceleration(), 300);
    ASSERT_EQ(motor_->getProfileDeceleration(), 350);
    ASSERT_EQ(motor_->getProfileAcceleration(), 400);
    ASSERT_EQ(motor_->getProfileVelocity(), 450);

    ASSERT_TRUE(motor_->setProfileVelocity(250));
    ASSERT_TRUE(motor_->setProfileAcceleration(350));
    ASSERT_TRUE(motor_->setProfileDeceleration(450));
    ASSERT_TRUE(motor_->setQuickstopDeceleration(550));
    ASSERT_TRUE(motor_->setPositionLimits({0, 200}));

    ASSERT_EQ(motor_->getPositionLimits().first, 0);
    ASSERT_EQ(motor_->getPositionLimits().second, 200);
    ASSERT_EQ(motor_->getQuickstopDeceleration(), 550);
    ASSERT_EQ(motor_->getProfileDeceleration(), 450);
    ASSERT_EQ(motor_->getProfileAcceleration(), 350);
    ASSERT_EQ(motor_->getProfileVelocity(), 250);

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, correctlyRemoveFaultAndEnableMotor) {
    context_->addDevice(motor_);

    setValue(statusWordRegister_, 0x00, statusWordFault);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor_->initialize());

    ASSERT_FALSE(motor_->isEnabled());
    ASSERT_TRUE(motor_->inFault());
    ASSERT_TRUE(motor_->faultReset());
    ASSERT_TRUE(motor_->enableOperation());
    ASSERT_TRUE(motor_->isEnabled());

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, correctlyEnableIfReadyToSwitchOn) {
    context_->addDevice(motor_);

    setValue(statusWordRegister_, 0x00, statusWordReadyToSwitchOn);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor_->initialize());

    ASSERT_FALSE(motor_->isEnabled());
    ASSERT_TRUE(motor_->isReadyToSwitchOn());
    ASSERT_TRUE(motor_->enableOperation());
    ASSERT_TRUE(motor_->isEnabled());

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, correctlyReturnsIsAliveIfReceivedAGuard) {
    context_->addDevice(motor_);

    setValue(statusWordRegister_, 0x00, statusWordReadyToSwitchOn);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor_->initialize());
    {
        std::unique_lock<std::mutex> lck(frameMutex_);
        can_frame response = {};
        response.can_id = 0x701;
        response.can_dlc = 0x01;
        response.data[0] = 0x05;
        framesQueue_.push(response);
    }

    while (!framesQueue_.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ASSERT_TRUE(motor_->isAlive());
    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, correctlySetupMotor) {
    context_->addDevice(motor_);

    setValue(statusWordRegister_, 0x00, statusWordReadyToSwitchOn);
    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(motor_->initialize());

    auto reg = dictionary_->getRegister("consumerHeartbeatTime/consumer1").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), ((0x0025 << 16) | 255));
    reg = dictionary_->getRegister("producerHeartbeatTime").get();
    ASSERT_EQ(getValue<uint16_t>(reg.getIndex(), reg.getSubindex()), 250);

    // RPDO
    reg = dictionary_->getRegister("receivePDO1Parameter/cobID").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), 0x40000201);
    reg = dictionary_->getRegister("receivePDO1Parameter/transmissionType").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 255);
    reg = dictionary_->getRegister("receivePDO2Parameter/cobID").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), 0x40000301);
    reg = dictionary_->getRegister("receivePDO2Parameter/transmissionType").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 255);
    reg = dictionary_->getRegister("receivePDO3Parameter/cobID").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), 0x40000401);
    reg = dictionary_->getRegister("receivePDO3Parameter/transmissionType").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 255);
    reg = dictionary_->getRegister("receivePDO4Parameter/cobID").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), 0x40000501);
    reg = dictionary_->getRegister("receivePDO4Parameter/transmissionType").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 255);

    reg = dictionary_->getRegister("receivePDO1Mapping/numberOfObjects").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 2);
    reg = dictionary_->getRegister("receivePDO2Mapping/numberOfObjects").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 2);
    reg = dictionary_->getRegister("receivePDO3Mapping/numberOfObjects").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 2);
    reg = dictionary_->getRegister("receivePDO4Mapping/numberOfObjects").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 2);

    // TPDO
    reg = dictionary_->getRegister("transmitPDO1Parameter/cobID").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), 0x40000181);
    reg = dictionary_->getRegister("transmitPDO1Parameter/transmissionType").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 1);
    reg = dictionary_->getRegister("transmitPDO2Parameter/cobID").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), 0x40000281);
    reg = dictionary_->getRegister("transmitPDO2Parameter/transmissionType").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 1);
    reg = dictionary_->getRegister("transmitPDO3Parameter/cobID").get();
    ASSERT_EQ(getValue<uint32_t>(reg.getIndex(), reg.getSubindex()), 0x40000381);
    reg = dictionary_->getRegister("transmitPDO3Parameter/transmissionType").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 1);

    reg = dictionary_->getRegister("transmitPDO1Mapping/numberOfObjects").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 2);
    reg = dictionary_->getRegister("transmitPDO2Mapping/numberOfObjects").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 2);
    reg = dictionary_->getRegister("transmitPDO3Mapping/numberOfObjects").get();
    ASSERT_EQ(getValue<uint8_t>(reg.getIndex(), reg.getSubindex()), 2);

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, enableDisableOperationSequence) {
    context_->addDevice(motor_);

    setValue(statusWordRegister_, 0x00, statusWordReadyToSwitchOn);
    ASSERT_TRUE(context_->initialize());

    ASSERT_TRUE(motor_->initialize());

    ASSERT_FALSE(motor_->isEnabled());
    ASSERT_TRUE(motor_->enableOperation());
    ASSERT_TRUE(motor_->isEnabled());
    ASSERT_TRUE(motor_->disableOperation());
    ASSERT_FALSE(motor_->isEnabled());
    ASSERT_TRUE(motor_->enableOperation());
    ASSERT_TRUE(motor_->isEnabled());

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, ifWriteSDOSuccedsObjectDictionaryHasUpdatedValue) {
    context_->addDevice(motor_);

    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(motor_->initialize());

    ASSERT_TRUE(motor_->setProfileVelocity(5000));
    ASSERT_EQ(motor_->getObjectDictionary()->getRegister("profileVelocity")
        .get().getValue<uint32_t>(), 5000);

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}

TEST_P(ICANOpenMotorShould, ifWriteSDOFailsObjectDictionaryHasNotUpdatedValue) {
    context_->addDevice(motor_);
    failToWriteThisRegister_ = 0x6081;
    setValue(0x6081, 0x00, 2000);
    ASSERT_TRUE(context_->initialize());
    ASSERT_TRUE(motor_->initialize());

    ASSERT_EQ(motor_->getProfileVelocity(), 2000);
    ASSERT_FALSE(motor_->setProfileVelocity(5000));
    ASSERT_EQ(motor_->getObjectDictionary()->getRegister("profileVelocity")
        .get().getValue<uint32_t>(), 2000);

    ASSERT_TRUE(motor_->deinitialize());
    ASSERT_TRUE(context_->deinitialize());
}
