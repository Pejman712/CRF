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
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/ObjectDictionaryRegister.hpp"

using testing::_;
using testing::Return;

using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::ObjectDictionaryRegister;

class ObjectDictionaryShould : public ::testing::Test {
 protected:
    ObjectDictionaryShould() :
        logger_("ObjectDictionaryShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("modules/"));
        testDirName_ += "modules/Devices/CANOpenDevicesDeprecated/tests/configurations/";
    }

    ~ObjectDictionaryShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        SDOPositionShould_.can_id = 0x581;
        SDOPositionShould_.can_dlc = 0x08;
        SDOPositionShould_.data[0] = 0x43;
        SDOPositionShould_.data[1] = 0x64;
        SDOPositionShould_.data[2] = 0x60;
        SDOPositionShould_.data[3] = 0x00;
        SDOPositionShould_.data[4] = 0x51;
        SDOPositionShould_.data[5] = 0x00;
        SDOPositionShould_.data[6] = 0x00;
        SDOPositionShould_.data[7] = 0x00;

        SDOStatusWordShould_.can_id = 0x581;
        SDOStatusWordShould_.can_dlc = 0x08;
        SDOStatusWordShould_.data[0] = 0x4B;
        SDOStatusWordShould_.data[1] = 0x41;
        SDOStatusWordShould_.data[2] = 0x60;
        SDOStatusWordShould_.data[3] = 0x00;
        SDOStatusWordShould_.data[4] = 0x33;
        SDOStatusWordShould_.data[5] = 0x00;
        SDOStatusWordShould_.data[6] = 0x00;
        SDOStatusWordShould_.data[7] = 0x00;

        WrongFunctionCode_.can_id = 0x781;
        WrongFunctionCode_.can_dlc = 0x08;
        WrongFunctionCode_.data[0] = 0x4F;
        WrongFunctionCode_.data[1] = 0x61;
        WrongFunctionCode_.data[2] = 0x60;
        WrongFunctionCode_.data[3] = 0x00;
        WrongFunctionCode_.data[4] = 0x44;
        WrongFunctionCode_.data[5] = 0x00;
        WrongFunctionCode_.data[6] = 0x00;
        WrongFunctionCode_.data[7] = 0x00;

        SDOWrongCommandCode_.can_id = 0x581;
        SDOWrongCommandCode_.can_dlc = 0x08;
        SDOWrongCommandCode_.data[0] = 0x43;
        SDOWrongCommandCode_.data[1] = 0x61;
        SDOWrongCommandCode_.data[2] = 0x60;
        SDOWrongCommandCode_.data[3] = 0x00;
        SDOWrongCommandCode_.data[4] = 0x44;
        SDOWrongCommandCode_.data[5] = 0x00;
        SDOWrongCommandCode_.data[6] = 0x00;
        SDOWrongCommandCode_.data[7] = 0x00;

        SDOError_.can_id = 0x581;
        SDOError_.can_dlc = 0x08;
        SDOError_.data[0] = 0x80;
        SDOError_.data[1] = 0xFF;
        SDOError_.data[2] = 0x60;
        SDOError_.data[3] = 0x00;
        SDOError_.data[4] = 0x44;
        SDOError_.data[5] = 0x00;
        SDOError_.data[6] = 0x00;
        SDOError_.data[7] = 0x00;

        PDO1Should_.can_id = 0x183;
        PDO1Should_.can_dlc = 0x08;
        PDO1Should_.data[0] = 0x4F;
        PDO1Should_.data[1] = 0x61;
        PDO1Should_.data[2] = 0x60;
        PDO1Should_.data[3] = 0x01;
        PDO1Should_.data[4] = 0x44;
        PDO1Should_.data[5] = 0x00;
        PDO1Should_.data[6] = 0x00;
        PDO1Should_.data[7] = 0x00;

        WrongPDO_.can_id = 0x481;
        WrongPDO_.can_dlc = 0x08;
        WrongPDO_.data[0] = 0x43;
        WrongPDO_.data[1] = 0xD3;
        WrongPDO_.data[2] = 0x30;
        WrongPDO_.data[3] = 0x01;
        WrongPDO_.data[4] = 0x40;
        WrongPDO_.data[5] = 0x00;
        WrongPDO_.data[6] = 0x00;
        WrongPDO_.data[7] = 0x00;
    }

    can_frame SDOPositionShould_;
    can_frame SDOStatusWordShould_;

    can_frame WrongFunctionCode_;
    can_frame SDOWrongCommandCode_;
    can_frame SDOError_;

    can_frame PDO1Should_;
    can_frame WrongPDO_;

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::unique_ptr<ObjectDictionary> sut_;
};

TEST_F(ObjectDictionaryShould, throwsExceptionOnBadConfiguration) {
    ASSERT_THROW(sut_.reset(new ObjectDictionary(testDirName_+"badConfiguration.json")),
        std::runtime_error);
}

TEST_F(ObjectDictionaryShould, failsToAddTwiceSameRegister) {
    ASSERT_THROW(sut_.reset(new ObjectDictionary(testDirName_+"twiceSameRegister.json")),
        std::runtime_error);
    ASSERT_THROW(sut_.reset(new ObjectDictionary(testDirName_+"twiceSameSubRegister.json")),
        std::runtime_error);
}

TEST_F(ObjectDictionaryShould, failsToSetDataIfRegistersDontExist) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));

    ASSERT_EQ(sut_->getRegistersVector().size(), 2);
    ASSERT_FALSE(sut_->setData(SDOPositionShould_));
    ASSERT_FALSE(sut_->setData(PDO1Should_));
}

TEST_F(ObjectDictionaryShould, correctlyReturnTimeoutIfNoSdoReceived) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    auto vector = sut_->getRegistersVector();
    ASSERT_FALSE(sut_->waitForSdoResponse(vector[0], std::chrono::milliseconds(10)));
}

TEST_F(ObjectDictionaryShould, failsTwoAddAnExistingRegister) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x60FF, 0);
    ASSERT_FALSE(sut_->addRegister(currentPosition));

    ObjectDictionaryRegister targetVelocity("targetVelocity", 4, 0x602F, 0);
    ASSERT_FALSE(sut_->addRegister(targetVelocity));

    ASSERT_EQ(currentPosition.getIndex(), sut_->getRegister(0x60FF).get().getIndex());
}

TEST_F(ObjectDictionaryShould, returnsNoneWithNonExistingRegisters) {
    sut_.reset(new ObjectDictionary(testDirName_+"CiA402.json"));
    std::vector<ObjectDictionaryRegister> registers;

    ASSERT_FALSE(sut_->getRegister("actualMozzarella"));
    ASSERT_FALSE(sut_->getRegister(0x4839, 0x10));
}

TEST_F(ObjectDictionaryShould, correctlySetNMTState) {
    can_frame frame = {};
    frame.can_id = 0x701;
    frame.can_dlc = 0x01;
    frame.data[0] = 0x7F;

    sut_.reset(new ObjectDictionary(testDirName_+"CiA402.json"));
    ASSERT_TRUE(sut_->setData(frame));
    ASSERT_EQ(sut_->getNMTState().first, 0x7F);
}

TEST_F(ObjectDictionaryShould, failsToAddTooLargePDO) {
    sut_.reset(new ObjectDictionary(testDirName_+"CiA402.json"));
    std::vector<ObjectDictionaryRegister> registers;
    registers.push_back(sut_->getRegister("actualVelocity").get());
    registers.push_back(sut_->getRegister("actualPosition").get());
    registers.push_back(sut_->getRegister("targetTorque").get());

    ASSERT_FALSE(sut_->addPDOMapping(0x01, registers));
}

TEST_F(ObjectDictionaryShould, allowResettingPDO) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x6064, 0);

    std::vector<ObjectDictionaryRegister> mapping;
    mapping.push_back(currentPosition);

    ASSERT_TRUE(sut_->addRegister(currentPosition));
    ASSERT_TRUE(sut_->addPDOMapping(1, mapping));
    ASSERT_TRUE(sut_->addPDOMapping(1, mapping));

    ASSERT_EQ(currentPosition.getIndex(), sut_->getRegister(0x6064).get().getIndex());
}

TEST_F(ObjectDictionaryShould, addRegisterDuringPDOMappingIfDidntExist) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x6064, 0);

    std::vector<ObjectDictionaryRegister> mapping;
    mapping.push_back(currentPosition);

    ASSERT_TRUE(sut_->addPDOMapping(1, mapping));

    ASSERT_EQ(currentPosition.getIndex(), sut_->getRegister(0x6064).get().getIndex());
}

TEST_F(ObjectDictionaryShould, correctlyAddSDOValue) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x6064, 0);
    ObjectDictionaryRegister statusWord("TargetVel", 1, 0x6041, 0);

    ASSERT_TRUE(sut_->addRegister(currentPosition));
    ASSERT_TRUE(sut_->addRegister(statusWord));

    ASSERT_TRUE(sut_->setData(SDOPositionShould_));
    ASSERT_TRUE(sut_->setData(SDOStatusWordShould_));
    ASSERT_EQ(sut_->getRegister(0x6064).get().getValue<int32_t>(), 0x51);
    ASSERT_EQ(sut_->getRegister(0x6041).get().getValue<uint8_t>(), 0x33);
}

TEST_F(ObjectDictionaryShould, correctlyAddPDOValues) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x6064, 0);
    ObjectDictionaryRegister currentVelocity("TargetVel", 4, 0x6065, 0);

    std::vector<ObjectDictionaryRegister> mapping;
    mapping.push_back(currentPosition);
    mapping.push_back(currentVelocity);

    ASSERT_TRUE(sut_->addPDOMapping(3, mapping));

    ASSERT_TRUE(sut_->setData(PDO1Should_));
    ASSERT_EQ(sut_->getRegister(0x6064).get().getValue<int32_t>(), 23093583);
    ASSERT_EQ(sut_->getRegister(0x6065).get().getValue<int32_t>(), 0x44);
}

TEST_F(ObjectDictionaryShould, failsToParseBadPdo) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x6064, 0);
    ObjectDictionaryRegister currentVelocity("TargetVel", 4, 0x6065, 0);

    std::vector<ObjectDictionaryRegister> mapping;
    mapping.push_back(currentPosition);
    mapping.push_back(currentVelocity);

    ASSERT_TRUE(sut_->addPDOMapping(3, mapping));

    ASSERT_FALSE(sut_->setData(WrongPDO_));
}

TEST_F(ObjectDictionaryShould, failsToParseWrongSdos) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x6064, 0);
    ObjectDictionaryRegister currentVelocity("TargetVel", 4, 0x6065, 0);

    std::vector<ObjectDictionaryRegister> mapping;
    mapping.push_back(currentPosition);
    mapping.push_back(currentVelocity);

    ASSERT_TRUE(sut_->addPDOMapping(3, mapping));

    ASSERT_FALSE(sut_->setData(WrongFunctionCode_));
    ASSERT_FALSE(sut_->setData(SDOWrongCommandCode_));
}

TEST_F(ObjectDictionaryShould, returnsSDOErrorOnWait) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    std::mutex mutex;
    std::condition_variable cv;
    std::unique_lock<std::mutex> lck(mutex);
    auto future = std::async(std::launch::async, [this, &mutex, &cv]() {
        {
            std::unique_lock<std::mutex> lck(mutex);
            cv.notify_all();
        }
        auto retval = sut_->waitForSdoResponse(
            sut_->getRegister("targetVelocity").get(), std::chrono::seconds(1));
        return retval;
    });

    cv.wait(lck);
    ASSERT_TRUE(sut_->setData(SDOError_));

    auto retval = future.get();
    ASSERT_TRUE(retval);
    ASSERT_FALSE(retval->second);
}

TEST_F(ObjectDictionaryShould, waitOnlyOnSecondSDOWorks) {
    sut_.reset(new ObjectDictionary(testDirName_+"correctConfiguration.json"));
    ObjectDictionaryRegister currentPosition("TargetPos", 4, 0x6064, 0);
    ObjectDictionaryRegister statusWord("TargetVel", 1, 0x6041, 0);

    ASSERT_TRUE(sut_->addRegister(currentPosition));
    ASSERT_TRUE(sut_->addRegister(statusWord));

    ASSERT_TRUE(sut_->setData(SDOPositionShould_));
    ASSERT_TRUE(sut_->setData(SDOStatusWordShould_));
    ASSERT_TRUE(sut_->setData(SDOPositionShould_));

    auto retval = sut_->waitForSdoResponse(statusWord, std::chrono::milliseconds(100));
    ASSERT_TRUE(retval);
    ASSERT_TRUE(retval->second);
    ASSERT_EQ(sut_->getRegister(0x6064).get().getValue<int32_t>(), 0x51);
    ASSERT_EQ(sut_->getRegister(0x6041).get().getValue<uint8_t>(), 0x33);
    retval = sut_->waitForSdoResponse(statusWord, std::chrono::milliseconds(10));
    ASSERT_FALSE(retval);
    retval = sut_->waitForSdoResponse(currentPosition, std::chrono::milliseconds(10));
    ASSERT_TRUE(retval);
    ASSERT_TRUE(retval->second);
    retval = sut_->waitForSdoResponse(statusWord, std::chrono::milliseconds(10));
    ASSERT_FALSE(retval);
    retval = sut_->waitForSdoResponse(currentPosition, std::chrono::milliseconds(10));
    ASSERT_FALSE(retval);

    ASSERT_TRUE(sut_->setData(SDOPositionShould_));
    ASSERT_TRUE(sut_->setData(SDOStatusWordShould_));
    ASSERT_TRUE(sut_->setData(SDOPositionShould_));
    retval = sut_->waitForSdoResponse(currentPosition, std::chrono::milliseconds(10));
    ASSERT_TRUE(retval);
    ASSERT_TRUE(retval->second);
    retval = sut_->waitForSdoResponse(statusWord, std::chrono::milliseconds(10));
    ASSERT_FALSE(retval);
}

