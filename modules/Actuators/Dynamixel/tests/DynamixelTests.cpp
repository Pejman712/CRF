/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro & Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include "Dynamixel/Dynamixel.hpp"
#include "Dynamixel/DynamixelConfiguration.hpp"
#include "Dynamixel/DynamixelSDK.hpp"
#include "Dynamixel/IDynamixel.hpp"
#include "EventLogger/EventLogger.hpp"

#include "Mocks/Devices/DynamixelSDKMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using ::testing::Return;

class DynamixelShould : public ::testing::Test {
 protected:
    DynamixelShould() : logger_("DynamixelShould"), currentPosition_(0) {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        sdkInterfaceMock_.reset(new NiceMock<crf::devices::dynamixelstepper::DynamixelSDKMock>);
        std::string testAddress = __FILE__;
        testAddress = testAddress.substr(0, testAddress.find("DynamixelTests.cpp"));
        std::ifstream dxl_data(testAddress +
            "./../../../modules/Devices/Dynamixel/configuration/DXL-AX-12A.json");
        dxl_data >> dxl_JSON_;
    }

    void SetUp() override {
        ON_CALL(*sdkInterfaceMock_, getPortHandler(_)).
            WillByDefault(Invoke([this](const char *port_name) {
                dynamixel::PortHandler* portHandler = NULL;
                return portHandler;
            }));
        ON_CALL(*sdkInterfaceMock_, getPacketHandler(_)).
            WillByDefault(Invoke([this](float protocol_version) {
                dynamixel::PacketHandler* packetHandler = NULL;
                return packetHandler;
            }));
        ON_CALL(*sdkInterfaceMock_, openPort(_)).WillByDefault(Return(true));
        ON_CALL(*sdkInterfaceMock_, closePort(_)).WillByDefault(Return());
        ON_CALL(*sdkInterfaceMock_, setBaudRate(_, _)).WillByDefault(Return(true));
        ON_CALL(*sdkInterfaceMock_, write1ByteTxRx(_, _, _, _, _, _)).WillByDefault(Return(0));
        ON_CALL(*sdkInterfaceMock_, write2ByteTxRx(_, _, _, _, _, _)).WillByDefault(Invoke([this]
            (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
            uint8_t id, uint16_t address, uint16_t data, uint8_t *error = 0) {
                auto dxl_limits = dxl_JSON_.at("Limits")[0];
                int maxValue = dxl_limits.at("MaxPosValue").get<uint16_t>();
                if (data > maxValue) {
                    currentPosition_ = maxValue;
                } else {
                    currentPosition_ = data;
                }
                *error = 0;
                return 0;
            }));
        ON_CALL(*sdkInterfaceMock_, read2ByteTxRx(_, _, _, _, _, _)).WillByDefault(Invoke([this]
            (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
            uint8_t id, uint16_t address, uint16_t *data, uint8_t *error = 0) {
                *error = 0;
                return *data = currentPosition_;
            }));
        ON_CALL(*sdkInterfaceMock_, getRxPacketError(_, _)).WillByDefault(Return(""));
    }

    ~DynamixelShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    int currentPosition_;

    std::shared_ptr<crf::devices::dynamixelstepper::DynamixelSDKMock> sdkInterfaceMock_;
    std::unique_ptr<crf::devices::dynamixelstepper::IDynamixel> dxl_;
    nlohmann::json dxl_JSON_;
};

TEST_F(DynamixelShould, initializeAndDeinitializeTwice) {
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));

    ASSERT_TRUE(dxl_->initialize());
    ASSERT_FALSE(dxl_->initialize());
    ASSERT_TRUE(dxl_->deinitialize());
    ASSERT_FALSE(dxl_->deinitialize());

    ASSERT_TRUE(dxl_->initialize());
    ASSERT_FALSE(dxl_->initialize());
    ASSERT_TRUE(dxl_->deinitialize());
    ASSERT_FALSE(dxl_->deinitialize());
}

TEST_F(DynamixelShould, initializeReturnsFalseIfBadJsonFile) {
    nlohmann::json emptyJson;
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        emptyJson,
        sdkInterfaceMock_));
    ASSERT_FALSE(dxl_->initialize());
}

TEST_F(DynamixelShould, initializeReturnsFalseCannotOpenPort) {
    EXPECT_CALL(*sdkInterfaceMock_, openPort(_)).WillOnce(Return(false));

    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));
    ASSERT_FALSE(dxl_->initialize());
}

TEST_F(DynamixelShould, initializeReturnsFalseCannotSetBaudRate) {
    EXPECT_CALL(*sdkInterfaceMock_, setBaudRate(_, _)).WillOnce(Return(false));

    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));
    ASSERT_FALSE(dxl_->initialize());
}

TEST_F(DynamixelShould, initializeReturnsFalseCannotWriteByte) {
    EXPECT_CALL(*sdkInterfaceMock_, write1ByteTxRx(_, _, _, _, _, _)).WillOnce(Invoke([]
        (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
            uint8_t id, uint16_t address, uint16_t data, uint8_t *error = 0) {
            *error = 7;
            return 1;
        }));
    EXPECT_CALL(*sdkInterfaceMock_, getRxPacketError(_, _)).WillOnce(
        Return("[RxPacketError] Writing or Reading is not available to target address!"));

    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));
    ASSERT_FALSE(dxl_->initialize());
}

TEST_F(DynamixelShould, deinitializeReturnsFalseCannotWriteByte) {
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));
    ASSERT_TRUE(dxl_->initialize());

    EXPECT_CALL(*sdkInterfaceMock_, write1ByteTxRx(_, _, _, _, _, _)).WillRepeatedly(Invoke([]
        (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
            uint8_t id, uint16_t address, uint16_t data, uint8_t *error = 0) {
            *error = 7;
            return 1;
        }));
    EXPECT_CALL(*sdkInterfaceMock_, getRxPacketError(_, _)).WillRepeatedly(
        Return("[RxPacketError] Writing or Reading is not available to target address!"));

    ASSERT_FALSE(dxl_->deinitialize());
}

TEST_F(DynamixelShould, returnTheCurrentPosition) {
    int position = 0;
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_, sdkInterfaceMock_));

    ASSERT_TRUE(dxl_->initialize());
    position = dxl_->readCurrentPosition(0);
    ASSERT_EQ(0, position);
    ASSERT_TRUE(dxl_->writeDynamixel(0, 33));
    position = dxl_->readCurrentPosition(0);
    ASSERT_EQ(33, position);
    ASSERT_TRUE(dxl_->writeDynamixel(0, 0));
    position = dxl_->readCurrentPosition(0);
    ASSERT_EQ(dxl_->getConfiguration().getMotorsChain()[0].limits.DXLMinPosValue, position);
    ASSERT_TRUE(dxl_->writeDynamixel(0, 919));
    ASSERT_EQ(919, dxl_->readCurrentPosition(0));
    ASSERT_TRUE(dxl_->writeDynamixel(0, 5555));
    ASSERT_EQ(dxl_->getConfiguration().getMotorsChain()[0].limits.DXLMaxPosValue, dxl_->readCurrentPosition(0));  // NOLINT
    ASSERT_TRUE(dxl_->deinitialize());
}

TEST_F(DynamixelShould, failToReadCurrentPosition) {
    EXPECT_CALL(*sdkInterfaceMock_, read2ByteTxRx(_, _, _, _, _, _)).WillOnce(Invoke([]
        (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
        uint8_t id, uint16_t address, uint16_t *data, uint8_t *error = 0) {
            *data = 0;
            *error = 1;
            return 0;
        }));
    EXPECT_CALL(*sdkInterfaceMock_, getRxPacketError(_, _)).WillRepeatedly(
        Return("[RxPacketError] Failed to process the instruction packet!"));

    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));

    ASSERT_TRUE(dxl_->initialize());
    int position = dxl_->readCurrentPosition(0);
    ASSERT_TRUE(dxl_->deinitialize());
    ASSERT_EQ(position, -1);
}

TEST_F(DynamixelShould, returnsFalseIfOutOfRangePosition) {
    EXPECT_CALL(*sdkInterfaceMock_, write2ByteTxRx(_, _, _, _, _, _)).WillOnce(Invoke([]
        (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
        uint8_t id, uint16_t address, uint16_t data, uint8_t *error = 0) {
            *error = 4;
            return 1;
        }));
    EXPECT_CALL(*sdkInterfaceMock_, getRxPacketError(_, _)).WillRepeatedly(
        Return("[RxPacketError] The data value is out of range!"));

    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));

    ASSERT_TRUE(dxl_->initialize());
    ASSERT_FALSE(dxl_->writeDynamixel(0, 6666));
    ASSERT_TRUE(dxl_->deinitialize());
}

TEST_F(DynamixelShould, getZeroPosition) {
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));

    ASSERT_TRUE(dxl_->initialize());
    std::vector<uint16_t> zeroPosition = dxl_->getZeroPosition();
    ASSERT_TRUE(dxl_->deinitialize());

    for (int i = 0; i < static_cast<int>(zeroPosition.size()); i++)
        ASSERT_EQ(zeroPosition[i], dxl_JSON_.at("Limits")[i].at("StartValue").get<uint16_t>());
}

TEST_F(DynamixelShould, getConfiguration) {
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));

    ASSERT_TRUE(dxl_->initialize());
    crf::devices::dynamixelstepper::DynamixelConfiguration config = dxl_->getConfiguration();
    ASSERT_TRUE(dxl_->deinitialize());

    ASSERT_EQ(config.getNumberOfMotors(), dxl_JSON_.at("NumberOfMotors").get<int>());
    auto motors = config.getMotorsChain();
    for (int i = 0; i < config.getNumberOfMotors(); i++) {
        ASSERT_EQ(motors[i].dxl_id, dxl_JSON_.at("dxl_id").get<std::vector<int>>()[i]);
        ASSERT_EQ(motors[i].mType, dxl_JSON_.at("MotorType").get<std::string>());

        auto dxl_limits = dxl_JSON_.at("Limits")[i];
        ASSERT_EQ(motors[i].limits.DXLMinPosValue,
            dxl_limits.at("MinPosValue").get<uint16_t>());
        ASSERT_EQ(motors[i].limits.DXLMaxPosValue,
            dxl_limits.at("MaxPosValue").get<uint16_t>());
        ASSERT_EQ(motors[i].limits.DXLStartValue,
            dxl_limits.at("StartValue").get<uint16_t>());
        ASSERT_EQ(motors[i].pulseMovement, dxl_JSON_.at("PulseMovement").get<int>());
        ASSERT_EQ(motors[i].unitDegree, dxl_JSON_.at("Unit_degree").get<double>());
    }
}

TEST_F(DynamixelShould, writeDynamixelWithoutInitialization) {
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));
    ASSERT_FALSE(dxl_->writeDynamixel(0, 0));
}

TEST_F(DynamixelShould, readCurrentPositionWithoutInitialization) {
    dxl_.reset(new crf::devices::dynamixelstepper::Dynamixel("dev/ttyUSB0",
        dxl_JSON_,
        sdkInterfaceMock_));
    int position = dxl_->readCurrentPosition(0);
    ASSERT_EQ(-1, position);
}
