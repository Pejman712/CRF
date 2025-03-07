/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/StreamWriter.hpp"
#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/FetchWritePacket.hpp"
#include "TIMAlignment/TIMAlignment.hpp"
#include "VisionTypes/VisionTypes.hpp"

#include "Mocks/Communication/NetworkClientMock.hpp"
#include "Mocks/Communication/RequestMock.hpp"
#include "Mocks/Vision/ObjectDetectorMock.hpp"

using crf::applications::timalignment::TIMAlignment;
using crf::communication::networkclient::NetworkClientMock;
using crf::communication::zmqcomm::RequestMock;
using crf::vision::objectDetection::ObjectDetectorMock;
using crf::vision::types::BoundingBox;

using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;

using namespace std::chrono_literals;   // NOLINT

class TIMAlignmentShould: public ::testing::Test {
 protected:
    TIMAlignmentShould(): logger_("TIMAlignmentShould") {
        logger_->info("{0} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testAddress_ = __FILE__;
        testAddress_ = testAddress_.substr(0, testAddress_.find("TIMAlignmentTests.cpp"));
        std::ifstream bboxes(testAddress_ + "./config/boundingboxes.json");
        std::string requestAddress = "inproc://testTIMAlignment";
        bboxes >> bboxes_;
        client_.reset(new NiceMock<NetworkClientMock>);
        objectDetector_.reset(new NiceMock<ObjectDetectorMock>);
        packet_.dataBlockNumber_ = 224;
        packet_.startAddress_ = 0;
        packet_.dataLength_ = 2;
        frame_ = cv::Mat::zeros(480, 640, CV_8UC3);
        EXPECT_CALL(*client_, connect(_)).WillRepeatedly(
            Return(true));
        EXPECT_CALL(*client_, disconnect()).WillRepeatedly(
            Return(true));
        EXPECT_CALL(*objectDetector_, initialize()).WillRepeatedly(
            Return(true));
        EXPECT_CALL(*objectDetector_, deinitialize()).WillRepeatedly(
            Return(true));

        requesterMock_.reset(new NiceMock<RequestMock>(requestAddress));
        ON_CALL(*requesterMock_, open()).WillByDefault(Return(true));
        ON_CALL(*requesterMock_, close()).WillByDefault(Return(true));
        ON_CALL(*requesterMock_, write(_, _)).WillByDefault(Return(true));
        ON_CALL(*requesterMock_, setRcvTimeOut(_)).WillByDefault(Return(true));
        ON_CALL(*requesterMock_, setSndTimeOut(_)).WillByDefault(Return(true));
        ON_CALL(*requesterMock_, read(_, _)).WillByDefault(Invoke(
            [](std::string* serializedMsg, int) {
                cv::Mat img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
                Packets::StreamWriter writer;
                writer.write(img);
                *serializedMsg = writer.toString();
                return true;
            }));

        timeOut_ = 100ms;
    }

    ~TIMAlignmentShould() {
        logger_->info("{0} END with {1}",
        testing::UnitTest::GetInstance()->current_test_info()->name(),
        testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    std::vector<BoundingBox> getBoundingBox(std::string testName) {
        std::vector<BoundingBox> boundingBoxes;
        nlohmann::json boxes = bboxes_.at(testName);
        for (const auto& box : boxes) {
            BoundingBox boundingBox;
            boundingBox.classif = box.at("classification").get<std::string>();
            auto itBox = box.at("box").get<std::vector<int>>();
            boundingBox.box = cv::Rect(cv::Point(itBox.at(0), itBox.at(1)),
                                       cv::Point(itBox.at(2), itBox.at(3)));
            boundingBoxes.push_back(boundingBox);
        }
        return boundingBoxes;
    }

    crf::utility::logger::EventLogger logger_;
    std::string testAddress_;
    nlohmann::json bboxes_;
    std::shared_ptr<NiceMock<NetworkClientMock>> client_;
    std::shared_ptr<NiceMock<ObjectDetectorMock>> objectDetector_;
    std::shared_ptr<NiceMock<RequestMock>> requesterMock_;
    Packets::FetchWritePacket packet_;
    cv::Mat frame_;
    std::condition_variable cv_;
    std::mutex m_;
    std::chrono::milliseconds timeOut_;
};

TEST_F(TIMAlignmentShould, returnTrueIfInitializeAndDeinitializeProperly) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [](const Packets::PacketHeader&, const std::string&) {
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("TwoGuidesRight");}));
    ASSERT_TRUE(align->initialize());
    ASSERT_FALSE(align->initialize());
    ASSERT_TRUE(align->deinitialize());
    ASSERT_FALSE(align->deinitialize());
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithTwoGuidesToMoveRight) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("TwoGuidesRight");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 1);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithTwoGuidesToMoveLeft) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("TwoGuidesLeft");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 2);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithTwoGuidesToStop) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("TwoGuidesStop");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 3);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithCollimatorToMoveRight) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("CollimatorRight");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 1);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithCollimatorToMoveLeft) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("CollimatorLeft");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 2);
}

TEST_F(TIMAlignmentShould, DISABLED_isEqualIfBoundingBoxesWithCollimatorToStop) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("CollimatorStop");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 3);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithOneGuideToMoveRight) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("OneGuideRight");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 1);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithOneGuideToMoveLeft) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("OneGuideLeft");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 2);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithOneGuideToStop) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("OneGuideStop");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 3);
}

TEST_F(TIMAlignmentShould, isEqualIfBoundingBoxesWithUnknownClass) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("RandomStuff");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 0);
}

TEST_F(TIMAlignmentShould, isEqualIfNotDetection) {
    std::unique_ptr<TIMAlignment> align = std::make_unique<TIMAlignment>(client_, objectDetector_,
        requesterMock_, packet_);
    std::vector<uint8_t> data;
    EXPECT_CALL(*client_, send(_, _)).WillOnce(Invoke(
        [&data, this](const Packets::PacketHeader&, const std::string& buffer) {
            Packets::FetchWritePacket packet;
            packet.deserialize(buffer);
            data = packet.data_;
            cv_.notify_one();
            return true;
        })).WillRepeatedly(Return(true));
    EXPECT_CALL(*objectDetector_, getBoundingBoxes(_)).WillRepeatedly(Invoke(
        [this](cv::Mat) {return getBoundingBox("RandomStuff");}));
    ASSERT_TRUE(align->initialize());
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, timeOut_);
    }
    ASSERT_TRUE(align->deinitialize());
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 0);
}
