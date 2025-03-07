/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */


#include <condition_variable>
#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPoint.hpp"
#include "RGBDCameras/RGBDCameraMockConfiguration.hpp"
#include "DataPackets/RGBDFramePacket/RGBDFramePacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "RGBDCameras/RGBDCameraMock.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::sensors::rgbdcameras::RGBDCameraMock;
using crf::sensors::rgbdcameras::RGBDCameraMockConfiguration;
using crf::sensors::rgbdcameras::RGBDCameraManager;
using crf::sensors::rgbdcameras::RGBDCameraCommunicationPoint;

using crf::communication::datapackets::RGBDFramePacket;
using crf::communication::datapacketsocket::PacketSocket;
using crf::communication::datapackets::JSONPacket;
using crf::communication::sockets::SocketMock;

class RGBDCameraCommunicationPointShould : public ::testing::Test {
 protected:
    RGBDCameraCommunicationPointShould() :
        logger_("RGBDCameraCommunicationPointShould"),
        isSocketOpen_(true),
        readMutex_(),
        readCv_(),
        bytesToRead_(),
        writeMutex_(),
        writeCv_(),
        dataWritten_(),
        bytesWritten_(),
        socketMock_(std::make_shared<NiceMock<SocketMock>>()),
        cameraMock_(std::make_shared<NiceMock<RGBDCameraMockConfiguration>>()),
        manager_(std::make_shared<RGBDCameraManager>(cameraMock_)),
        packetSocket_(std::make_shared<PacketSocket>(socketMock_)),
        sut_(std::make_unique<RGBDCameraCommunicationPoint>(packetSocket_, manager_)) {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
        }

    void SetUp() override {
        bytesWritten_.clear();
        bytesToRead_.clear();

        ON_CALL(*socketMock_, isOpen()).WillByDefault(Invoke([this]() { return isSocketOpen_; }));
        ON_CALL(*socketMock_, open()).WillByDefault(Invoke([this]() {
            isSocketOpen_ = true;
            return true;
        }));

        ON_CALL(*socketMock_, close()).WillByDefault(Invoke([this]() {
            std::unique_lock<std::mutex> lock(readMutex_);
            isSocketOpen_ = false;
            readCv_.notify_all();
            return true;
        }));

        ON_CALL(*socketMock_, read(_)).WillByDefault(Invoke([this](int length) {
            std::unique_lock<std::mutex> lock(readMutex_);

            while (bytesToRead_.length() < static_cast<size_t>(length)) {
                readCv_.wait_for(lock, std::chrono::milliseconds(10));
                if (!isSocketOpen_) {
                    return std::string();
                }
            }

            std::string bytes = bytesToRead_.substr(0, length);
            bytesToRead_ = bytesToRead_.substr(length, bytesToRead_.length());
            return bytes;
        }));

        ON_CALL(*socketMock_, write(_, _)).WillByDefault(
            Invoke([this](std::string buffer, bool ack) {
                std::unique_lock<std::mutex> lock(writeMutex_);
                if (!isSocketOpen_) {
                    return false;
                }

                bytesWritten_.append(buffer);
                dataWritten_ = true;
                writeCv_.notify_all();
                return true;
        }));
    }

    ~RGBDCameraCommunicationPointShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void writePacket(const crf::communication::datapackets::IPacket& packet) {
        std::unique_lock<std::mutex> lock(readMutex_);
        bytesToRead_.append("~~~~~~");
        bytesToRead_.append(packet.getHeader().serialize());
        bytesToRead_.append(packet.serialize());
        readCv_.notify_all();
    }

    std::optional<JSONPacket> readJsonPacket() {
        {
            std::unique_lock<std::mutex> lock(writeMutex_);
            dataWritten_ = false;
            if (bytesWritten_.length() == 0) {
                if (!writeCv_.wait_for(lock, std::chrono::seconds(2), [this] {
                    return dataWritten_; })) {
                        return std::nullopt;
                }
            }
        }

        std::string sync = bytesWritten_.substr(0, 6);
        if (sync != "~~~~~~") {
            logger_->warn("Missed sync");
            return std::nullopt;
        }

        crf::communication::datapackets::PacketHeader header;
        std::string headerBytes = bytesWritten_.substr(6, header.size());
        if (!header.deserialize(headerBytes)) {
            logger_->warn("Failed to deserialize header");
            return std::nullopt;
        }

        JSONPacket json;
        std::string jsonBytes = bytesWritten_.substr(6 + header.size(), header.length());
        if (!json.deserialize(jsonBytes)) {
            logger_->warn("Failed to deserialize packet: {}", jsonBytes);
            return std::nullopt;
        }

        bytesWritten_ = bytesWritten_.erase(0, 6 + header.size() + header.length());

        return json;
    }

    std::optional<crf::communication::datapackets::RGBDFramePacket> readRGBDFramePacket() {
        std::unique_lock<std::mutex> lock(writeMutex_);
        dataWritten_ = false;
        if (bytesWritten_.length() == 0) {
            writeCv_.wait(lock);
        }


        std::string sync = bytesWritten_.substr(0, 6);
        if (sync != "~~~~~~") {
            logger_->warn("Missed sync");
            return std::nullopt;
        }

        crf::communication::datapackets::PacketHeader header;
        std::string headerBytes = bytesWritten_.substr(6, header.size());
        if (!header.deserialize(headerBytes)) {
            logger_->warn("Failed to deserialize header");
            return std::nullopt;
        }

        crf::communication::datapackets::RGBDFramePacket frame;
        std::string frameBytes = bytesWritten_.substr(6 + header.size(), header.length());
        if (!frame.deserialize(frameBytes)) {
            logger_->warn("Failed to deserialize packet");
        }
        bytesWritten_ = bytesWritten_.erase(0, 6 + header.size() + header.length());

        return frame;
    }

    crf::utility::logger::EventLogger logger_;

    bool isSocketOpen_;
    std::mutex readMutex_;
    std::condition_variable readCv_;
    std::string bytesToRead_;
    std::mutex writeMutex_;
    std::condition_variable writeCv_;
    bool dataWritten_;
    std::string bytesWritten_;

    std::shared_ptr<SocketMock> socketMock_;
    std::shared_ptr<NiceMock<RGBDCameraMockConfiguration>> cameraMock_;
    std::shared_ptr<RGBDCameraManager> manager_;
    std::shared_ptr<PacketSocket> packetSocket_;
    std::unique_ptr<RGBDCameraCommunicationPoint> sut_;
};

TEST_F(RGBDCameraCommunicationPointShould, initializeDeinitializeSequence) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, opensTheSocketIfItWasNotOpen) {
    isSocketOpen_ = false;
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(isSocketOpen_);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, sendErrorOnWrongPacket) {
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::RGBDFramePacket frame;
    writePacket(frame);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Not supported packet type");
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, sendErrorIfMissingCmd) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["commanda"] = "Uros Djurdjevic";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Cannot get command field from received json");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, sendErrorIfUnknownCommand) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "Puxa Sporting";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Unknown command: Puxa Sporting");
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, correctlySetAndGetCameraParameters) {
    // EXPECT_CALL(*RGBDCameraConfig_->getMock(), setZoom(1.0f)).Times(1);
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "setProperty";
    json.data["priority"] = 1;
    json.data["property"]["zoom"] = 1.0f;
    writePacket(json);
    auto response = readJsonPacket();

    json.data["command"] = "getStatus";
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");
    ASSERT_NEAR(
        response.value().data["message"]["property"]["zoom"].get<crf::expected<float>>(),
        1.0, 1e-3);
}

TEST_F(RGBDCameraCommunicationPointShould, getErrorOnNotActiveStopStream) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "stopStreamStatus";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Stream was not active");
}

TEST_F(RGBDCameraCommunicationPointShould, getErrorOnWrongMessageStartStream) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["resolution"] = "startFrameStream";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");

    json.data.erase("resolution");
    json.data["resolution"]["width"] = 100;
    writePacket(json);
    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");

    json.data["resolution"]["height"] = 768;
    writePacket(json);
    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");

    json.data["format"] = "ciao";
    writePacket(json);
    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");

    json.data["format"] = "jpeg";
    writePacket(json);
    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");

    json.data["quality"] = 100;
    writePacket(json);
    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");

    json.data["quality"] = 5;
    writePacket(json);
    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["replyCommand"], "error");
}

/** This test is disabled because it will fail with valgrind and stress,
 * but it works in normal operation **/
TEST_F(RGBDCameraCommunicationPointShould, DISABLED_correctlyReceiveFramesWith30Fps) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["encoding_format"] = 1;
    json.data["encoding_quality"] = 5;
    writePacket(json);

    auto responseJSON = readJsonPacket();
    ASSERT_TRUE(responseJSON);
    ASSERT_EQ(responseJSON.value().data["command"], "reply");
    ASSERT_EQ(responseJSON.value().data["replyCommand"], "startFrameStream");
    ASSERT_EQ(responseJSON.value().data["message"], true);

    const int numberOfFramesToRead = 10;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfFramesToRead; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);
        auto bytes = response.value().getRGBBytes();
        auto init = bytes.begin()+4;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    ASSERT_GT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead-1));
    ASSERT_LT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead+1));
}

/** This test is disabled because it will fail with valgrind and stress,
 * but it works in normal operation **/
TEST_F(RGBDCameraCommunicationPointShould, DISABLED_correctlyReceiveFramesWith5Fps) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["encoding_format"] = 1;
    json.data["encoding_quality"] = 5;
    writePacket(json);

    auto responseJSON = readJsonPacket();
    ASSERT_TRUE(responseJSON);
    ASSERT_EQ(responseJSON.value().data["command"], "reply");
    ASSERT_EQ(responseJSON.value().data["replyCommand"], "startFrameStream");
    ASSERT_EQ(responseJSON.value().data["message"], true);

    const int numberOfFramesToRead = 10;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfFramesToRead; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);
        auto bytes = response.value().getRGBBytes();
        auto init = bytes.begin()+4;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    ASSERT_GT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead-1));
    ASSERT_LT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead+1));
}

TEST_F(RGBDCameraCommunicationPointShould, startStreamsJpegAndCorrectlyReceiveFrames) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["encoding_format"] = 1;
    json.data["encoding_quality"] = 5;
    writePacket(json);

    auto responseJSON = readJsonPacket();
    ASSERT_TRUE(responseJSON);
    ASSERT_EQ(responseJSON.value().data["command"], "reply");
    ASSERT_EQ(responseJSON.value().data["replyCommand"], "startFrameStream");
    ASSERT_EQ(responseJSON.value().data["message"], true);

    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);
        auto bytes = response.value().getRGBBytes();
        auto init = bytes.begin()+8;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }
    json.data.clear();
    json.data["command"] = "setProfile";
    json.data["image_profile"] = Profile(cv::Size(150, 150), 5);
    writePacket(json);

    for (int  i= 0; i < 10; i++)
        auto response = readRGBDFramePacket();

    bool resolutionChanged = false;

    do {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);

        auto bytes = response.value().getRGBBytes();
        auto init = bytes.begin()+8;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        if (mat.cols == 100) {
            ASSERT_EQ(mat.cols, 100);
            ASSERT_EQ(mat.rows, 100);
        } else if (mat.cols == 150) {
            ASSERT_EQ(mat.cols, 150);
            ASSERT_EQ(mat.rows, 150);
            resolutionChanged = true;
        }
    } while (!resolutionChanged);
}

TEST_F(RGBDCameraCommunicationPointShould, startStreamsx264AndCorrectlyReceiveFrames) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["encoding_format"] = 3;
    json.data["encoding_quality"] = 5;

    writePacket(json);
    auto responseJSON = readJsonPacket();
    ASSERT_TRUE(responseJSON);
    ASSERT_EQ(responseJSON.value().data["command"], "reply");
    ASSERT_EQ(responseJSON.value().data["replyCommand"], "startFrameStream");
    ASSERT_EQ(responseJSON.value().data["message"], true);

    for (int i=0; i < 5; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::X264);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);
    }

    json.data["resolution"]["width"] = 150;
    json.data["resolution"]["height"] = 150;
    writePacket(json);

    for (int i = 0; i < 10; i++)
        auto response = readRGBDFramePacket();

    for (int i=0; i < 5; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::X264);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);
    }

    json.data["command"] = "stopFrameStream";
    writePacket(json);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

TEST_F(RGBDCameraCommunicationPointShould, correctlyChangeCompressionMethod) {
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["encoding_format"] = 1;
    json.data["encoding_quality"] = 5;
    writePacket(json);
    auto responseJSON = readJsonPacket();
    ASSERT_TRUE(responseJSON);
    ASSERT_EQ(responseJSON.value().data["command"], "reply");
    ASSERT_EQ(responseJSON.value().data["replyCommand"], "startFrameStream");
    ASSERT_EQ(responseJSON.value().data["message"], true);

    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);
        auto bytes = response.value().getRGBBytes();
        auto init = bytes.begin()+8;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }
}


TEST_F(RGBDCameraCommunicationPointShould, correctlyChangeDepthCompressionMethod) {
    ASSERT_TRUE(sut_->initialize());
    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["encoding_format"] = 1;
    json.data["encoding_quality"] = 5;
    json.data["depth_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["depth_format"] = 1;

    writePacket(json);

    auto responseJSON = readJsonPacket();
    ASSERT_TRUE(responseJSON);
    ASSERT_EQ(responseJSON.value().data["command"], "reply");
    ASSERT_EQ(responseJSON.value().data["replyCommand"], "startFrameStream");
    ASSERT_EQ(responseJSON.value().data["message"], true);

    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getDepthEncoding(),
            crf::communication::datapackets::RGBDFramePacket::DepthEncoding::CV_MAT);
        ASSERT_NE(response.value().getDepthBytes().length(), 0);
    }
}

TEST_F(RGBDCameraCommunicationPointShould, requestBothDepthAndColor) {
    ASSERT_TRUE(sut_->initialize());
    JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["image_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["encoding_format"] = 1;
    json.data["encoding_quality"] = 5;
    json.data["depth_profile"] = Profile(cv::Size(100, 100), 5);
    json.data["depth_format"] = 2;

    writePacket(json);
    auto responseJSON = readJsonPacket();
    ASSERT_TRUE(responseJSON);
    ASSERT_EQ(responseJSON.value().data["command"], "reply");
    ASSERT_EQ(responseJSON.value().data["replyCommand"], "startFrameStream");
    ASSERT_EQ(responseJSON.value().data["message"], true);

    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        std::optional<crf::communication::datapackets::RGBDFramePacket> response =
            readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.value().getRGBBytes().length(), 0);
        ASSERT_EQ(response.value().getDepthEncoding(),
            crf::communication::datapackets::RGBDFramePacket::DepthEncoding::LZ4);
        ASSERT_NE(response.value().getDepthBytes().length(), 0);
    }
}
