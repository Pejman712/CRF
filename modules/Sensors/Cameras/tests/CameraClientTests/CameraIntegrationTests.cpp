/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "Cameras/CameraClient/CameraClient.hpp"
#include "Cameras/CameraCommunicationPoint/CameraCommunicationPointFactory.hpp"
#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Sockets/SocketMock.hpp"
#include "Cameras/CameraMock.hpp"
#include "Cameras/CameraMockConfiguration.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"
#include "Sockets/IPC/UnixSocket.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::vision::videocodecs::IVideoEncoder;
using crf::sensors::cameras::CameraMock;
using crf::sensors::cameras::CameraMockConfiguration;
using crf::sensors::cameras::CameraClient;
using crf::sensors::cameras::CameraCommunicationPointFactory;
using crf::sensors::cameras::CameraManager;
using crf::communication::communicationpointserver::CommunicationPointServer;
using crf::communication::datapacketsocket::PacketSocket;
using crf::communication::sockets::UnixSocketServer;
using crf::communication::sockets::UnixSocket;

class CamerasIntegrationShould : public ::testing::Test {
 protected:
    CamerasIntegrationShould() :
        logger_("CamerasIntegrationShould"),
        mock_(new NiceMock<CameraMock>),
        simulator_(new CameraMockConfiguration(mock_)),
        socketServer_(new UnixSocketServer("integration_test")),
        manager_(new CameraManager(mock_)),
        factory_(new CameraCommunicationPointFactory(manager_)),
        server_(new CommunicationPointServer(socketServer_, factory_)),
        clientSocket1_(new PacketSocket(std::make_shared<UnixSocket>("integration_test"))),
        clientSocket2_(new PacketSocket(std::make_shared<UnixSocket>("integration_test"))),
        cameraClient1_(new CameraClient(clientSocket1_)),
        cameraClient2_(new CameraClient(clientSocket2_)) {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
            simulator_->initializeDelay = std::chrono::milliseconds(1);
    }

    void SetUp() {
        simulator_->configureMock();
    }

    void TearDown() override {
        server_->deinitialize();
    }

    ~CamerasIntegrationShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<NiceMock<CameraMock>> mock_;
    std::unique_ptr<CameraMockConfiguration> simulator_;

    std::shared_ptr<UnixSocketServer> socketServer_;
    std::shared_ptr<CameraManager> manager_;
    std::shared_ptr<CameraCommunicationPointFactory> factory_;
    std::unique_ptr<CommunicationPointServer> server_;

    std::shared_ptr<PacketSocket> clientSocket1_;
    std::shared_ptr<PacketSocket> clientSocket2_;

    std::shared_ptr<CameraClient> cameraClient1_;
    std::shared_ptr<CameraClient> cameraClient2_;
};

TEST_F(CamerasIntegrationShould, DISABLED_correctlyGetFramesFromMultipleClientsWithChanges) {
    ASSERT_TRUE(server_->initialize());

    ASSERT_TRUE(cameraClient1_->setResolution(cv::Size(640, 480)));
    ASSERT_TRUE(cameraClient1_->setFramerate(10));
    ASSERT_TRUE(cameraClient2_->setResolution(cv::Size(1280, 720)));
    ASSERT_TRUE(cameraClient2_->setFramerate(20));

    ASSERT_TRUE(cameraClient1_->initialize());
    ASSERT_TRUE(cameraClient2_->initialize());

    for (int i=0; i < 5; i++) {
        auto frame1 = cameraClient1_->getFrame();
        ASSERT_EQ(frame1.cols, 640);
        ASSERT_EQ(frame1.rows, 480);
        auto frame2 = cameraClient2_->getFrame();
        ASSERT_EQ(frame2.cols, 1280);
        ASSERT_EQ(frame2.rows, 720);
    }

    ASSERT_TRUE(cameraClient2_->setResolution(cv::Size(640, 480)));
    ASSERT_TRUE(cameraClient2_->setFramerate(10));
    ASSERT_TRUE(cameraClient1_->setResolution(cv::Size(1280, 720)));
    ASSERT_TRUE(cameraClient1_->setFramerate(20));

    for (int i=0; i < 5; i++) {
        auto frame1 = cameraClient1_->getFrame();
        auto frame2 = cameraClient2_->getFrame();
    }

    for (int i=0; i < 5; i++) {
        auto frame1 = cameraClient2_->getFrame();
        ASSERT_EQ(frame1.cols, 640);
        ASSERT_EQ(frame1.rows, 480);
        auto frame2 = cameraClient1_->getFrame();
        ASSERT_EQ(frame2.cols, 1280);
        ASSERT_EQ(frame2.rows, 720);
    }
}
