/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <future>
#include <memory>
#include <random>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/TcpClient.hpp"
#include "NetworkServer/TcpServer.hpp"

#include "Mocks/Utility/DummyPacket.hpp"

using crf::communication::networkclient::INetworkClient;
using crf::communication::networkclient::TcpClient;
using crf::communication::networkserver::INetworkServer;
using crf::communication::networkserver::TcpServer;

class NetworkInterfacesShould: public ::testing::Test {
 protected:
    NetworkInterfacesShould(): logger_("NetworkInterfacesShould") {
        std::random_device r;
        std::default_random_engine e1(r());
        std::uniform_int_distribution<int> uniform_dist(6000, 7000);
        /*
         * Choose port randomly from [6000, 7000)
         */
        somePort_ = uniform_dist(e1);
        someAddr_ = "localhost";
        /*
         * Client <---> Server creation order doesn't matter
         */
        client_.reset(new TcpClient(someAddr_, somePort_));
        server_.reset(new TcpServer(somePort_));
    }
    crf::utility::logger::EventLogger logger_;
    int somePort_;
    std::string someAddr_;
    std::unique_ptr<INetworkClient> client_;
    std::unique_ptr<INetworkServer> server_;
};

TEST_F(NetworkInterfacesShould, transmitDataBetweenEachOther) {
    int numMsg = 100;
    /*
     * Doesn't matter which one (accept/connect) is invoked first
     */
    std::future<bool> serverAccepted = std::async(std::launch::async,
        [this]() {
            return server_->acceptConnection();
        });
    std::future<bool> clientConnected = std::async(std::launch::async,
        [this]() {
            return client_->connect();
        });
    ASSERT_TRUE(serverAccepted.get());
    ASSERT_TRUE(clientConnected.get());
    ASSERT_TRUE(server_->isConnected());
    ASSERT_TRUE(client_->isConnected());
    Packets::DummyPacket packet;
    Packets::DummyPacket receivedPacket;
    std::string buff;
    Packets::PacketHeader header = packet.getHeader();
    /*
     * Client to server test
     */
    for (int i = 0; i < numMsg; i++) {
        packet.x_ = i;
        packet.y_ = i + 69;
        ASSERT_TRUE(client_->send(header, packet.serialize()));
        ASSERT_TRUE(server_->receive(&header, &buff));
        ASSERT_TRUE(receivedPacket.deserialize(buff));
        ASSERT_EQ(packet, receivedPacket);
    }

    /*
     * Server to client test
     */
    for (int i = 0; i < numMsg; i++) {
        packet.x_ = i;
        packet.y_ = i + 69;
        ASSERT_TRUE(server_->send(header, packet.serialize()));
        ASSERT_TRUE(client_->receive(&header, &buff));
        ASSERT_TRUE(receivedPacket.deserialize(buff));
        ASSERT_EQ(packet, receivedPacket);
    }
    ASSERT_TRUE(client_->disconnect());
    ASSERT_FALSE(client_->isConnected());
    ASSERT_FALSE(server_->receive(&header, &buff));
    ASSERT_FALSE(server_->isConnected());
    // one last time :P
    serverAccepted = std::async(std::launch::async,
        [this]() {
            return server_->acceptConnection();
        });
    clientConnected = std::async(std::launch::async,
        [this]() {
            return client_->connect();
        });
    ASSERT_TRUE(serverAccepted.get());
    ASSERT_TRUE(clientConnected.get());
    ASSERT_TRUE(server_->isConnected());
    ASSERT_TRUE(client_->isConnected());
    ASSERT_TRUE(server_->send(header, packet.serialize()));
    ASSERT_TRUE(client_->receive(&header, &buff));
    ASSERT_TRUE(receivedPacket.deserialize(buff));
    ASSERT_EQ(packet, receivedPacket);
    ASSERT_TRUE(server_->disconnect());
    ASSERT_FALSE(client_->receive(&header, &buff));
    ASSERT_FALSE(client_->isConnected());
}
