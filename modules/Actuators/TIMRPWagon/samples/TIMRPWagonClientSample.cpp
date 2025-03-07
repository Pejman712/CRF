/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <chrono>

#include "TIMRPWagon/TIMRPWagonClient/TIMRPWagonClient.hpp"
#include "Sockets/TCP/TCPSocket.hpp"

int main(int argc, char *argv[]) {
    std::shared_ptr<crf::actuators::timrpwagon::TIMRPWagonClient> client;

    std::shared_ptr<crf::communication::sockets::TCPSocket> code =
        std::make_shared<crf::communication::sockets::TCPSocket>("localhost", 3002);

    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket(
        new crf::communication::datapacketsocket::PacketSocket(code));

    std::chrono::milliseconds server_reply_timeout(10000);
    int priority_client = 1;
    float streamer_frequency = 0;
    client.reset(new crf::actuators::timrpwagon::TIMRPWagonClient(
        socket,
        server_reply_timeout,
        streamer_frequency,
        priority_client));

    if (!client->initialize()) {
        std::cout << "Could not initialize the client" << std::endl;
        return -1;
    }
    std::cout << "Client started correctly\n";

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "RP Arm position = " << static_cast<int>(client->getRPArmPosition()) << std::endl;
    std::cout << "RP Arm in error = " << client->isRPArmInError().value() << std::endl;

    // client->retractRPArm();
    // client->deployRPArm();
    // client->moveRPArmUp();
    // client->moveRPArmDown();
    // client->stopRPArm();
    // client->lockRPArm();
    // client->unlockRPArm();
    // client->resetRPArmDriver();
    client->acknowledgeErrors();

    return 0;
}
