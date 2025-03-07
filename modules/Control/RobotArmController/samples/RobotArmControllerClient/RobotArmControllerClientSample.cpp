/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <nlohmann/json.hpp>
#include <memory>
#include <vector>
#include <iostream>
#include <chrono>

#include "RobotArmController/RobotArmControllerClient/RobotArmControllerClient.hpp"
#include "Sockets/TCP/TCPSocket.hpp"

int main(int argc, char *argv[]) {
    std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerClient> client;

    std::shared_ptr<crf::communication::sockets::TCPSocket> code =
        std::make_shared<crf::communication::sockets::TCPSocket>("localhost", 10000);

    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket(
        new crf::communication::datapacketsocket::PacketSocket(code));

    std::chrono::milliseconds server_reply_timeout(10000);
    int priority_client = 1;
    float streamer_frequency = 0;
    client.reset(new crf::control::robotarmcontroller::RobotArmControllerClient(
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

    std::cout << "Pos Joints: " << client->getJointPositions() << std::endl;
    std::cout << "Pos Task: " << client->getTaskPose() << std::endl;

    return 0;
}
