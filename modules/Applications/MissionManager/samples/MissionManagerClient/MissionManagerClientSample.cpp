/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@crf.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#include <string>
#include <nlohmann/json.hpp>
#include <memory>
#include <vector>
#include <iostream>
#include <csignal>
#include <chrono>

#include "MissionManager/MissionManagerClient/MissionManagerClient.hpp"
#include "Sockets/TCP/TCPSocket.hpp"

int main(int argc, char *argv[]) {
    std::shared_ptr<crf::applications::missionmanager::MissionManagerClient> client;

    std::shared_ptr<crf::communication::sockets::TCPSocket> code =
        std::make_shared<crf::communication::sockets::TCPSocket>("localhost", 3002);

    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket(
        new crf::communication::datapacketsocket::PacketSocket(code));

    client.reset(new crf::applications::missionmanager::MissionManagerClient(
        socket,
        std::chrono::milliseconds(1500)));

    if (!client->initialize()) {
        std::cout << "Could not initialize the client" << std::endl;
        return -1;
    }

    std::cout << "Client started correctly\n";

    std::this_thread::sleep_for(std::chrono::seconds(2));

    if (!client->start()) {
        std::puts("Could not start the client");
        return -1;
    }
    std::cout << "Launch start\n";
    return 0;
}
