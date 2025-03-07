/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thomas Breant CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#include <iostream>
#include <memory>
#include <string>

#include "CANSocket/CANSocket.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/CANOpenContext.hpp"
#include "CANOpenDevices/CANOpenMotors/ERB.hpp"
#include "boost/optional/optional_io.hpp"

using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::CANOpenContext;
using crf::devices::canopendevices::ERB;
using crf::communication::cansocket::CANSocket;

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cout << "Wrong number of parameters" << std::endl;
        std::cout << "[1] can port (e.g. can0)" << std::endl;
        std::cout << "[2] motor ID (e.g. 4)" << std::endl;
        std::cout << "[3] target position (e.g. 90000 mdeg)" << std::endl;
        std::cout << "[4] Mode (e.g. p for position and v for velocity)" << std::endl;
        return -1;
    }

    auto canSocket = std::make_shared<CANSocket>(argv[1]);
    auto ctx = std::make_shared<CANOpenContext>(canSocket);
    ctx->initialize();

    int id = std::stoi(argv[2]);

    auto motor = std::make_shared<ERB>(
        id, canSocket, crf::devices::canopendevices::ModesOfOperation::ProfilePositionMode);
    ctx->addDevice(motor);

    if (!motor->initialize()) {
        std::cout << "Could not initialize ERB motor" << std::endl;
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!motor->enableOperation()) {
        std::cout << "Could not enable the operation" << std::endl;
        return -1;
    }

    int32_t destination = atoi(argv[3]);
    std::string command(argv[4]);
    int i = 0;
    if (command == "p") {
        motor->setPosition(destination, 2952000, false);
    } else if (command == "v") {
        motor->setVelocity(destination);
    } else {
        return -1;
    }
    while (!motor->positionReached()) {
        if ((i % 10) == 0)
            ctx->sendGuard();
        ctx->sendSync();
        if ((command == "v") && i == 100) {
            motor->setVelocity(0);
            break;
        }
        std::cout << "Position : " << motor->getPosition().value() << " Velocity: "
            << motor->getVelocity().value() << " current " << motor->getCurrent().value()
            << " Torque : " << motor->getTorque().value() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        i++;
    }

    return 0;
}
