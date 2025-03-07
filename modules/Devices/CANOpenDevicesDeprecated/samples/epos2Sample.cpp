/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include "CANSocket/CANSocket.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/CANOpenContext.hpp"
#include "CANOpenDevices/CANOpenMotors/MaxonEPOS2.hpp"

using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::CANOpenContext;
using crf::devices::canopendevices::MaxonEPOS2;

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cout << "Wrong number of parameters" << std::endl;
        std::cout << "[1] can port (e.g. can0)" << std::endl;
        std::cout << "[2] p for position control v for velocity control" << std::endl;
        std::cout << "[3] the value to send to the motor" << std::endl;
        return -1;
    }

    auto canSocket = std::make_shared<crf::communication::cansocket::CANSocket>(argv[1]);
    auto ctx = std::make_shared<CANOpenContext>(canSocket);
    ctx->setGuardFrequency(std::chrono::milliseconds(150));
    ctx->setSyncFrequency(std::chrono::milliseconds(2));

    ctx->initialize();

    auto motor = std::make_shared<MaxonEPOS2>(0x07, canSocket);
    ctx->addDevice(motor);

    if (!motor->initialize()) {
        std::cout << "Could not initialize EPOS2" << std::endl;
        return -1;
    }

    std::cout << " In fault "<< motor->inFault() << std::endl;

    if (!motor->faultReset()) {
        return -1;
    }

    std::cout << "Is ready to switch on " << motor->isReadyToSwitchOn() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!motor->enableOperation()) {
        return -1;
    }

    std::cout << "Is enabled " << motor->isEnabled() << std::endl;

    int32_t destination = atoi(argv[3]);
    std::string command(argv[2]);
    for (int i=0; i < 2000; i++) {
        std::cout << "Position: " << motor->getPosition().value() << " Velocity: "
            << motor->getVelocity().value() << " current " << motor->getCurrent().value()
            <<" Position reached " << motor->positionReached() << std::endl;
        if (command == "p") {
            motor->setPosition(destination, 2000, false);
        } else if (command == "v") {
            motor->setVelocity(destination);
        } else if (command == "c") {
            int16_t current = static_cast<int16_t>(destination);
            motor->setCurrent(current);
        } else {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    motor->deinitialize();
    ctx->deinitialize();
    return 0;
}
