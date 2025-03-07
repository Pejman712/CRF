/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>

#include "CANSocket/CANSocket.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/CANOpenContext.hpp"
#include "CANOpenDevices/CANOpenMotors/MaxonEPOS4.hpp"

using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::CANOpenContext;
using crf::devices::canopendevices::MaxonEPOS4;

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
    ctx->initialize();

    auto motor = std::make_shared<MaxonEPOS4>(0x02, canSocket);
    ctx->addDevice(motor);

    if (!motor->initialize()) {
        std::cout << "Could not initialize EPOS4" << std::endl;
        return -1;
    }

    std::cout << " In fault " << motor->inFault() << std::endl;

    if (!motor->faultReset()) {
        return -1;
    }

    std::cout << "Ready to switch on " << motor->isReadyToSwitchOn() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!motor->enableOperation()) {
        return -1;
    }

    int32_t destination = atoi(argv[3]);
    std::string command(argv[2]);

    for (int i=0; i < 2000; i++) {
        if ((i % 10) == 0)
            ctx->sendGuard();
        ctx->sendSync();

        std::cout << "Position: " << motor->getPosition().value() << " Velocity: "
            << motor->getVelocity().value() << " current " << motor->getCurrent().value()
            <<" Position reached " << motor->positionReached() << std::endl;
        std::cout << " " << motor->inQuickStop() << std::endl;
        if (command == "p")
            motor->setPosition(destination, 2000, false);
        else if (command == "v")
            motor->setVelocity(destination);
        else
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    motor->deinitialize();
    ctx->deinitialize();
    return 0;
}
