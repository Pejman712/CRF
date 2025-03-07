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
#include "CANOpenDevices/CANOpenIOs/CANOpenIOModule.hpp"

using crf::devices::canopendevices::ObjectDictionary;
using crf::devices::canopendevices::CANOpenContext;
using crf::devices::canopendevices::CANOpenIOModule;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Wrong number of parameters" << std::endl;
        std::cout << "[1] can port (e.g. can0)" << std::endl;
        return -1;
    }

    auto canSocket = std::make_shared<crf::communication::cansocket::CANSocket>(argv[1]);
    auto ctx = std::make_shared<CANOpenContext>(canSocket);
    ctx->setGuardFrequency(std::chrono::milliseconds(150));

    ctx->initialize();

    auto device = std::make_shared<CANOpenIOModule>(0x01, canSocket);
    ctx->addDevice(device);

    if (!device->initialize()) {
        std::cout << "Could not initialize CANOpenIOModule" << std::endl;
        return -1;
    }

    if (!device->setDigitalOutputState(1, true)) {
        std::cout << "Failed to set digital output state\n" << std::endl;
        return -1;
    }

    device->deinitialize();
    ctx->deinitialize();
    return 0;
}
