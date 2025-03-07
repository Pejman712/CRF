/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <fstream>

#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"
#include "EtherCATDrivers/BasicEtherCATDriver/BasicEtherCATDriver.hpp"

using crf::devices::ethercatdrivers::EtherCATMaster;
using crf::devices::ethercatdrivers::BasicEtherCATDriver;

int main() {
    std::string ifname = "enx00e04c68027c";
    int nSlaves = 1;

    std::shared_ptr<EtherCATMaster> master = std::make_shared<EtherCATMaster>(
        ifname, nSlaves, std::chrono::milliseconds(1), 1024);

    if (!master->initialize()) {
        std::puts("Master initialization failed");
        return -1;
    }

    BasicEtherCATDriver driver(master, 1);

    if (!driver.initialize()) {
        std::puts("Driver initialization failed");
        return -1;
    }

    std::puts("Initialization correct");
    return 0;
}
