/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>

#include "IPC/MMAP.hpp"
#include "TeltonikaRUT/TeltonikaRUT.hpp"

using crf::devices::TeltonikaRUT;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Wrong number of parameters\n";
        std::cout << "[1] IP address of the modem\n";
        std::cout << "[2] MMAP filename\n";
        return -1;
    }
    std::shared_ptr<MMAP> outputIPC = MMAP::CreateWriterPtr(argv[2]);
    outputIPC->open();
    TeltonikaRUT modem(std::string(argv[1]), 22, "root", "EnSmmMro2018", outputIPC);
    while (!modem.initialize()) {
        std::cout << "Could not initialize the modem. Trying again in 5 seconds\n";
        sleep(5);
    }
    modem.joinThreads();
    modem.deinitialize();
    return 0;
}
