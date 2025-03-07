/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <string>
#include <csignal>
#include <stdio.h>
#include <memory>

#include "YoubotBase/YoubotBase.hpp"
#include "RobotBase/RobotBaseCommunicationPoint.hpp"
#include "IPC/FIFO.hpp"
#include "IPC/MMAP.hpp"

using crf::robots::YoubotBase;
using crf::robots::RobotBaseCommunicationPoint;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
     if (argc < 3) {
        std::cout << "Few arguments provided:" <<std::endl;
        std::cout << "  [1] Where to write (Es. '/tmp/fifo_youbot')" <<std::endl;
        std::cout << "  [2] Where to read (Es. '/tmp/mmap_youbot')" <<std::endl;
        return -1;
    }
    std::signal(SIGTSTP, signal_handler);
    printf("Starting YoubotBase\n");
    std::shared_ptr<FIFO> inputIPC = FIFO::CreateReaderPtr(argv[1]);
    std::shared_ptr<MMAP> outputIPC = MMAP::CreateWriterPtr(argv[2]);
    std::string configFileDir = __FILE__;
    configFileDir = configFileDir.substr(0, configFileDir.find(
        "samples/youbot.cpp"));
    configFileDir.append("config/youBotConfig.json");
    std::shared_ptr<YoubotBase> bot = std::make_shared<YoubotBase>(configFileDir);
    bot->initialize();
    RobotBaseCommunicationPoint communicationPoint(inputIPC, outputIPC, bot);
    communicationPoint.initialize();
    std::cout << "YoubotBase started correctly\n";
    while (gSignalStatus != SIGTSTP) {
        usleep(100000);
    }
    std::cout << "YoubotBase process clean up\n";
    communicationPoint.deinitialize();
    bot->deinitialize();
    return 0;
}
