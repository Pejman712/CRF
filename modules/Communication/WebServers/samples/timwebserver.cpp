/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>
#include <signal.h>

#include "TIM/TIMS300.hpp"
#include "WebServers/TIMWebServer.hpp"

using crf::robots::tim::TIMS300;
using crf::communication::webservers::TIMWebServer;

namespace {
    static volatile int keepRunning = 1;
    void intHandler(int dummy) {
        keepRunning = 0;
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Missing configuration fiile for TIM webserver" << std::endl;
        return -1;
    }

    signal(SIGINT, intHandler);

    TIMWebServer server(argv[1]);
    server.initialize();

    std::cout << "Press CTRL+C to kill" << std::endl;
    while (keepRunning) { sleep(1); }

    std::cout << "Received CTRL+C. Going to deinitialized" << std::endl;
    server.deinitialize();
    return 0;
}
