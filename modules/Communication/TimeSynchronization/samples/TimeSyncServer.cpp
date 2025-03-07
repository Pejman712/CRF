/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <chrono>
#include <memory>

#include "NetworkServer/TcpServer.hpp"
#include "IPC/NetworkIPC.hpp"
#include "TimeSynchronization/TimeSynchronizationServer.hpp"

using crf::communication::networkserver::TcpServer;
using crf::communication::ipc::NetworkIPC;

namespace {
    volatile std::sig_atomic_t gSignalStatus;
    void signal_handler(int signal) {
            gSignalStatus = signal;
    }
}   // unnamed namespace

int main() {
    std::shared_ptr<TcpServer> tcpServer = std::make_shared<TcpServer>(21301);
    auto ipc = std::make_shared<NetworkIPC>(tcpServer);
    crf::communication::timesynchronizationserver::TimeSynchronizationServer time(ipc);
    if (!ipc->open()) {
        std::cout << "Cannot open ipc" << std::endl;
        return -1;
    }
    if (!time.initialize()) {
        std::cout << "Could not initialie TimeSynchronizationServer" << std::endl;
        return -1;
    }
    std::signal(SIGTSTP, signal_handler);
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
