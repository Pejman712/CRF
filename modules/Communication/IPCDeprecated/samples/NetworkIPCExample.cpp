/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Contributors: Giacomo Lunghi CERN EN/SMM/MRO,
 *  ==================================================================================================
 */

#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "NetworkServer/TcpServer.hpp"
#include "IPC/NetworkIPC.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

using crf::communication::networkserver::TcpServer;
using crf::communication::ipc::NetworkIPC;

std::atomic<bool> stop(false);

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

void sender(std::shared_ptr<IPC> ipc) {
    while (!stop) {
        communication::datapackets::JSONPacket json;
        json.data_["temp"] = "temp";

        if (!ipc->write(json.serialize(), json.getHeader()))
            std::cout << "Spento" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void receiver(std::shared_ptr<IPC> ipc) {
    std::string buffer;
    Packets::PacketHeader header;
    while (!stop) {
        if (ipc->read(buffer, header))
            std::cout << header.length << std::endl;
    }
}

int main(int argc, char* argv[]) {
    auto tcpserver = std::make_shared<TcpServer>(3001);
    auto ipc = std::make_shared<NetworkIPC>(tcpserver);

    std::signal(SIGTSTP, signal_handler);

    ipc->open();

    auto senderThread = std::thread(sender, ipc);
    auto receiverThread = std::thread(receiver, ipc);

    while (gSignalStatus != SIGTSTP) {
        usleep(100000);
    }

    stop = true;
    senderThread.join();
    receiverThread.join();

    ipc->close();
    return 0;
}
