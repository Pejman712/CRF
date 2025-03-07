#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO
 *
 *  ==================================================================================================
 */

#include <chrono>
#include <memory>
#include <string>

#include "CommUtility/CommunicationPacket.hpp"
#include "IPC/IPC.hpp"
#include "EventLogger/EventLogger.hpp"

#ifndef FIFO_NONBLOCK
# define FIFO_NONBLOCK 001
#endif

class FIFO : public IPC {
 public:
    FIFO() = delete;
    FIFO(const FIFO& other) = delete;
    FIFO(FIFO&& other) = delete;
    ~FIFO() override;
    bool open() override;
    bool close() override;
    bool write(const std::string& bytes, const Packets::PacketHeader& header) override;
    bool read(std::string& bytes, Packets::PacketHeader& header) override;
    static std::shared_ptr<FIFO> CreateReaderPtr(const std::string& filename);
    static std::shared_ptr<FIFO> CreateReaderPtrNonBlock(const std::string& filename,
        std::chrono::milliseconds timeout);
    static std::shared_ptr<FIFO> CreateWriterPtr(const std::string& filename,
        bool hasSupport = true);

 private:
    FIFO(const std::string& filename, bool reader, bool nonBlock = false,
        bool hasSupport = false);
    bool nonBlockingRead(std::string& bytes, Packets::PacketHeader& header);  // NOLINT
    bool blockingRead(std::string& bytes, Packets::PacketHeader& header);  // NOLINT
    bool read_service_msg();
    void setTimeout(std::chrono::milliseconds timeout);
    crf::utility::logger::EventLogger logger_;
    std::string ipcFilename_;
    bool isReader_;
    bool isNonBlock_;
    bool hasSupport_;
    int fd_;
    int fifoSupportFd_;
    std::chrono::milliseconds timeout_;
};
