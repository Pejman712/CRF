#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>

#include "CommUtility/CommunicationPacket.hpp"
#include "IPC/IPC.hpp"
#include "EventLogger/EventLogger.hpp"

class MMAP : public IPC{
 public:
    MMAP() = delete;
    MMAP(const MMAP& other) = delete;
    MMAP(MMAP&& other) = delete;
    ~MMAP() override;

    bool open() override;
    bool close() override;
    bool write(const std::string& bytes, const Packets::PacketHeader& header) override;
    bool read(std::string& bytes, Packets::PacketHeader& header) override;
    static std::shared_ptr<MMAP> CreateWriterPtr(const std::string& filename, int size = 4096);
    static std::shared_ptr<MMAP> CreateReaderPtr(const std::string& filename, int size = 4096);   // NOLINT

    bool setBlockingRead(std::chrono::duration<float> timeout = std::chrono::seconds(0));
    bool setNonBlockingRead();

 private:
    MMAP(const std::string& filename, int size, bool reader);
    bool checkReaderReadiness();
    bool attemptMmapReader();
    crf::utility::logger::EventLogger logger_;
    std::string ipcFilename_;
    int size_;
    bool isReader_;
    int fd_;
    // let's allow C-style type name
    typedef struct {
        pthread_mutex_t ipc_mutex;
        pthread_cond_t ipc_condvar;
    } mmap_mutex_t;
    mmap_mutex_t* mmapMutex_;
    void* mmapFile_;
    void* completeMmap_;

    bool isBlockingReader_;
    std::chrono::duration<float> blockReadTimeout_;
    timespec blockReadTimeoutTv_;
    int64_t latestTimestampRead_;
    bool blockingRead(std::string& bytes, Packets::PacketHeader& header);  // NOLINT
    bool nonBlockingRead(std::string& bytes, Packets::PacketHeader& header);  // NOLINT
};
