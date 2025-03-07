
/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO
 *
 *  ==================================================================================================
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <memory>

#include "IPC/MMAP.hpp"
#include "CommUtility/CommunicationPacket.hpp"

MMAP::~MMAP() {
    logger_->debug("DTor");
    // TODO(glunghi): should MMAP close itself, or better to leave this decision to the user?
}

bool MMAP::open() {
    if (fd_ > 0) {
        logger_->warn("MMAP was open before");
        return false;
    }
    if (isReader_) {
        if (!attemptMmapReader()) {
            logger_->error("attemptMmapReader: {}", std::strerror(errno));
            return false;
        }
    } else {
        fd_ = ::open(ipcFilename_.c_str(), O_RDWR | O_CREAT | O_TRUNC | O_NONBLOCK, 0666);
        if (ftruncate(fd_, size_ + sizeof(mmap_mutex_t)) == -1) {
            logger_->error("MMAP ftruncate: {}", std::strerror(errno));
            return false;
        }
        completeMmap_ = mmap(NULL, size_ + sizeof(mmap_mutex_t),
            PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        mmapMutex_ = reinterpret_cast<mmap_mutex_t*>(completeMmap_);
        mmapFile_ = reinterpret_cast<uint8_t*>(completeMmap_) + sizeof(mmap_mutex_t);
        std::memset(mmapFile_, 0, size_);
        pthread_mutexattr_t mutex_attr;
        pthread_mutexattr_init(&mutex_attr);
        pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&mmapMutex_->ipc_mutex, &mutex_attr);
        pthread_condattr_t cond_attr;
        pthread_condattr_init(&cond_attr);
        pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
        pthread_cond_init(&mmapMutex_->ipc_condvar, &cond_attr);
    }
    return true;
}

bool MMAP::close() {
    if (fd_ == 0) {
        logger_->warn("MMAP not open before");\
        return false;
    }
    if (::close(fd_) == -1) {
        logger_->error("close failed: {}", std::strerror(errno));
        fd_ = 0;
        return false;
    }
    fd_ = 0;
    if (!isReader_) {
        if (munmap(completeMmap_, size_) == -1) {
            logger_->error("Impossible to remove the MMAP: {}", std::strerror(errno));
            return false;
        }
    }
    return true;
}

bool MMAP::write(const std::string& bytes, const Packets::PacketHeader& header) {
    if (fd_ <= 0) {
        logger_->warn("MMAP not open");
        return false;
    }
    if (isReader_) {
        logger_->warn("This is reader, cannot invoke write");
        return false;
    }
    /* For the purpose of timestamp assignment we copy the header to apply
     * the timestamp, but we still keep const ref interface for the other ipc
     * classes that do not wish to modify the header
     */
    Packets::PacketHeader copyOfAHeader(header);
    if (copyOfAHeader.length != bytes.size()) {
        logger_->warn("header.length != bytes.size()");
        return false;
    }
    if (copyOfAHeader.timestamp == 0) {
        struct timeval start;
        gettimeofday(&start, NULL);
        copyOfAHeader.timestamp = ((start.tv_sec) * 1000 + start.tv_usec/1000.0) + 0.5;
    }

    pthread_mutex_lock(&mmapMutex_->ipc_mutex);
    memcpy(mmapFile_, copyOfAHeader.serialize().c_str(), Packets::PacketHeader::size);
    memcpy(reinterpret_cast<uint8_t*>(mmapFile_) + Packets::PacketHeader::size, bytes.c_str(),
        copyOfAHeader.length);
    pthread_cond_signal(&mmapMutex_->ipc_condvar);
    pthread_mutex_unlock(&mmapMutex_->ipc_mutex);
    return true;
}

bool MMAP::read(std::string& bytes, Packets::PacketHeader& header) {
    if (fd_ <= 0) {
        logger_->warn("MMAP not open");
        return false;
    }
    if (!isReader_) {
        logger_->warn("This is writer, cannot invoke read");
        return false;
    }
    if (!checkReaderReadiness() && !attemptMmapReader()) {
        return false;
    }

    if (isBlockingReader_) {
        return blockingRead(bytes, header);
    } else {
        return nonBlockingRead(bytes, header);
    }
}

std::shared_ptr<MMAP> MMAP::CreateWriterPtr(const std::string& filename, int size) {
    return std::shared_ptr<MMAP>(new MMAP(filename, size, false));
}

std::shared_ptr<MMAP> MMAP::CreateReaderPtr(const std::string& filename, int size) {
    return std::shared_ptr<MMAP>(new MMAP(filename, size, true));
}

MMAP::MMAP(const std::string& filename, int size, bool reader):
    logger_("MMAP"),
    ipcFilename_(filename),
    size_(size),
    isReader_(reader),
    isBlockingReader_(false),
    fd_(0),
    mmapMutex_(nullptr),
    mmapFile_(nullptr),
    completeMmap_(nullptr),
    latestTimestampRead_(0) {
    size_ = size_ + sizeof(Packets::PacketHeader);

    std::memset(&blockReadTimeoutTv_, 0, sizeof(blockReadTimeoutTv_));
}

bool MMAP::checkReaderReadiness() {
    if (fd_ < 0) {
        return false;
    }
    if (mmapFile_ == reinterpret_cast<void*>(-1)) {
        return false;
    }
    return true;
}

bool MMAP::attemptMmapReader() {
    fd_ = ::open(ipcFilename_.c_str(), O_RDWR, S_IRUSR);
    if (fd_ == -1) {
        return false;
    }
    completeMmap_ = mmap(NULL, size_ + sizeof(mmap_mutex_t),
        PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    mmapMutex_ = reinterpret_cast<mmap_mutex_t*>(completeMmap_);
    mmapFile_ = reinterpret_cast<uint8_t*>(completeMmap_) + sizeof(mmap_mutex_t);
    if (completeMmap_ == reinterpret_cast<void*>(-1)) {
        return false;
    }
    return true;
}

bool MMAP::blockingRead(std::string& bytes, Packets::PacketHeader& header) {
    if (!nonBlockingRead(bytes, header)) {
        return false;
    }

    if (header.timestamp != latestTimestampRead_) {
        latestTimestampRead_ = header.timestamp;
        return true;
    }

    pthread_mutex_lock(&mmapMutex_->ipc_mutex);
    do {
        if (blockReadTimeout_.count() == 0) {
            pthread_cond_wait(&mmapMutex_->ipc_condvar, &mmapMutex_->ipc_mutex);
        } else {
            struct timeval now;
            struct timespec timeToWait;
            gettimeofday(&now, NULL);
            timeToWait.tv_sec = now.tv_sec + blockReadTimeoutTv_.tv_sec;
            timeToWait.tv_nsec = now.tv_usec*1000UL + blockReadTimeoutTv_.tv_nsec;

            int retval = pthread_cond_timedwait(&mmapMutex_->ipc_condvar,
                &mmapMutex_->ipc_mutex, &timeToWait);

            if (retval == ETIMEDOUT) {
                logger_->warn("Read timeout");
                pthread_mutex_unlock(&mmapMutex_->ipc_mutex);
                return false;
            }
        }
        char header_buf[Packets::PacketHeader::size];
        memcpy(header_buf, mmapFile_, Packets::PacketHeader::size);
        std::string header_string(header_buf, Packets::PacketHeader::size);
        header.deserialize(header_string);
        char* buf;
        try {
            buf = new char[header.length];
        } catch (const std::bad_alloc&) {
            pthread_mutex_unlock(&mmapMutex_->ipc_mutex);
            return false;
        }
        memcpy(buf, reinterpret_cast<uint8_t*>(mmapFile_) + Packets::PacketHeader::size,
            header.length);
        bytes.assign(buf, header.length);
        delete[] buf;
    } while (header.timestamp == latestTimestampRead_);  // Loop to detect spurious wake-up

    latestTimestampRead_ = header.timestamp;
    pthread_mutex_unlock(&mmapMutex_->ipc_mutex);
    return true;
}

bool MMAP::nonBlockingRead(std::string& bytes, Packets::PacketHeader& header) {
    pthread_mutex_lock(&mmapMutex_->ipc_mutex);
    char header_buf[Packets::PacketHeader::size];
    memcpy(header_buf, mmapFile_, Packets::PacketHeader::size);
    std::string header_string(header_buf, Packets::PacketHeader::size);
    header.deserialize(header_string);
    char* buf;
    try {
        buf = new char[header.length];
    } catch (const std::bad_alloc&) {
        pthread_mutex_unlock(&mmapMutex_->ipc_mutex);
        return false;
    }
    memcpy(buf, reinterpret_cast<uint8_t*>(mmapFile_) + Packets::PacketHeader::size,
        header.length);
    bytes.assign(buf, header.length);
    delete[] buf;

    pthread_mutex_unlock(&mmapMutex_->ipc_mutex);
    return true;
}

bool MMAP::setBlockingRead(std::chrono::duration<float> timeout) {
    isBlockingReader_ = true;
    blockReadTimeout_ = timeout;
    blockReadTimeoutTv_.tv_sec =
        std::chrono::duration_cast<std::chrono::seconds>(blockReadTimeout_).count();
    blockReadTimeoutTv_.tv_nsec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(blockReadTimeout_).count() - blockReadTimeoutTv_.tv_sec*1e9; // NOLINT

    return true;
}

bool MMAP::setNonBlockingRead() {
    isBlockingReader_ = false;
    return true;
}
