
/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO
 *
 *  ==================================================================================================
 */

#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>

#include <cerrno>
#include <memory>
#include <string>
#include <iostream>

#include "IPC/FIFO.hpp"
#include "CommUtility/CommunicationPacket.hpp"

FIFO::~FIFO() {
    logger_->debug("DTor");
}

bool FIFO::open() {
    if (isReader_) {
        struct stat stats;
        if (stat(ipcFilename_.c_str(), &stats) < 0) {
            if ( errno != ENOENT ) {
                // ENOENT is OK for us
                logger_->warn("stat failed: {}", std::strerror(errno));
                return false;
            }
        } else {
            if (unlink(ipcFilename_.c_str()) < 0) {
                logger_->warn("unlink failed: {}", std::strerror(errno));
                return false;
            }
        }
        mkfifo(ipcFilename_.c_str(), 0666);
        if (isNonBlock_) {
            fd_ = ::open(ipcFilename_.c_str(), O_RDWR | O_CREAT | O_NONBLOCK, 0666);
        } else {
            fd_ = ::open(ipcFilename_.c_str(), O_RDWR | O_CREAT, 0666);
        }
    }
    /*
     * TODO: Giacomo for review, what is the deal with thjat support FIFO?
     */
    if (!isReader_ && hasSupport_) {
        std::string fifo_support_filename = ipcFilename_ + "_support";
        logger_->info("I create a support FIFO at {}", fifo_support_filename);
        struct stat stats;
        if (stat(fifo_support_filename.c_str(), &stats) < 0) {
            // ENOENT is ok, since we intend to delete the file anyways
            if ( errno != ENOENT ) {
                logger_->warn("stat failed: {}", std::strerror(errno));
                return false;
            }
        } else {  // stat succeeded, so the file exists
            // attempt to delete the file
            if (unlink(fifo_support_filename.c_str() ) < 0) {
                // the most likely error is EBUSY, indicating that some other process is using the file NOLINT
                logger_->warn("unlink failed: {}", std::strerror(errno));
                return false;
            }
        }
        int ret_val = mkfifo(fifo_support_filename.c_str(), 0666);
        if (ret_val < 0) {
            perror("mkfifo() support");
        }
        fifoSupportFd_ = ::open(fifo_support_filename.c_str(),
            O_RDWR | O_CREAT | O_NONBLOCK, 0666);
    }
    return true;
}

bool FIFO::close() {
    if (::close(fd_) == -1) {
        logger_->error("close failed: {}", std::strerror(errno));
        return false;
    }
    return true;
}

bool FIFO::write(const std::string& bytes, const Packets::PacketHeader& header) {
    if (isReader_) {
        logger_->warn("This is reader, cannot invoke write");
        return false;
    }
    if (header.length != bytes.size()) {
        logger_->warn("header.length != bytes.size()");
        return false;
    }
    if (fifoSupportFd_ > 0) {
        read_service_msg();
    }
    fd_ = ::open(ipcFilename_.c_str(),  O_WRONLY | O_NONBLOCK, 0666);
    if (fd_ < 0) {
        logger_->warn("Failed to open for writing: {}", std::strerror(errno));
        return false;
    }
    /* For the purpose of timestamp assignment we copy the header to apply
     * the timestamp, but we still keep const ref interface for the other ipc
     * classes that do not wish to modify the header
     */
    Packets::PacketHeader copyOfAHeader(header);
    if (copyOfAHeader.timestamp == 0) {
        struct timeval start;
        gettimeofday(&start, NULL);
        copyOfAHeader.timestamp = ((start.tv_sec) * 1000 + start.tv_usec/1000.0) + 0.5;
    }
    int ret_val = ::write(fd_, copyOfAHeader.serialize().c_str(), Packets::PacketHeader::size);
    if (ret_val <= 0) {
        logger_->warn("Failed to write header: {}", std::strerror(errno));
        ::close(fd_);
        return false;
    }
    ret_val = ::write(fd_, bytes.c_str(), copyOfAHeader.length);
    if (ret_val <= 0) {
        logger_->warn("Failed to write packet: {}", std::strerror(errno));
        ::close(fd_);
        return false;
    }
    ::close(fd_);
    return true;
}

bool FIFO::read(std::string& bytes, Packets::PacketHeader& header) {
    if (!isReader_) {
        logger_->warn("This is writer, cannot invoke read");
        return false;
    }
    if (isNonBlock_) {
        return nonBlockingRead(bytes, header);
    } else {
        return blockingRead(bytes, header);
    }
}

std::shared_ptr<FIFO> FIFO::CreateReaderPtr(const std::string& filename) {
    std::shared_ptr<FIFO> fifo(new FIFO(filename, 1, false, false));
    return fifo;
}

std::shared_ptr<FIFO> FIFO::CreateReaderPtrNonBlock(const std::string& filename,
    std::chrono::milliseconds timeout) {
    std::shared_ptr<FIFO> fifo(new FIFO(filename, 1, true, false));
    fifo->setTimeout(timeout);
    return fifo;
}

std::shared_ptr<FIFO> FIFO::CreateWriterPtr(const std::string& filename, bool has_support) {
    std::shared_ptr<FIFO> fifo(new FIFO(filename, 0, false, has_support));
    return fifo;
}

FIFO::FIFO(const std::string& filename, bool reader, bool nonBlock,
    bool hasSupport):
    logger_("FIFO"),
    ipcFilename_(filename),
    isReader_(reader),
    isNonBlock_(nonBlock),
    hasSupport_(hasSupport),
    fd_(0),
    fifoSupportFd_(0),
    timeout_() {
    // TODO(glunghi): why here was if (filename == "/tmp/null") ???
    logger_->debug("CTor");
}

bool FIFO::nonBlockingRead(std::string& bytes, Packets::PacketHeader& header) {
    auto start = std::chrono::high_resolution_clock::now();
    char header_buf[Packets::PacketHeader::size];
    int read = 0;
    do {
        std::chrono::milliseconds elapsed
            = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start);
        if ((read == 0) && (elapsed.count() >= timeout_.count())) {
            return false;
        }
        int ret_val = ::read(fd_, header_buf+read, Packets::PacketHeader::size-read);
        if ((errno != EAGAIN) && (ret_val == -1) && (read == 0)) {
            logger_->warn("nonBlockingRead fail: {}", std::strerror(errno));
            return false;
        } else {
            if (ret_val >= 0)
                read += ret_val;
        }
    } while (read < Packets::PacketHeader::size);
    std::string header_string(header_buf, Packets::PacketHeader::size);
    header.deserialize(header_string);
    char* buf = new char[header.length];
    read = 0;
    do {
        int ret_val = ::read(fd_, buf+read, header.length-read);
        if ((errno != EAGAIN) && (ret_val == -1) && (read == 0)) {
            logger_->warn("nonBlockingRead fail: {}", std::strerror(errno));
            delete[] buf;
            return false;
        } else {
            if (ret_val >= 0)
                read += ret_val;
        }
    } while (read < header.length);
    bytes.assign(buf, header.length);
    delete[] buf;
    return true;
}

bool FIFO::blockingRead(std::string& bytes, Packets::PacketHeader& header) {
    char header_buf[Packets::PacketHeader::size];
    int ret_val = ::read(fd_, header_buf, Packets::PacketHeader::size);
    if (ret_val < 0) {
        logger_->warn("blockingRead fail: {}", std::strerror(errno));
        return false;
    }
    std::string header_string(header_buf, Packets::PacketHeader::size);
    header.deserialize(header_string);
    char* buf = new char[header.length];
    ret_val = ::read(fd_, buf, header.length);
    if (ret_val < 0) {
        logger_->warn("blockingRead fail: {}", std::strerror(errno));
        delete[] buf;
        return false;
    }
    bytes.assign(buf, header.length);
    delete[] buf;
    return true;
}

bool FIFO::read_service_msg() {
    /*
     * TODO: all the code in this file was commented-out ...
     */
    return false;
}

void FIFO::setTimeout(std::chrono::milliseconds timeout) {
    if (isNonBlock_) {
        timeout_ = timeout;
    }
}
