/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdio>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "IPC/IPC.hpp"
#include "IPC/FIFO.hpp"  // TODO(pptaszni): no FIFO tests yet, someone write them please?
#include "IPC/MMAP.hpp"

#include "Mocks/Utility/DummyPacket.hpp"

class MmapShould: public ::testing::Test {
 protected:
    MmapShould(): logger_("MmapShould") {
        logger_->info("{0} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        filename_ = "mmap_testing";
        message_ = "Dummy msg";
        header_.length = message_.length();
    }
    void TearDown() override {
        std::remove(filename_.c_str());
        /*
         * Mmmm ... I don't know, should we expect IPC to remove mmap files?
         * Because it is not removing mmap files right now ...
         */
        // ASSERT_NE(0, std::remove(filename_.c_str()))
        //     << "seems like MMAP did not clean the files";
    }

    crf::utility::logger::EventLogger logger_;
    std::string filename_;
    std::string message_;
    Packets::PacketHeader header_;

    ~MmapShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
};

TEST_F(MmapShould, returnFalseIfOpenOrCloseTwice) {
    std::shared_ptr<IPC> writer = MMAP::CreateWriterPtr(filename_);
    std::shared_ptr<IPC> reader = MMAP::CreateReaderPtr(filename_);
    ASSERT_TRUE(writer->open());
    ASSERT_FALSE(writer->open());
    ASSERT_TRUE(reader->open());
    ASSERT_FALSE(reader->open());
    ASSERT_TRUE(writer->close());
    ASSERT_FALSE(writer->close());
    ASSERT_TRUE(reader->close());
    ASSERT_FALSE(reader->close());
}

TEST_F(MmapShould, returnFalseForReadWriteIfNotOpen) {
    std::shared_ptr<IPC> writer = MMAP::CreateWriterPtr(filename_);
    std::shared_ptr<IPC> reader = MMAP::CreateReaderPtr(filename_);
    ASSERT_FALSE(writer->write(message_, header_));
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    ASSERT_FALSE(reader->read(receivedMsg, receivedHeader));
}

TEST_F(MmapShould, returnTrueForWriteAndReadOperationsIfWriterWasOpenFirst) {
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    std::shared_ptr<IPC> writer = MMAP::CreateWriterPtr(filename_);
    std::shared_ptr<IPC> reader = MMAP::CreateReaderPtr(filename_);
    ASSERT_TRUE(writer->open());
    ASSERT_TRUE(reader->open());

    /* Will read some garbage (zeros - if writer constructor takes care of it),
     * but it is OK - operation should be considered
     * successful as long as communication point is correct.
     */
    ASSERT_TRUE(reader->read(receivedMsg, receivedHeader));
    ASSERT_TRUE(writer->write(message_, header_));
    ASSERT_TRUE(reader->read(receivedMsg, receivedHeader));
    ASSERT_EQ(message_, receivedMsg);
    ASSERT_EQ(header_.length, receivedHeader.length);
    ASSERT_TRUE(reader->close());
    ASSERT_TRUE(writer->close());
}

TEST_F(MmapShould, returnFalseIfReaderAttemptsToOpenBeforeWriterCreationAndTrueWhenWriterIsFinallyCreated) {  // NOLINT
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    std::shared_ptr<IPC> reader = MMAP::CreateReaderPtr(filename_);
    std::shared_ptr<IPC> writer = MMAP::CreateWriterPtr(filename_);
    ASSERT_FALSE(reader->open());
    ASSERT_TRUE(writer->open());
    ASSERT_TRUE(reader->open());
    ASSERT_TRUE(writer->write(message_, header_));
    ASSERT_TRUE(reader->read(receivedMsg, receivedHeader));
    ASSERT_EQ(message_, receivedMsg);
    ASSERT_EQ(header_.length, receivedHeader.length);
    ASSERT_TRUE(reader->close());
    ASSERT_TRUE(writer->close());
}

TEST_F(MmapShould, writeAndReadConsistentPacketsWithoutCollisions) {
    float expectedX = 69.69999;
    float xyDifference = 0.69;
    std::shared_ptr<IPC> writer = MMAP::CreateWriterPtr(filename_);
    std::shared_ptr<IPC> reader = MMAP::CreateReaderPtr(filename_);
    ASSERT_TRUE(writer->open());
    ASSERT_TRUE(reader->open());
    int numThreads = 100;
    int numIterations = 10000;
    std::vector<std::thread> writerThreads(numThreads);
    std::vector<std::thread> readerThreads(numThreads);
    Packets::DummyPacket packet{};
    packet.x_ = expectedX;
    packet.y_ = expectedX * xyDifference;
    auto writerRoutine = [writer, packet, xyDifference, numIterations]() {
        Packets::DummyPacket packetCopy(packet);
        for (int i = 0; i < numIterations; i++) {
            packetCopy.x_ += i;
            packetCopy.y_ = packetCopy.x_ * xyDifference;
            ASSERT_TRUE(writer->write(packetCopy.serialize(), packetCopy.getHeader()));
        }
    };
    auto readerRoutine = [this, reader, packet, xyDifference, numIterations]() {
        std::string buff;
        Packets::DummyPacket receivedPacket;
        Packets::PacketHeader receivedHeader;
        for (int i = 0; i < numIterations; i++) {
            ASSERT_TRUE(reader->read(buff, receivedHeader));
            ASSERT_TRUE(receivedPacket.deserialize(buff));
            ASSERT_EQ(receivedHeader.type, receivedPacket.getHeader().type);
            ASSERT_EQ(packet.y_, packet.x_ * xyDifference);
        }
    };
    for (auto& t : writerThreads) {
        t = std::thread(writerRoutine);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    for (auto& t : readerThreads) {
        t = std::thread(readerRoutine);
    }
    for (auto& t : writerThreads) {
        t.join();
    }
    for (auto& t : readerThreads) {
        t.join();
    }
}

TEST_F(MmapShould, blockingReadProperlyWorking) {
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    std::shared_ptr<IPC> writer = MMAP::CreateWriterPtr(filename_);
    std::shared_ptr<MMAP> reader = MMAP::CreateReaderPtr(filename_);
    reader->setBlockingRead(std::chrono::milliseconds(200));
    ASSERT_TRUE(writer->open());
    ASSERT_TRUE(reader->open());

    auto result = std::async(std::launch::async, [this, writer]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        writer->write(message_, header_);
        return true;
    });

    ASSERT_TRUE(reader->read(receivedMsg, receivedHeader));
    ASSERT_FALSE(reader->read(receivedMsg, receivedHeader));
    ASSERT_TRUE(result.get());

    result = std::async(std::launch::async, [this, writer]() {
        writer->write(message_, header_);
        return true;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(reader->read(receivedMsg, receivedHeader));
    ASSERT_FALSE(reader->read(receivedMsg, receivedHeader));
    ASSERT_TRUE(result.get());
}

/*
 * You can run those 2 tests from 2 different places and test MMAP
 * on the integration testing level.
 */

TEST_F(MmapShould, DISABLED_writePackets) {
    int iters = 100;
    std::shared_ptr<IPC> writer = MMAP::CreateWriterPtr(filename_);
    ASSERT_TRUE(writer->open());
    for (int i = 0; i < iters; i++) {
        ASSERT_TRUE(writer->write(message_, header_)) << "iter: " << i;
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
    }
    ASSERT_TRUE(writer->close());
}

TEST_F(MmapShould, DISABLED_readPackets) {
    int iters = 100;
    std::string receivedMsg;
    Packets::PacketHeader receivedHeader;
    std::shared_ptr<IPC> reader = MMAP::CreateReaderPtr(filename_);
    ASSERT_TRUE(reader->open());
    for (int i = 0; i < iters; i++) {
        ASSERT_TRUE(reader->read(receivedMsg, receivedHeader)) << "iter: " << i;
        ASSERT_EQ(message_, receivedMsg) << "iter: " << i;
        ASSERT_EQ(header_.length, receivedHeader.length) << "iter: " << i;
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
    }
    ASSERT_TRUE(reader->close());
}
