#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MNRO
 *
 *  ==================================================================================================
 */
#include <memory>
#include <thread>
#include <string>
#include "CommUtility/CommunicationPacket.hpp"

/**
 * Interface class for generic Inter Process Communication
 *
 * Every class implementing some kind of inter-process communication
 * (excluding network interfaces and some specific hardware APIs)
 * should implement this interface.
 */
class IPC {
 public:
    virtual ~IPC() = default;
    /**
     * Open should always be called before write and read operations
     * Returns:
     *      true upon success
     *      false upon failure (e.g. wrong comm point filename, already open)
     */
    virtual bool open() = 0;
    /**
     * Implementations are encouraged to always call close in the destructor
     * Returns:
     *      true upon success
     *      false upon failure (e.g. already closed)
     */
    virtual bool close() = 0;
    /**
     * Uses the open communication channel to send the PacketHeader followed
     * by the serialized Packet;
     * Typical usage: ipc_->write(packet.serialize(), packet.getHeader());
     * Returns:
     *      true upon success
     *      false upon failure (e.g. ipc not open)
     */
    virtual bool write(const std::string& bytes, const Packets::PacketHeader& header) = 0;
    /**
     * Uses the open communication channel to receive the PacketHeader followed
     * by the serialized Packet;
     * Typical usage:
     *      std::string buff;
     *      Packets::PacketHeader header;
     *      Packets::SomePacket packet;
     *      ipc_->read(buff, header);
     *      if (!packet.deserialize()) { received some garbage }
     * Returns:
     *      true upon success
     *      false upon failure (e.g. ipc not open)
     *
     * Note (TODO):
     *      For better readability we should use the following function signature:
     *          bool read(std::string* bytes, Packets::PacketHeader* header)
     *      but this change requires a lot of modifications in most of the CRF
     *      code. We can postpone it for now.
     */
    virtual bool read(std::string& bytes, Packets::PacketHeader& header) = 0;  // NOLINT
};

namespace crf {
namespace communication {
namespace ipc {

struct IpcCommunicationPoint{
    std::shared_ptr<IPC> inputIPC_;
    std::shared_ptr<IPC> outputIPC_;
    std::thread receiverThread_;
    std::thread senderThread_;
    std::chrono::duration<float> publisherRate_;
    bool active_;
};

}  // namespace ipc
}  // namespace communication
}  // namespace crf


