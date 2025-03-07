/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <errno.h>
#include <stdexcept>
#include <string>
#include <memory>

#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/SocketInterface.hpp"
#include "NetworkClient/TcpClient.hpp"

namespace crf {
namespace communication {
namespace networkclient {

TcpClient::TcpClient(std::shared_ptr<utility::commutility::ISocketInterface> socketInterface):
    logger_("TcpClient"),
    socketInterface_(socketInterface),
    socket_(0),
    addrinfoPtr_(nullptr) {
    logger_->debug("CTor");
    if (!socketInterface_) {
        logger_->debug("Provided socketInterface is empty, constructing a default one");
        socketInterface_.reset(new utility::commutility::SocketInterface);
    }
}

TcpClient::TcpClient(const std::string& address, int port,
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface):
    logger_("TcpClient"),
    socketInterface_(socketInterface),
    socket_(0),
    addrinfoPtr_(nullptr) {
    logger_->debug("CTor");
    if (!socketInterface_) {
        logger_->debug("Provided socketInterface is empty, constructing a default one");
        socketInterface_.reset(new utility::commutility::SocketInterface);
    }
    struct addrinfo hints{};
    struct addrinfo* res = nullptr;
    int status;
    std::memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    status = socketInterface_->getaddrinfo(
        address.c_str(), std::to_string(port).c_str(), &hints, &res);
    if (status != 0) {
        logger_->error("Failed to getaddrinfo: {}", gai_strerror(status));
        throw std::runtime_error("Failed to getaddrinfo");
    }
    addrinfoPtr_ = res;
}

TcpClient::~TcpClient() {
    logger_->debug("DTor");
    if (isConnected()) {
        disconnect();
    }
    socketInterface_->freeaddrinfo(addrinfoPtr_);
}

bool TcpClient::connect(bool blocking) {
    logger_->debug("connect");
    if (!blocking) {
        logger_->warn("connect for non-blocking mode is not implemented");
        return false;
    }
    if (isConnected()) {
        logger_->warn("Already connected.");
        return false;
    }
    socket_ = socketInterface_->socket(addrinfoPtr_->ai_family, addrinfoPtr_->ai_socktype, 0);
    if (socket_ == -1) {
        logger_->error("Failed to create client socket: {}", strerror(errno));
        return false;
    }
    logger_->debug("Going to connect ... ");
    if (socketInterface_->connect(socket_, addrinfoPtr_->ai_addr,
        addrinfoPtr_->ai_addrlen)== -1) {
        logger_->warn("Failed to connect: {}", strerror(errno));
        return false;
    }
    logger_->debug("Connected");
    return true;
}

bool TcpClient::disconnect() {
    logger_->debug("disconnect");
    if (!isConnected()) {
        logger_->warn("Cannot disconnect, because not connected");\
        return false;
    }
    if (socketInterface_->shutdown(socket_, SHUT_RDWR) == -1) {
        logger_->warn("Failed to shutdown the socket {}", strerror(errno));
        socket_ = 0;
        return false;
    }
    if (socketInterface_->close(socket_) == -1) {
        logger_->warn("Failed to close the socket {}", strerror(errno));
        socket_ = 0;
        return false;
    }
    socket_ = 0;
    return true;
}

bool TcpClient::send(const Packets::PacketHeader& header, const std::string& buffer) {
    logger_->debug("send");
    if (!isConnected()) {
        logger_->warn("Cannot send, because not connected");\
        return false;
    }
    std::string bytes;
    bytes.append(header.serialize());
    bytes.append(buffer);
    int bytesSent = socketInterface_->send(socket_, bytes.c_str(),
        Packets::PacketHeader::size+header.length, 0);
    if (bytesSent < 0) {
        logger_->warn("Failed to send packet: {}", strerror(errno));
        return false;
    }
    return true;
}

bool TcpClient::receive(Packets::PacketHeader* header, std::string* buffer) {
    logger_->debug("receive");
    if (!isConnected()) {
        logger_->warn("Cannot receive, because not connected");\
        return false;
    }
    int bytesReceived = 0;
    std::unique_ptr<char[]> buff(new char[Packets::PacketHeader::size]);
    bytesReceived = socketInterface_->recv(socket_, buff.get(), Packets::PacketHeader::size, 0);
    logger_->debug("Received {} bytes", bytesReceived);
    if (bytesReceived < Packets::PacketHeader::size) {
        logger_->warn("Failed to receive header: {}", strerror(errno));
        logger_->info("Connection probably broken, going to disconnect");
        disconnect();
        return false;
    }
    if (!header->deserialize(std::string(buff.get(), Packets::PacketHeader::size))) {
        logger_->warn("Failed to deserialize the header, consider disconnecting");
        return false;
    }
    buff.reset(new char[header->length]);
    bytesReceived = socketInterface_->recv(socket_, buff.get(), header->length, 0);
    logger_->debug("Received {} bytes", bytesReceived);
    if (bytesReceived < 0) {
        logger_->warn("Failed to receive header: {}", strerror(errno));
        return false;
    }
    buffer->resize(header->length);
    buffer->assign(buff.get(), header->length);

    return true;
}

bool TcpClient::isConnected() const {
    int error = 0;
    socklen_t len = sizeof (error);
    int retval = socketInterface_->getsockopt(
        socket_, SOL_SOCKET, SO_ERROR, &error, &len);
    logger_->debug("isConnected: {}", retval == 0);
    return (retval == 0);
}

}  // namespace networkclient
}  // namespace communication
}  // namespace crf
