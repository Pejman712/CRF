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
#include <memory>
#include <errno.h>  // maybe there is some better equivalent ???
#include <stdexcept>
#include <string>

#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/SocketInterface.hpp"
#include "NetworkServer/TcpServer.hpp"

namespace crf {
namespace communication {
namespace networkserver {

TcpServer::TcpServer(int port,
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface):
    logger_("TcpServer"),
    socketInterface_(socketInterface),
    serverSocket_(0),
    connectedSocket_(0) {
    logger_->debug("CTor");
    if (!socketInterface_) {
        logger_->debug("Provided socketInterface is empty, constructing a default one");
        socketInterface_.reset(new utility::commutility::SocketInterface);
    }
    // Opens the TCP socket
    if (((serverSocket_ = socketInterface_->socket(AF_INET, SOCK_STREAM, 0))) == -1) {
        logger_->error("Failed to create server socket: {}", strerror(errno));
        throw std::runtime_error("Failed to create server socket");
    }
    int flag = 1;
    socketInterface_->setsockopt(serverSocket_, IPPROTO_TCP, TCP_NODELAY,
        reinterpret_cast<char*>(&flag), sizeof(int));
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (socketInterface_->bind(serverSocket_, (struct sockaddr*) &server_addr,
        sizeof(struct sockaddr_in)) == -1) {
        logger_->error("Failed to bind the socket: {}", strerror(errno));
        throw std::runtime_error("Failed to bind the socket");
    }
    if (socketInterface_->listen(serverSocket_, 1) == -1) {
        logger_->error("Failed to listen: {}", strerror(errno));
        throw std::runtime_error("Failed to listen");
    }
}

TcpServer::~TcpServer() {
    logger_->debug("DTor");
    disconnect();
    logger_->debug("shutdown: {}",
        socketInterface_->shutdown(serverSocket_, SHUT_RDWR));
    logger_->debug("close: {}",
        socketInterface_->close(connectedSocket_));
}

bool TcpServer::acceptConnection(bool blocking) {
    logger_->debug("acceptConnection");
    if (!blocking) {
        logger_->warn("acceptConnection for non-blocking mode is not implemented");
        return false;
    }
    if (isConnected()) {
        logger_->warn("Already connected. Current impl doesn't allow multiple client connections");
        return false;
    }
    struct sockaddr_in clientAddr{};
    socklen_t clientAddrLen{};
    connectedSocket_ = socketInterface_->accept(serverSocket_,
        (struct sockaddr *)&clientAddr, &clientAddrLen);
    if (connectedSocket_ == -1) {
        logger_->warn("Failed to accept the connection {}", strerror(errno));
        connectedSocket_ = 0;
        return false;
    }
    logger_->info("Client({}) connected on socket: {}",
        clientAddr.sin_addr.s_addr, connectedSocket_);
    return true;
}

bool TcpServer::disconnect() {
    if (!isConnected()) {
        logger_->warn("Cannot disconnect, because not connected");
        return false;
    }
    if (socketInterface_->shutdown(connectedSocket_, SHUT_RDWR) == -1) {
        logger_->warn("Failed to shutdown the socket {}", strerror(errno));
        connectedSocket_ = 0;
        return false;
    }
    if (socketInterface_->close(connectedSocket_) == -1) {
        logger_->warn("Failed to close the socket {}", strerror(errno));
        connectedSocket_ = 0;
        return false;
    }
    connectedSocket_ = 0;
    return true;
}

bool TcpServer::send(const Packets::PacketHeader& header, const std::string& buffer) {
    logger_->debug("send");
    if (!isConnected()) {
        logger_->warn("Cannot send, because not connected");
        return false;
    }
    std::string bytes;
    bytes.append(header.serialize());
    bytes.append(buffer);
    int bytesSent = socketInterface_->send(connectedSocket_, bytes.c_str(),
        Packets::PacketHeader::size+header.length, 0);
    if (bytesSent < 0) {
        logger_->warn("Failed to send packet: {}", strerror(errno));
        return false;
    }
    return true;
}

bool TcpServer::receive(Packets::PacketHeader* header, std::string* buffer) {
    logger_->debug("receive");
    if (!isConnected()) {
        logger_->warn("Cannot receive, because not connected");
        return false;
    }
    int bytesReceived = 0;
    std::unique_ptr<char[]> buff(new char[Packets::PacketHeader::size]);
    bytesReceived = socketInterface_->recv(connectedSocket_,
        buff.get(), Packets::PacketHeader::size, 0);
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
    bytesReceived = socketInterface_->recv(connectedSocket_, buff.get(), header->length, 0);
    logger_->debug("Received {} bytes", bytesReceived);
    if (bytesReceived < 0) {
        logger_->warn("Failed to receive packet: {}", strerror(errno));
        logger_->info("Connection probably broken, going to disconnect");
        disconnect();
        return false;
    }
    buffer->resize(header->length);
    buffer->assign(buff.get(), header->length);
    return true;
}

bool TcpServer::isConnected() const {
    int error = 0;
    socklen_t len = sizeof (error);
    int retval = socketInterface_->getsockopt(
        connectedSocket_, SOL_SOCKET, SO_ERROR, &error, &len);
    logger_->debug("isConnected: {}", retval == 0);
    return (retval == 0);
}

}  // namespace networkserver
}  // namespace communication
}  // namespace crf
