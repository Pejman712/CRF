/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <memory>
#include <errno.h>  // maybe there is some better equivalent ???
#include <stdexcept>
#include <string>
#include <cstring>

#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/SocketInterface.hpp"
#include "NetworkServer/UdpServer.hpp"
#include "NetworkServer/UDPSocketPackets.hpp"

namespace crf {
namespace communication {
namespace networkserver {

UdpServer::UdpServer(int port,
    std::shared_ptr<utility::commutility::ISocketInterface> socketInterface):
    logger_("UdpServer"),
    socketInterface_(socketInterface),
    serverSocket_{},
    clientAddress_{},
    isConnected_{false} {
    logger_->debug("CTor");
    if (!socketInterface_) {
        logger_->debug("Provided socketInterface is empty, constructing a default one");
        socketInterface_.reset(new utility::commutility::SocketInterface);
    }
    // Opens the UDP socket
    if (((serverSocket_ = socketInterface_->socket(AF_INET, SOCK_DGRAM, 0))) == -1) {
        logger_->error("Failed to create server socket: {}", strerror(errno));
        throw std::runtime_error("Failed to create server socket");
    }
    // Enables the reuse of the port
    int flag = 1;
    socketInterface_->setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<char*>(&flag), sizeof(int));

    // Creates the socket address type
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    // htons(int) : converts int from little to big endian
    server_addr.sin_port = htons(port);
    // htonl(long) : converts long from little to big endian
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (socketInterface_->bind(serverSocket_, (struct sockaddr*) &server_addr,
        sizeof(struct sockaddr_in)) == -1) {
        logger_->error("Failed to bind the socket: {}", strerror(errno));
        throw std::runtime_error("Failed to bind the socket");
    }
}

UdpServer::~UdpServer() {
    logger_->debug("DTor");
    disconnect();
    // Shutdown closes the socket for all connected processes,
    // also lets you read out messages that are already in
    // It is not needed here strictly, but it makes things a bit safer
    logger_->debug("shutdown: {}",
        socketInterface_->shutdown(serverSocket_, SHUT_RDWR));
    // Destroys the outgoing port
    logger_->debug("close: {}",
        socketInterface_->close(serverSocket_));
}

bool UdpServer::acceptConnection(bool blocking) {
    logger_->debug("acceptConnection");
    if (!blocking) {
        logger_->warn("acceptConnection for non-blocking mode is not implemented");
        return false;
    }
    if (isConnected()) {
        logger_->warn("Already connected. Current impl doesn't allow multiple client connections");
        return false;
    }

    // read connection message
    while (!isConnected_) {
        Packets::PacketHeader header {};
        struct sockaddr_in sender {};
        socklen_t sendsize = sizeof(sender);
        int bytesReceived = 0;

        std::unique_ptr<char[]> buff(new char[Packets::PacketHeader::size]);
        bytesReceived = socketInterface_->recvfrom(serverSocket_, buff.get(),
            Packets::PacketHeader::size, MSG_WAITALL, reinterpret_cast<sockaddr *>(&sender),
            &sendsize);

        if ((bytesReceived < Packets::PacketHeader::size) ||
            !header.deserialize(std::string(buff.get(), Packets::PacketHeader::size)) ||
            (header.type != Packets::UDP_CONNECTION_PACKET_TYPE)) {
            logger_->warn("Failed to receive packet header during the Accept Connection");
            continue;
        }

        // check if accept connection
        Packets::UDPConnectionPacket connection_packet {};
        buff.reset(new char[connection_packet.size]);
        bytesReceived = socketInterface_->recvfrom(serverSocket_, buff.get(),
            connection_packet.size, MSG_WAITALL,
            reinterpret_cast<sockaddr *>(&sender), &sendsize);
        std::string str(buff.get(), connection_packet.size);
        connection_packet.deserialize(str);

        // Setting the client address into local field
        if (connection_packet.action ==
            Packets::UDPConnectionPacket::UDP_CONNECTION_STATE::CONNECT) {
            isConnected_ = true;
            std::memset(&clientAddress_, 0, sizeof(struct sockaddr_in));
            clientAddress_.sin_family = AF_INET;
            clientAddress_.sin_port = sender.sin_port;
            clientAddress_.sin_addr = sender.sin_addr;
        }
    }
    return true;
}

bool UdpServer::disconnect() {
    logger_->debug("disconnect");
    if (!isConnected()) {
        logger_->warn("Cannot disconnect, because not connected");
        return false;
    }

    // Create disconnect packet
    Packets::UDPConnectionPacket dcpacket {};
    dcpacket.action = Packets::UDPConnectionPacket::UDP_CONNECTION_STATE::DISCONNECT;
    std::string bytes;
    bytes.append(dcpacket.getHeader().serialize());
    bytes.append(dcpacket.serialize());
    int msg_length = Packets::PacketHeader::size + dcpacket.size;
    // Send packet
    socketInterface_->sendto(serverSocket_, bytes.c_str(), msg_length, 0,
        reinterpret_cast<sockaddr*>(&clientAddress_), sizeof(struct sockaddr_in));
    clientAddress_ = {};
    isConnected_ = false;
    return true;
}

bool UdpServer::send(const Packets::PacketHeader& header, const std::string& buffer) {
    logger_->debug("send");
    if (!isConnected()) {
        logger_->warn("Cannot send, because not connected");
        return false;
    }
    std::string bytes;
    bytes.append(header.serialize());
    bytes.append(buffer);
    int bytesSent = socketInterface_->sendto(serverSocket_, bytes.c_str(),
        Packets::PacketHeader::size+header.length, 0,
        (struct sockaddr*)&clientAddress_, sizeof(struct sockaddr_in));
    if (bytesSent < 0) {
        logger_->warn("Failed to send packet: {}", strerror(errno));
        return false;
    }
    return true;
}

bool UdpServer::receive(Packets::PacketHeader* header, std::string* buffer) {
    logger_->debug("receive");
    if (!isConnected()) {
        logger_->warn("Cannot send, because not connected");
        return false;
    }
    int bytesReceived = 0;
    std::unique_ptr<char[]> buff(new char[Packets::PacketHeader::size]);
    struct sockaddr_in sender {};
    socklen_t sendsize = sizeof(sender);
    bytesReceived = socketInterface_->recvfrom(serverSocket_, buff.get(),
        Packets::PacketHeader::size, 0, reinterpret_cast<sockaddr *>(&sender), &sendsize);
    if (bytesReceived < Packets::PacketHeader::size) {
        logger_->warn("Failed to receive header: {}", strerror(errno));
        logger_->info("Connection probably broken, going to disconnect");
        return false;
    }
    if (!header->deserialize(std::string(buff.get(), Packets::PacketHeader::size))) {
        logger_->warn("Failed to deserialize the header, consider disconnecting");
        return false;
    }
    buff.reset(new char[header->length]);
    bytesReceived = socketInterface_->recvfrom(serverSocket_, buff.get(), header->length,  0,
        reinterpret_cast<sockaddr *>(&sender), &sendsize);
    logger_->debug("Received {} bytes", bytesReceived);
    if (bytesReceived < 0) {
        logger_->warn("Failed to receive packet: {}", strerror(errno));
        logger_->info("Connection probably broken, going to disconnect");
        return false;
    }
    buffer->resize(header->length);
    buffer->assign(buff.get(), header->length);
    return true;
}

bool UdpServer::isConnected() const {
    logger_->debug("isConnected");
    return isConnected_;
}

}  // namespace networkserver
}  // namespace communication
}  // namespace crf
