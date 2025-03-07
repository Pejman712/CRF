/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Contributors: Alessandro Mosca CERN EN/STI/ECE, Giacomo Lunghi CERN EN/STI/ECE, 
 * Jorge Camarero Vera CERN EN/STI/ECE, Carlos Veiga Almagro CERN EN/STI/ECE, 
 * David Blanco Mulero CERN EN/STI/ECE, Pawel Ptasznik CERN EN/STI/ECE
 *  ==================================================================================================
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <CommUtility/CommunicationPacket.hpp>
#include <CommUtility/PacketTypes.hpp>

#include <string>

namespace Packets {

class ThermalCameraPacket : Packet {
 public:
    static const uint16_t type = THERMAL_CAMERA_PACKET_TYPE;
    cv::Mat temperatures;

    virtual std::string serialize() const {
        int size = temperatures.total() * temperatures.elemSize()+8;
        char buf[size];  // NOLINT
        std::memcpy(buf, &temperatures.cols, 4);
        std::memcpy(buf+4, &temperatures.rows, 4);
        std::memcpy(buf+8, temperatures.data, size);

        return std::string(buf, size);
    }

    virtual bool deserialize(const std::string& buffer) {
        int rows, cols;
        std::memcpy(&cols, buffer.c_str(), 4);
        std::memcpy(&rows, buffer.c_str()+4, 4);

        temperatures = cv::Mat(rows, cols, CV_32FC1,
            reinterpret_cast<char*>(buffer).c_str()+8).clone();
    }

    virtual PacketHeader getHeader() const {
        PacketHeader header;
        header.type = this->type;
        header.length = temperatures.total() * temperatures.elemSize()+8;

        return header;
    }
};

}  // namespace Packets
