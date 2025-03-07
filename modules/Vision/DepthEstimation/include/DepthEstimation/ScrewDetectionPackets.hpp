#pragma once

#include <CommUtility/PacketTypes.hpp>
#include <CommUtility/CommunicationPacket.hpp>

namespace Packets {

	class PacketAreaStillNoClicked : public Packet {
    public:
        static const unsigned short type = PACKET_AREA_STILL_NO_CLICKED_TYPE;
        static const unsigned int   size = 2;
		unsigned int flag;
	};

	class PacketAreaClickedCoordinate : public Packet {
    public:
        static const unsigned short type = PACKET_AREA_CLICKED_COORDINATE_TYPE;
        static const unsigned int size = 10;
		int x;
        unsigned short int y;
        float z;

        PacketAreaClickedCoordinate() : x(0),y(0),z(0.0) {}

        virtual PacketHeader getHeader() const {
            PacketHeader header;
            header.type   = this->type;
            header.length = this->size;
            return header;
        }

        virtual std::string serialize() const
        {
            char buf[this->size];
            std::memcpy(buf,    &this->x, 4);
            std::memcpy(buf+4,  &this->y, 2);
            std::memcpy(buf+6,  &this->z, 4);

            return std::string(buf, this->size);
        }

        virtual bool deserialize(const std::string& buffer)
        {
            if (buffer.length() != this->size) return false;

            std::memcpy(&this->x, buffer.c_str(),    4);
            std::memcpy(&this->y, buffer.c_str()+4,  2);
            std::memcpy(&this->z, buffer.c_str()+6,  4);

            return true;
        }
	};

 	class PacketDimensionOfInterestArea : public Packet {
    public:
        static const unsigned short type = PACKET_DIMENSION_OF_INTEREST_AREA_TYPE;
        static const unsigned int   size = 24;

       	unsigned short int is_running;
        unsigned short int is_tracking;
        unsigned short int object_detected;
        unsigned short int square_center[2];
        unsigned short int square_size[2], center_circle[2];
        unsigned short int radius_circle;
        unsigned short int camera_resolution_x;
        unsigned short int camera_resolution_y;

        PacketDimensionOfInterestArea() 
            : is_running(0), is_tracking(0), object_detected(0), radius_circle(0), camera_resolution_x(0), camera_resolution_y(0)
        {
            for (unsigned short int i = 0; i < 2; i++)
                this->square_center[i] = this->square_size[i] = this->center_circle[i] = 0;
        }

        virtual PacketHeader getHeader() const {
            PacketHeader header;
            header.type   = this->type;
            header.length = this->size;
            return header;
        }

        virtual std::string serialize() const
        {
            char buf[this->size];
            std::memcpy(buf,    &this->is_running,          2);
            std::memcpy(buf+2,  &this->is_tracking,         2);
            std::memcpy(buf+4,  &this->object_detected,     2);
            std::memcpy(buf+6,  &this->square_center,       4);
            std::memcpy(buf+10, &this->square_size,         4);
            std::memcpy(buf+14, &this->center_circle,       4);
            std::memcpy(buf+18, &this->radius_circle,       2);
            std::memcpy(buf+20, &this->camera_resolution_x, 2);
            std::memcpy(buf+22, &this->camera_resolution_y, 2);

            return std::string(buf, this->size);
        }

        virtual bool deserialize(const std::string& buffer)
        {
            if (buffer.length() != this->size) return false;

            std::memcpy(&this->is_running,          buffer.c_str(),    2);
            std::memcpy(&this->is_tracking,         buffer.c_str()+2,  2);
            std::memcpy(&this->object_detected,     buffer.c_str()+4,  2);
            std::memcpy(&this->square_center,       buffer.c_str()+6,  4);
            std::memcpy(&this->square_size,         buffer.c_str()+10, 4);
            std::memcpy(&this->center_circle,       buffer.c_str()+14, 4);
            std::memcpy(&this->radius_circle,       buffer.c_str()+18, 2);
            std::memcpy(&this->camera_resolution_x, buffer.c_str()+20, 2);
            std::memcpy(&this->camera_resolution_y, buffer.c_str()+22, 2);
            
            return true;
        }
    };
}
