#pragma once

#include "CommUtility/PacketTypes.hpp"
#include "CommUtility/CommunicationPacket.hpp"
#include "CommUtility/PacketSerialization.hpp"

#include <sstream>
#include <vector>

namespace Packets {
  class ControllinoStatusPacket : public Packet {
  public:
    static const unsigned int type = CONTROLLINO_STATUS_PACKET_TYPE;

    std::vector<uint8_t> digitalPinsConfiguration;
    std::vector<bool> digitalPinsStatus;
    std::vector<bool> relayStatus;

    std::string serialize() const override {
        std::ostringstream os;
        
        Serializer::serialize(os, digitalPinsConfiguration);
        Serializer::serialize(os, digitalPinsStatus);
        Serializer::serialize(os, relayStatus);
        return os.str();
    }
    
    bool deserialize(const std::string& buffer) override {
        std::istringstream is(buffer);

        if (!Deserializer::deserialize(is, digitalPinsConfiguration)) return false;
        if (!Deserializer::deserialize(is, digitalPinsStatus)) return false;
        if (!Deserializer::deserialize(is, relayStatus)) return false;

        return true;
    }

    PacketHeader getHeader() const override {
      PacketHeader header;
      header.type = this->type;
      header.length = serialize().length();
      return header;
    }
  };  

  class ControllinoConfigureDigitalPinPacket : public Packet {
  public:  
    static const unsigned int type = CONTROLLINO_CONFIGURE_DIGITAL_PIN_TYPE;
    static const unsigned int length = 5;

    int32_t pinID;
    int8_t setting;

    virtual std::string serialize() const {
        std::ostringstream os;
        
        Serializer::serialize(os, pinID);
        Serializer::serialize(os, setting);
        return os.str();
    }
    
    virtual bool deserialize(const std::string& buffer) {
        std::istringstream is(buffer);

        if (!Deserializer::deserialize(is, pinID)) return false;
        if (!Deserializer::deserialize(is, setting)) return false;

        return true;
    }

    virtual PacketHeader getHeader() const {
      PacketHeader header;
      header.type = this->type;
      header.length = this->length;
      return header;
    }
  };

  class ControllinoSetPinPacket : public Packet {
  public:  
    static const unsigned int type = CONTROLLINO_SET_PIN_VALUE_TYPE;
    static const unsigned int length = 6;
    
    enum PinTypes { Digital = 1, Relay = 2, Analog =3};

    PinTypes pinType;
    int32_t pinID;
    bool value;

    virtual std::string serialize() const {
        std::ostringstream os;

        uint8_t pinTypeInt = -1;
        if (pinType == Digital) pinTypeInt = 1;
        else if (pinType == Relay) pinTypeInt = 2;
        else if (pinType == Analog) pinTypeInt = 3;
        
        Serializer::serialize(os, pinTypeInt);
        Serializer::serialize(os, pinID);
        Serializer::serialize(os, value);
        return os.str();
    }
    
    virtual bool deserialize(const std::string& buffer) {
        std::istringstream is(buffer);

        uint8_t pinTypeInt = -1;
        if (!Deserializer::deserialize(is, pinTypeInt)) return false;

        pinType = static_cast<PinTypes>(pinTypeInt);
      
        if (!Deserializer::deserialize(is, pinID)) return false;
        if (!Deserializer::deserialize(is, value)) return false;

        return true;
    }

    virtual PacketHeader getHeader() const {
      PacketHeader header;
      header.type = this->type;
      header.length = this->length;
      return header;
    }
  };
}
