/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "Controllino/Controllino.hpp"
#include "Controllino/ControllinoPackets.hpp"

namespace crf {
namespace devices {

Controllino::Controllino(const std::string device_name):
    _device_name(device_name), _stopThreads(false) {
    _hasIPC = false;
    _initialized = false;
}

Controllino::Controllino(const std::string device_name, std::shared_ptr<IPC> input_IPC,
    std::shared_ptr<IPC> output_IPC):
    _device_name(device_name),
    _inputIPC(input_IPC),
    _outputIPC(output_IPC),
    _stopThreads(false) {
    _hasIPC = true;
    _initialized = false;
}

Controllino::~Controllino() {
    if (_initialized) {
        deinitialize();
    }
}

bool Controllino::initialize() {
    if (_initialized) {
        return true;
    }
    if (!(_inputIPC->open() && _outputIPC->open())) {
        return false;
    }
    _modbus_ctx = modbus_new_rtu(_device_name.c_str(), 115200, 'N', 8, 1);
    if (_modbus_ctx == NULL) {
        printf("Could not connect to controllino \n");
        perror("Controllino");
        return false;
    }
    modbus_set_slave(_modbus_ctx, 10);
    if (modbus_connect(_modbus_ctx) == -1) {
        modbus_free(_modbus_ctx);
        _modbus_ctx = NULL;
        printf("Could not connect to controllino (modbus_connect) \n");
        return false;
    }
    sleep(2);
    uint16_t msg[1];
    if (modbus_read_registers(_modbus_ctx, 1, 1, msg) == -1) {
      perror("initialize() read");
      deinitialize();
      return false;
    }
    _digitalPinsCount = msg[0];
    if (modbus_read_registers(_modbus_ctx, 3, 1, msg) == -1) {
      perror("initialize() read");
      deinitialize();
      return false;
    }
    _relayPinsCount = msg[0];
    if (_hasIPC) {
        _stopThreads = false;
        publishStatusThread = std::thread(&Controllino::publishStatus, this);
        ipcReaderThread = std::thread(&Controllino::ipcReader, this);
    }
    printf("Initialized controllino with %d Digital Pins, %d analog pins and %d relay pins\n", _digitalPinsCount, _analogPinsCount, _relayPinsCount);  // NOLINT
    _initialized = true;
    return true;
}

bool Controllino::deinitialize() {
    if (!_initialized) return true;
    if (_hasIPC) {
        _stopThreads = true;
        publishStatusThread.join();
    }
    modbus_close(_modbus_ctx);
    modbus_free(_modbus_ctx);
    _initialized = false;
    // control reaches end of non-void function
    return true;
}

void Controllino::publishStatus() {
    while (!_stopThreads.load()) {
        Packets::ControllinoStatusPacket packet;
        uint16_t* pinsConfiguration = new uint16_t[_digitalPinsCount];
        uint16_t* digitalInputStatus = new uint16_t[_digitalPinsCount];
        uint16_t* digitalOutputStatus = new uint16_t[_digitalPinsCount];
        uint16_t* relayStatus = new uint16_t[_relayPinsCount];
        {
            std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
            if (modbus_read_registers(_modbus_ctx, 100, _digitalPinsCount,
                pinsConfiguration) == -1) {
                perror("publishStatus() read");
            }
            if (modbus_read_registers(_modbus_ctx, 200, _digitalPinsCount,
                digitalInputStatus) == -1) {
                perror("publishStatus() read");
            }
            if (modbus_read_registers(_modbus_ctx, 300, _digitalPinsCount,
                digitalOutputStatus) == -1) {
                perror("publishStatus() read");
            }
            if (modbus_read_registers(_modbus_ctx, 700, _relayPinsCount, relayStatus) == -1) {
                perror("publishStatus() read");
            }
        }
        for (int i=0; i < _digitalPinsCount; i++) {
            packet.digitalPinsConfiguration.push_back(pinsConfiguration[i]);
            if (pinsConfiguration[i] == 1) {
                packet.digitalPinsStatus.push_back(static_cast<bool>(digitalInputStatus[i]));
            } else if (pinsConfiguration[i] == 2) {
                packet.digitalPinsStatus.push_back(static_cast<bool>(digitalOutputStatus[i]));
            } else {
                packet.digitalPinsStatus.push_back(false);
            }
        }
        for (int i=0; i < _relayPinsCount; i++) {
            packet.relayStatus.push_back(relayStatus[i]);
        }
        delete[] pinsConfiguration;
        delete[] digitalInputStatus;
        delete[] digitalOutputStatus;
        delete[] relayStatus;
        _outputIPC->write(packet.serialize(), packet.getHeader());
        usleep(250000);
    }
}

void Controllino::ipcReader() {
    Packets::PacketHeader header;
    std::string buffer;
    while (!_stopThreads) {
        _inputIPC->read(buffer, header);
        if (header.type == Packets::ControllinoConfigureDigitalPinPacket::type) {
            Packets::ControllinoConfigureDigitalPinPacket packet;
            packet.deserialize(buffer);
            if (packet.setting == 1) {
                createDigitalInput(packet.pinID);
            } else if (packet.setting == 2) {
                createDigitalOutput(packet.pinID);
            }
        } else if (header.type == Packets::ControllinoSetPinPacket::type) {
            Packets::ControllinoSetPinPacket packet;
            packet.deserialize(buffer);
            if (packet.pinType == Packets::ControllinoSetPinPacket::Digital) {
                setDigitalOutput(packet.pinID, packet.value);
            } else if (packet.pinType == Packets::ControllinoSetPinPacket::Relay) {
                setRelay(packet.pinID, packet.value);
            }
        }
    }
}

int Controllino::getDigitalPinConfiguration(const int pinID) {
    if ((pinID <0) || (pinID> _digitalPinsCount)) return -1;
    uint16_t msg[1];
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_read_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
      perror("createDigitalInput() read");
      return -1;
    }
    return msg[0];
}

bool Controllino::createDigitalInput(const int pinID) {
    if ((pinID <0) || (pinID> _digitalPinsCount)) return false;
    uint16_t msg[1];
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_read_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
      perror("createDigitalInput() read");
      return false;
    }
    if (msg[0] == 1) return true;       // It's already configured to be digital input
    if (msg[0] == 2) return false;      // It's already configured to be digital output
    msg[0] = 1;
    if (modbus_write_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
        perror("createDigitalInput() write");
        return false;
    }
    return true;
}

bool Controllino::createDigitalOutput(const int pinID) {
    if ((pinID <0) || (pinID> _digitalPinsCount)) return false;
    uint16_t msg[1];
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_read_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
      perror("createDigitalOutput() read");
      return false;
    }
    if (msg[0] == 1) return false;
    if (msg[0] == 2) return true;
    msg[0] = 2;
    if (modbus_write_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
        perror("createDigitalOutput() write");
        return false;
    }
    return true;
}

bool Controllino::getDigitalInput(const int pinID, bool &value) {
    if ((pinID <0) || (pinID> _digitalPinsCount)) return false;
    uint16_t msg[1];
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_read_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
      perror("getDigitalInput()");
      return false;
    }
    if (msg[0] != 1) return false;
    if (modbus_read_registers(_modbus_ctx, 200+pinID, 1, msg) == -1) {
        perror("getDigitalInput()");
        return false;
    }
    if (msg[0] == 1) {
        value = true;
        return true;
    } else if (msg[0] == 0) {
        value = false;
        return true;
    }
    return false;
}

bool Controllino::setDigitalOutput(const int pinID, const bool value) {
    if ((pinID <0) || (pinID> _digitalPinsCount)) return false;
    uint16_t msg[1];
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_read_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
      perror("setDigitalOutput() read");
      return false;
    }
    if (msg[0] != 2) return false;
    msg[0] = value ? 1 : 0;
    if (modbus_write_registers(_modbus_ctx, 300+pinID, 1, msg) == -1) {
        perror("setDigitalOutput() write");
        return false;
    }
    return true;
}

bool Controllino::getDigitalOutput(const int pinID, bool &value) {
    if ((pinID <0) || (pinID> _digitalPinsCount)) return false;
    uint16_t msg[1];
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_read_registers(_modbus_ctx, 100+pinID, 1, msg) == -1) {
      perror("getDigitalOutput()");
      return false;
    }
    if (msg[0] != 2) return false;
    if (modbus_read_registers(_modbus_ctx, 300+pinID, 1, msg) == -1) {
        perror("getDigitalOutput()");
        return false;
    }
    if (msg[0] == 1) {
        value = true;
        return true;
    } else if (msg[0] == 0) {
        value = false;
        return true;
    }
    return false;
}

bool Controllino::setRelay(const int pinID, const bool value) {
    if ((pinID <0) || (pinID> _relayPinsCount)) return false;
    uint16_t msg[1];
    msg[0] = value ? 1 : 0;
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_write_registers(_modbus_ctx, 700+pinID, 1, msg) == -1) {
        perror("setDigitalOutput() write");
        return false;
    }
    return true;
}

bool Controllino::getRelay(const int pinID, bool & value) {
    if ((pinID <0) || (pinID> _relayPinsCount)) return false;
    uint16_t msg[1];
    std::lock_guard<std::mutex> lock(_modbus_ctx_mutex);
    if (modbus_read_registers(_modbus_ctx, 700+pinID, 1, msg) == -1) {
        perror("getDigitalOutput()");
        return false;
    }
    if (msg[0] == 1) {
        value = true;
        return true;
    } else if (msg[0] == 0) {
        value = false;
        return true;
    }
    return false;
}

}  // namespace devices
}  // namespace crf
