#pragma once

#include <atomic>
#include <modbus/modbus.h>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>


#include "CommonInterfaces/IInitializable.hpp"
#include "IPC/IPC.hpp"

namespace crf {
namespace devices {  

class Controllino : public commoninterfaces::IInitializable {
    bool _hasIPC;
    bool _initialized;

    std::string _device_name;
    std::shared_ptr<IPC> _inputIPC;
    std::shared_ptr<IPC> _outputIPC;

    std::mutex _modbus_ctx_mutex;
    modbus_t* _modbus_ctx;

    int _digitalPinsCount;
    int _relayPinsCount;
    int _analogPinsCount;


    std::atomic<bool> _stopThreads;
    std::thread publishStatusThread;
    void publishStatus();

    std::thread ipcReaderThread;
    void ipcReader();
  public:
    Controllino(const std::string device_name);
    Controllino(const std::string device_name, std::shared_ptr<IPC> input_IPC, std::shared_ptr<IPC> output_IPC);
    ~Controllino();

    bool initialize() override;
    bool deinitialize() override;

    bool createDigitalInput(const int pinID);
    bool createDigitalOutput(const int pinID);

    int getDigitalPinConfiguration(const int pinID);
    bool getDigitalInput(const int pinID, bool &value);
    bool setDigitalOutput(const int pinID, const bool value);
    bool getDigitalOutput(const int pinID, bool &value);

    bool setRelay(const int pinID, const bool value);
    bool getRelay(const int pinID, bool & value);

    inline int getDigitalPinsCount() {return _digitalPinsCount;}
    int getRelayPinsCount() {return _relayPinsCount;}
    int getAnalogPinsCount() {return _analogPinsCount;}
};

}  // namespace temp
}  // namespace crf
