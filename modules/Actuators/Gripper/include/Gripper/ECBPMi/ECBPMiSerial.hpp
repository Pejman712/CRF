/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alessandro Vascelli CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <unistd.h>
#include <thread>
#include <iomanip>
#include <iostream>
#include <string>

#include "Tool/IActiveTool.hpp"
#include "SerialCommunication/ISerialCommunication.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::gripper {

using crf::communication::serialcommunication::ISerialCommunication;

/**
 * @ingroup group_ecbpmiserial
 * @brief This class is used to control the Schmalz ECBPMi vacuum gripper it inherits methods from
 *        IActiveTool interface and implements it with the specific features
 */
class ECBPMiSerial: public crf::devices::tool::IActiveTool {
 public:
    explicit ECBPMiSerial(std::shared_ptr<ISerialCommunication> serial,
        const std::string& urdfPath);

    ~ECBPMiSerial() override;

    bool initialize() override;
    bool deinitialize() override;

    crf::expected<bool> activate() override;
    crf::expected<bool> deactivate() override;
    crf::expected<bool> isActive() override;
    std::shared_ptr<urdf::ModelInterface> getURDF() override;

 private:
    enum class Option {
        Activate,
        Deactivate
    };

    std::shared_ptr<ISerialCommunication> serial_;
    bool initialized_;
    crf::utility::logger::EventLogger logger_;
    std::thread controlThread_;
    std::atomic<Option> currentOption_ = Option::Deactivate;
    std::atomic<bool> isRunning_ = false;
    std::shared_ptr<urdf::ModelInterface> model_;

    const unsigned int numberOfAttempts_ = 25;
    const std::chrono::milliseconds blowOffTime_ = std::chrono::milliseconds(700);
    const std::chrono::milliseconds sendingTime_ = std::chrono::milliseconds(35);
    const std::chrono::milliseconds initializeTime_ = std::chrono::milliseconds(30);
    const unsigned char unlock_[77] = {0xF8, 0x78, 0xF8, 0x01, 0x87, 0xE0, 0x00, 0xEA, 0x06, 0x04,
        0x20, 0x74, 0xA0, 0x00, 0xFA, 0x00, 0x1A, 0xA0, 0x0E, 0x31, 0x30, 0x2E, 0x30, 0x33, 0x2E,
        0x30, 0x31, 0x2E, 0x30, 0x30, 0x35, 0x38, 0x34, 0xDE, 0xA0, 0x00, 0x15, 0x00, 0x35, 0xA0,
        0x07, 0x32, 0x37, 0x38, 0x30, 0x30, 0x32, 0x38, 0x12, 0xA0, 0x00, 0xF0, 0x00, 0x10, 0xA0,
        0x14, 0x0A, 0x0E, 0x01, 0x01, 0x03, 0x02, 0x02, 0x00, 0x82, 0x00, 0x00, 0x00, 0xEA, 0x01,
        0x87, 0xE0, 0x00, 0x2A, 0x6B, 0x7C, 0x3A};
    const unsigned char stayCommand_[7] = {0xF1, 0x00, 0x3C, 0x1E, 0x00, 0X4B, 0xF1};
    const unsigned char activateCommand_[7] ={0xF1, 0x01, 0x10, 0x0A, 0x00, 0X0C, 0xF1};
    const unsigned char deactivateCommand_[7] ={0xF1, 0x02, 0x3C, 0x1E, 0x00, 0X4D, 0xF1};

    void control();
    bool sendCommand(const unsigned char* command);
    void checkResponse();
};

}  // namespace crf::actuators::gripper
