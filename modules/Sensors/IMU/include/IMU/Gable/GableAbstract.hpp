/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Henry Paul Espinosa Peralta CERN BE/CEM/MRO 2023
 *         Giancarlo D'Ago CERN BE/CEM/MRO 2023
 * 
 *  ==================================================================================================
 */

#pragma once

#include <array>
#include <memory>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"
#include "EtherCATDrivers/BasicEtherCATDriver/BasicEtherCATDriver.hpp"
#include "IMU/IIMU.hpp"
#include "IMU/Gable/GableInfo.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup group_gable
 * @brief Abstract class for Gable IMU sensors.
 * 
 */
class GableAbstract : public IIMU, public crf::devices::ethercatdrivers::BasicEtherCATDriver {
 public:
    explicit GableAbstract(std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master,
        const uint16_t& id);
    ~GableAbstract();
    bool initialize() override;
    bool deinitialize() override;
    crf::expected<bool> calibrate() override;

 protected:
    std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master_;
    crf::utility::logger::EventLogger logger_;
    IMUSignals imuData_;

    IMUSignals getSignal() override = 0;
    /**
     * @brief Get technical and configuration infos of the device.
     * @return A struct containing the info of the device. These data are encapsulated
     *         in a crf::expected object indicating whether the data is available or not.
     */
    virtual crf::expected<GableInfo> getInfo() = 0;
    bool bindIOMap() override = 0;
};

}  // namespace crf::sensors::imu
