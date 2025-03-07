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

#include <memory>

#include "IMU/Gable/GableAbstract.hpp"
#include "IMU/Gable/GableTxPDO.hpp"
#include "IMU/Gable/SE1/GableSE1RxPDO.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup group_gable_se1
 * @brief Class for Gable IMU SE1. The SE1 is an IMU that outputs calibrated 3D rate of turn,
 *        3D acceleration and 3D magnetic field. Data communication is via EtherCAT protocol.
 * 
 */
class GableSE1 : public GableAbstract {
 public:
    explicit GableSE1(std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master,
        const uint16_t& id);
    ~GableSE1() override = default;

    IMUSignals getSignal() override;
    crf::expected<GableInfo> getInfo() override;

 private:
    GableRxPDO_SE1* rxpdo_;
    GableTxPDO* txpdo_;
    bool bindIOMap() override;
};

}  // namespace crf::sensors::imu
