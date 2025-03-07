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
#include <vector>

#include "IMU/Gable/GableAbstract.hpp"
#include "IMU/Gable/GableTxPDO.hpp"
#include "IMU/Gable/SE2SE3/GableSE2SE3RxPDO.hpp"

namespace crf::sensors::imu {

/**
 * @ingroup group_gable_se2se3
 * @brief Class for Gable IMU SE2 and Gable IMU SE3. On top of the functionality of the SE1
 *        (which outputs calibrated 3D rate of turn, 3D acceleration and 3D magnetic field),
 *        SE2 algorithm also computes 3D orientation data with respect to a gravity referenced
 *        frame, hence it streams drift-free roll, drift-free pitch and unreferenced yaw.
 *        Gable SE3 supports all features of the SE1 and SE2, and in addition is a full
 *        magnetometer-enhanced AHRS. In addition to the roll and pitch, it outputs a true magnetic
 *        North referenced yaw (heading).
 *        Both the devices have the possibility to enable/disable Active Heading Stabilization
 *        (AHS) feature. AHS is a software component within the sensor fusion engine designed to
 *        give a low-drift unreferenced (not North-referenced) yaw solution even in a disturbed
 *        magnetic environment. Both the devices have the possibility to modify the current filter
 *        setting, depending on the environment and application.
 * 
 */
class GableSE2SE3 : public GableAbstract {
 public:
    explicit GableSE2SE3(std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master,
        const uint16_t& id);
    ~GableSE2SE3() override = default;

    IMUSignals getSignal() override;
    crf::expected<GableInfo> getInfo() override;
    /**
     * @brief Enable Active Heading Stabilization (AHS)
     * @return A boolean containg the state of the request
     */
    bool enableAHS();
    /**
     * @brief Disable Active Heading Stabilization (AHS)
     * @return A boolean containg the state of the request
     */
    bool disableAHS();
    /**
     * @brief Request for changing the filter profile
     * @param filterProfile name of the filter profile
     * @return A boolean containg the state of the request, true if successfull and false if a
     * timeout expires or the filter is not supported by the device
     */
    bool setFilter(FilterProfile profile);

 private:
    GableRxPDO_SE2SE3* rxpdo_;
    GableTxPDO* txpdo_;
    const std::chrono::milliseconds timeoutMs_ = std::chrono::milliseconds(1000);
    bool bindIOMap() override;
    /**
     * @brief Request state transition to Config Mode
     * @return A boolean containg the state of the request, true if successfull and false if a
     * timeout expires
     */
    bool goToConfigMode();
    /**
     * @brief Request state transition to Measurement Mode
     * @return A boolean containg the state of the request, true if successfull and false if a
     * timeout expires
     */
    bool goToMeasurementMode();
    /**
     * @brief Send a custom message for changing the settings of the device or requesting data
     * @param mid Unsigned integer containing the Message ID (MID) used to configure the XSENS® IMU.
     *        Is needed to identify the type of message
     * @param dataVector Variable size vector containing the payload of the message. Can assume
     *        different sizes on the basis of the message to send.
     * @return A boolean containg the state of the request, true if successful and false if a
     * timeout expires or if the request is not supported
     */
    bool sendCustomMessage(const unsigned int mid, std::vector<unsigned int> dataVector);
};

}  // namespace crf::sensors::imu
