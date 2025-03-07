/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2020
 * Contributor: Alejandro Diaz Rosales BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <memory>
#include <optional>
#include <atomic>
#include <map>

#include "CommonInterfaces/IInitializable.hpp"
#include "EtherCATDevices/ISoemApi.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

/*
 * @brief Class that retrieves the 5 etherCAT motors of the TIM conected to the given ethernet port
 *        (ifname).
 */
class TIMRobotArmWagonMotors : public utility::commoninterfaces::IInitializable {
 public:
    TIMRobotArmWagonMotors() = delete;
    TIMRobotArmWagonMotors(const TIMRobotArmWagonMotors&) = delete;
    TIMRobotArmWagonMotors(TIMRobotArmWagonMotors&&) = delete;
    explicit TIMRobotArmWagonMotors(const std::string& ifname, int ioMapSize = 218,
        std::shared_ptr<crf::devices::ethercatdevices::ISoemApi> soemApi = nullptr);
    ~TIMRobotArmWagonMotors() override;

    bool initialize() override;
    bool deinitialize() override;

    /*
     * @brief This function is used to retrieve the pointer to the Harmonic Drive 1, which is the
     *        first joint of the TIM Arm.
     * @return The Harmonic Drive 1 pointer if the the motor has been initialized.
     * @return std::nullopt otherwise.
     */
    std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> getHarmonicDrive1();
    /*
     * @brief This function is used to retrieve the pointer to the Harmonic Drive 2, which is the
     *        second joint of the TIM Arm.
     * @return The Harmonic Drive 2 pointer if the the motor has been initialized.
     * @return std::nullopt otherwise.
     */
    std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> getHarmonicDrive2();
    /*
     * @brief This function is used to retrieve the pointer to the Linear Sled, which is the
     *        second joint of the TIM Arm.
     * @return The Linear Sled pointer if the the motor has been initialized.
     * @return std::nullopt otherwise.
     */
    std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> getLinearSled();
    /*
     * @brief This function is used to retrieve the pointer to the Stabilizer, which is the fourth
     *        motor of the TIM ethercat chain.
     * @return The Stabilizer pointer if the the motor has been initialized.
     * @return std::nullopt otherwise.
     */
    std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> getStabilizer();
    /*
     * @brief This function is used to retrieve the pointer to the Shielding, which is the fifth
     *        motor of the TIM ethercat chain.
     * @return The Shielding pointer if the the motor has been initialized.
     * @return std::nullopt otherwise.
     */
    std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> getShielding();
    /*
     * @brief This function is used to retrieve the pointer to the Manager, which is in charge of
     *        the ethercat communication with the 5 motors.
     * @return The Manager pointer if the it has been initialized
     * @return std::nullopt otherwise
     */
    std::optional<std::shared_ptr<devices::ethercatdevices::EtherCATManager>> getManager();

 private:
    utility::logger::EventLogger logger_;
    std::string portName_;
    int ioMapSize_;
    std::shared_ptr<crf::devices::ethercatdevices::ISoemApi> soemApi_;
    std::atomic<bool> initialized_;
    const int etherCATCycleTime = 500;
    std::shared_ptr<devices::ethercatdevices::EtherCATManager> ECManager_;
    std::map<int, std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> motorsMap_;
    const int numberOfMotors = 5;
    const unsigned int harmonicDrive1Index = 2;
    const unsigned int harmonicDrive2Index = 1;
    const unsigned int LinearSledIndex = 5;
    const unsigned int stabilizerIndex = 3;
    const unsigned int shieldingIndex = 4;
};

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
