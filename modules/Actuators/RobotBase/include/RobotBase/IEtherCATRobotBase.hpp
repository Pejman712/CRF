#pragma once

/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <string>
#include <memory>
#include <map>
#include <boost/optional.hpp>

#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/ISoemApi.hpp"

namespace crf {
namespace actuators {
namespace robotbase {

class IEtherCATRobotBase {
 public:
    /**
     * @brief virtual destructor
     */
    virtual ~IEtherCATRobotBase() = default;

    /**
     * @brief Initializes the EtherCAT motors
     * The initialize method is responsible of initializing all the EtherCATRobotBase motors
     * running on EtherCAT communication:
     * MotorFrontLeft;
     * MotorFrontRight;
     * MotorBackLeft;
     * MotorBackRight.
     * 
     * @return true the initialization ended correctly for all the motors
     * @return false otherwise
     */
    virtual bool initialize() = 0;

    /**
     * @brief Deinitialize the EtherCAT motors
     * The deinitialize method is responsible of deinitializing all the EtherCATRobotBase motors
     * running on EtherCAT communication:
     * MotorFrontLeft;
     * MotorFrontRight;
     * MotorBackLeft;
     * MotorBackRight.
     * 
     * @return true the deinitialization ended correctly for all the motors
     * @return false otherwise
     */
    virtual bool deinitialize() = 0;

    /**
     * @brief Retrieve MotorFrontLeft;
     * This function is used to retrieve the pointer to the MotorFrontLeft, which is the first
     * first motor of the EtherCATRobotBase;
     * 
     * @return the MotorFrontLeft pointer if the the motor has been initialized
     * @return boost::none otherwise
     */
    virtual boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorFrontLeft() = 0;

    /**
     * @brief Retrieve MotorFrontRight;
     * This function is used to retrieve the pointer to the MotorFrontRight, which is the second
     * motor of the EtherCATRobotBase;
     * 
     * @return the MotorFrontRight pointer if the the motor has been initialized
     * @return boost::none otherwise
     */
    virtual boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorFrontRight() = 0;

    /**
     * @brief Retrieve MotorBackLeft
     * This function is used to retrieve the pointer to the MotorBackLeft, which is the third
     * motor of the EtherCATRobotBase;
     * 
     * @return the MotorBackLeft pointer if the the motor has been initialized
     * @return boost::none otherwise
     */
    virtual boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorBackLeft() = 0;

    /**
     * @brief Retrieve MotorBackRight
     * This function is used to retrieve the pointer to the MotorBackRight, which is the fourth
     * motor of the EtherCATRobotBase;
     * 
     * @return the MotorBackRight pointer if the the motor has been initialized
     * @return boost::none otherwise
     */
    virtual boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorBackRight() = 0;

    /**
     * @brief Retrieve Manager
     * This function is used to retrieve the pointer to the Manager, which is in charge of the
     * ethercat communication with the three joints;
     * 
     * @return the Manager pointer if the it has been initialized
     * @return boost::none otherwise
     */
    virtual boost::optional<std::shared_ptr<devices::ethercatdevices::EtherCATManager>>
    getManager() = 0;
};

}  // namespace robotbase
}  // namespace actuators
}  // namespace crf
