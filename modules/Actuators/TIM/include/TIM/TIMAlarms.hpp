/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <map>
#include <utility>
#include <shared_mutex>

#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::tim {

/**
 * @brief Class that works as container to save and access the alarms of the Train Inspection
 *        Monorail (TIM).
*/
class TIMAlarms {
 public:
    TIMAlarms();
    TIMAlarms(const TIMAlarms&);
    ~TIMAlarms() = default;

    /**
     * @brief Interprets the information received from the Siemens PLC and saves it in all the
     *        different variables.
     * 
     * @return true if manage to store all the alarm values.
     * @return false if it failed.
     */
    bool parseSiemensPLCBuffer(const std::string& buffer,
        std::map<std::string, std::array<unsigned int, 2>> variablesDBLocation);
    /**
     * @brief Resets the values of all the variables.
     */
    void clear();
    /**
     * @brief Function to check if the alarms are empty.
     * 
     * @return true if is empty
     * @return false if is not.
     */
    bool isEmpty() const;
    /**
     * @brief Tells if the is an error with the barcode reader. It could be a configuration error of
     *        the device or an error of the device not reading correctly.
     * 
     * @return true if there is an error.
     * @return false if the barcode reader is working properly.
     */
    bool barcodeReaderError() const;
    void barcodeReaderError(bool input);
    /**
     * @brief Tells if the is an error with the battery. If it is charging and the input current is
     *        almost 0 A is considered a charging error.
     * 
     * @return true if there is an error.
     * @return false if the battery is working properly.
     */
    bool batteryError() const;
    void batteryError(bool input);
    /**
     * @brief Tells if the is an issue with the motor that moves the arm to start the charging
     *        process. This information comes from the motor driver.
     * 
     * @return true if there is an error.
     * @return false if the motor of the charging arm is working properly.
     */
    bool chargingArmMotorError() const;
    void chargingArmMotorError(bool input);
    /**
     * @brief Function to check if the charging arm requires an ack to work.
     * 
     * @return true if the ack is needed.
     * @return false if there is no need for an ack.
     */
    bool chargingArmRequiresAcknowledgement() const;
    void chargingArmRequiresAcknowledgement(bool input);
    /**
     * @brief Tells if bumper in the front of the TIM touched something (possible collision). In
     *        that case the PLC does an emergency stop for safety reasons.
     * 
     * @return true if the bumper was pressed.
     * @return false if the bumper was not pressed.
     */
    bool frontBumperPressed() const;
    void frontBumperPressed(bool input);
    /**
     * @brief Tells if the is an error with the front laser scanner. It could be a configuration
     *        error of the device or an error of the device not reading correctly.
     * 
     * @return true if there is an error.
     * @return false if the front laser scanner is working properly.
     */
    bool frontLaserScannerError() const;
    void frontLaserScannerError(bool input);
    /**
     * @brief Tells if the front laser scanner is detecting any obstacle. The train must stop due
     *        to possible collision.
     * 
     * @return true if there is an obstacle.
     * @return false if the is nothing in the front of the TIM.
     */
    bool frontProtectiveFieldReading() const;
    void frontProtectiveFieldReading(bool input);
    /**
     * @brief Tells if bumper in the back of the TIM touched something (possible collision). In
     *        that case the PLC does an emergency stop for safety reasons.
     * 
     * @return true if the bumper was pressed.
     * @return false if the bumper was not pressed.
     */
    bool backBumperPressed() const;
    void backBumperPressed(bool input);
    /**
     * @brief Tells if the is an error with the back laser scanner. It could be a configuration
     *        error of the device or an error of the device not reading correctly.
     * @return true if there is an error.
     * @return false if the back laser scanner is working properly.
     */
    bool backLaserScannerError() const;
    void backLaserScannerError(bool input);
    /**
     * @brief Tells if the back laser scanner is detecting any obstacle. The train must stop due
     *        to possible collision.
     * @return true if there is an obstacle.
     * @return false if the is nothing in the back of the TIM.
     */
    bool backProtectiveFieldReading() const;
    void backProtectiveFieldReading(bool input);
    /**
     * @brief Tells if the is an error with the main motor. This error can be triggered by any of
     *        the following conditions:
     *            -  There is an error in the driver motor.
     *            -  The requested target velocity is bigger that the maximum allowed
     *            -  The acceleration is less than 0.
     *            -  The maximum velocity is less than 0
     *            -  THe maximum acceleration is less than 0
     * @return true if there is an error.
     * @return false if the main motor is working properly.
     */
    bool mainMotorError() const;
    void mainMotorError(bool input);
    /**
     * @brief Function to check if the main motor requires an ack to work.
     * @return true if the ack is needed.
     * @return false if there is no need for an ack.
     */
    bool mainMotorRequiresAcknowledgement() const;
    void mainMotorRequiresAcknowledgement(bool input);
    /**
     * @brief Tells if the is an error with the position encoder. It could be a configuration error
     *        of the device or an error of the device not able to read.
     * @return true if there is an error.
     * @return false if the position encoder is working properly.
     */
    bool positionEncoderError() const;
    void positionEncoderError(bool input);
    /**
     * @brief Tells if the is an error with the reading of position encoder. It could be that
     *        encoder is reading a negative value or that the two encoders have different readings.
     * @return true if there is an error.
     * @return false if the reading of position encoder is working properly.
     */
    bool positionEncoderReadingError() const;
    void positionEncoderReadingError(bool input);
    /*
     * @brief Tells if the is an error with the velocity encoder. It could be a configuration error
     *        of the device or an error of the device not able to read.
     * @return true if there is an error.
     * @return false if the velocity encoder is working properly.
     */
    bool velocityEncoderError() const;
    void velocityEncoderError(bool input);
    /**
     * @brief Tells if the is an error with the reading of velocity encoder. It could be that
     *        encoder is reading a negative value or that the two encoders have different readings.
     * @return true if there is an error.
     * @return false if the reading of velocity encoder is working properly.
     */
    bool velocityEncoderReadingError() const;
    void velocityEncoderReadingError(bool input);
    /**
     * @brief Tells if the emergency stop is enabled.
     * @return true if it is enabled.
     * @return false if it is not enabled.
     */
    bool emergencyStop() const;
    void emergencyStop(bool input);

 private:
    utility::logger::EventLogger logger_;
    std::atomic<bool> isEmpty_;

    std::atomic<bool> barcodeReaderError_;
    std::atomic<bool> batteryError_;
    std::atomic<bool> chargingArmMotorError_;
    std::atomic<bool> chargingArmRequiresAcknowledgement_;
    std::atomic<bool> frontBumperPressed_;
    std::atomic<bool> frontLaserScannerError_;
    std::atomic<bool> frontProtectiveFieldReading_;
    std::atomic<bool> backBumperPressed_;
    std::atomic<bool> backLaserScannerError_;
    std::atomic<bool> backProtectiveFieldReading_;
    std::atomic<bool> mainMotorError_;
    std::atomic<bool> mainMotorRequiresAcknowledgement_;
    std::atomic<bool> positionEncoderError_;
    std::atomic<bool> positionEncoderReadingError_;
    std::atomic<bool> velocityEncoderError_;
    std::atomic<bool> velocityEncoderReadingError_;
    std::atomic<bool> emergencyStop_;
};

}  // namespace crf::actuators::tim
