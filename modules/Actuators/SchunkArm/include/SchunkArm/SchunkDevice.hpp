#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

// This is a reimplementation of the original SchunkDevice Class
// It has integrated error handling, additional functionality, and provides a clean interface
// The interface is based on "Motion Control Schunk V 1.59_en_01.00_2015-01-19.pdf"

// The Schunk Powerball Motors and Grippers by default use the following measurement units:
//  - milliDegrees , millidegrees/s , milliDegrees/(s^2)
//  - milliAmper
//  - milliSecond

// The difference between target and actual values is:
//  - when you set actual values the robot tries to perform the required movement
//    ie. if you setVelocity the robot will immediately start moving with that velocity
//          until given another command or getting blocked
//  - when you set target values the robot will retain those, and use it for subsequent movements
//    ie. if you setTargetVelocity, the velocity will be saved in the motor,
//    when you order next position movement, the movement will be performed with the given velocity

// Things that is officially possible with Schunk, but I did not implement:
//  - current control of motor, it is not possible with our robots
//  - current control of gripper, it is not possible with our grippers
//  - time based movement for absolute and relative positioning, useless
//  - loop movement between 2 positions
//  - digital IO trough the motors (whatever that means)
//  - creating simple program for robot, a program can include 16 phrases,
//      the commands can containe any movement + timing, no if-else and no loops
//  - setting the properties of the EEPROM, you can change everything in the motors,
//      from control mode to nominal current, to maxPosition

# define M_PI      3.14159265358979323846

#include <linux/can.h>

#include "CANSocket/ICANSocket.hpp"
#include "SchunkArm/ISchunkDevice.hpp"

#include "EventLogger/EventLogger.hpp"
#include <boost/optional.hpp>
#include <string>
#include <chrono>
#include <memory>

using crf::communication::cansocket::ICANSocket;

namespace crf::actuators::schunkarm {

class SchunkDevice: public ISchunkDevice {
 public:
    SchunkDevice() = delete;
    SchunkDevice(std::shared_ptr<ICANSocket> sock, int id);
    SchunkDevice(const SchunkDevice& other) = delete;
    SchunkDevice(SchunkDevice&& other) = delete;
    ~SchunkDevice() override;

    // Reads out the state of the motor
    bool initialize() override;
    bool deinitialize() override;

    // Modify robot state
    // The position commands are for fine positioning, the robot puts on the breaks after
    // you should not use it for asynchronous positioning
    // One movement takes at least 40 ms
    // When you command to the same position the robot releases and reapplies the break
    bool setPosition(float rad) override;
    // Returns true if the message was sent successfully, does not listen for acknowledge signals
    bool setVelocity(float radPerSec) override;

    // The target function immediately return, the robot will not move
    bool setTargetVelocityToPercentage(float percentage) override;
    bool setTargetAccelerationToPercentage(float percentage) override;
    // When the target current is exceeded the robot joint signals blocked
    // this method is not super robust for collision detection
    // if you set the current too high the joint will overheat and throw ERROR_I2T
    bool setTargetCurrentToPercentage(float percentage) override;

    // Puts on breaks, reboot is bugged, it looses reference if you call this before
    bool applyBreak() override;
    // Only use this before calling reboot, otherwise totally screws up the CAN
    bool fastStop() override;

    // Gets rid of simple errors, allowing the control of the robotic arm
    bool handleError(std::string error) override;

    // The state gives the current position and velocity
    // (it could also return current, but it is totally useless)
    // This is a blocking call, gets the state of the motor at current time
    bool getState() override;
    // Asynchronous call, the motor will start sending its state in every x millisecond
    // The theoretically smallest time period if you use all 6 motors is 3 ms,
    // this is the result of the bounded baud rate of CAN
    bool getStatePeriodic(int milisecond) override;

    // if you pass this the message it parses it
    // position message with cmd code 0x84
    // position message with cmd code 0x86
    bool parseStatePeriodic(can_frame frame) override;

    // Clear all the noisy messages of the bus
    // you have to use it once before initiating the motors
    static bool cleanCanBus(std::shared_ptr<ICANSocket> socket);

    static bool printCanPacket(const can_frame&);
    static std::string translateErrorToString(uint8_t msg);
    static std::string translateMessageCodeToString(uint8_t msg);

    // returns rad, between [-pi,+pi]
    boost::optional<float> getMotorPosition() override;
    boost::optional<float> getMotorVelocity() override;
    boost::optional<float> getMaxPosition() override;
    boost::optional<float> getMinPosition() override;
    boost::optional<float> getMaxVelocity() override;


    bool isBrakeActive() override;
    /* Returns whether the motor path was blocked, all of the following is true:
     * motor velocity is 0
     * the motor current is over the limit by 15%
     * the break timed out (the break is not moving)
     */
    bool isMoveBlocked() override;
    // returns none if no error, otherwise returns the errorcode
    boost::optional<uint8_t> getErrorCode() override;

 private:
    const float millidegreeToRad_;
    const float radToMilliDeg_;

    std::shared_ptr<ICANSocket> socket_;
    int canID_;
    bool initialized_;
    utility::logger::EventLogger logger_;

    // Motor parameters
    int maxPosition_;
    int minPosition_;
    int maxVelocity_;
    int maxAcceleration_;
    int maxCurrent_;

    // Control parameters
    int targetPosition_;  // mDegree
    int targetVelocity_;  // mDegree
    int targetAcceleration_;  // mDegree/s^2
    int targetCurrent_;  // mAmper

    // State parameters
    float motorPosition_;  // rad
    float motorVelocity_;  // rad/s

    // the position state message contains the first byte of the velocity message
    // to be able to read the velocity command we have to save it here
    // (terrible design by schunk)
    uint8_t lastByteFromPositionMessage_;

    bool referenced_;
    bool moving_;
    bool warning_;
    bool error_;
    bool brakeOn_;
    bool moveBlocked_;
    bool positionReached_;
    uint8_t errorCode_;

    // Gets the max, min values of pos, vel an acc
    bool getConfigurationParameters();
    // Convert errors to warnings, enables movement
    bool quitError();
    // Parses the 1 byte state message into its 8 bit components
    bool parseDeviceState(uint8_t state);
    // Blocks until the motor acknowledges the successful reboot
    // Takes around 2.5 second to perform
    bool reboot();
    // References the robot joint
    bool reference();

    // Utility functions
    static std::string translateInfoToString(uint8_t msg);
};

}  // namespace crf::actuators::schunkarm
