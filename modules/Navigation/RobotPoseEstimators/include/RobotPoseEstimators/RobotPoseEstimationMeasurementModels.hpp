/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "RobotPoseEstimators/RobotPoseEstimationSystemModel.hpp"
#include "StateEstimator/kalman/LinearizedMeasurementModel.hpp"
#include "RobotBase/RobotBaseDefaultKinematics.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"

namespace crf {
namespace applications {
namespace robotposeestimator {

class VelocityMeasurement : public Kalman::Vector<float, 4> {
 public:
    /**
    * Measurement vector measuring the velocties of the moving platform's wheels
    */
    KALMAN_VECTOR(VelocityMeasurement, float, 4)

    static constexpr size_t FRONT_LEFT = 0;
    static constexpr size_t FRONT_RIGHT = 1;
    static constexpr size_t BACK_LEFT = 2;
    static constexpr size_t BACK_RIGHT = 3;

    float frontLeft() const { return (*this)[FRONT_LEFT];}
    float frontRight() const { return (*this)[FRONT_RIGHT];}
    float backLeft() const { return (*this)[BACK_LEFT];}
    float backRight() const { return (*this)[BACK_RIGHT];}

    float& frontLeft() { return (*this)[FRONT_LEFT];}
    float& frontRight() { return (*this)[FRONT_RIGHT];}
    float& backLeft() { return (*this)[BACK_LEFT];}
    float& backRight() { return (*this)[BACK_RIGHT];}
};

class AccelerationMeasurement : public Kalman::Vector<float, 3> {
 public:
    /**
    * Measurement vector measuring the acceleration of the moving platform
    */
    KALMAN_VECTOR(AccelerationMeasurement, float, 3)

    static constexpr size_t ACCELERATION_X = 0;
    static constexpr size_t ACCELERATION_Y = 1;
    static constexpr size_t ACCELERATION_Z = 2;

    float accelerationX() const { return (*this)[ACCELERATION_X];}
    float accelerationY() const { return (*this)[ACCELERATION_Y];}
    float accelerationZ() const { return (*this)[ACCELERATION_Z];}

    float& accelerationX() { return (*this)[ACCELERATION_X];}
    float& accelerationY() { return (*this)[ACCELERATION_Y];}
    float& accelerationZ() { return (*this)[ACCELERATION_Z];}
};

class VelocityMeasurementModel : public Kalman::LinearizedMeasurementModel <SystemState,
VelocityMeasurement, Kalman::StandardBase> {
 public:
    typedef SystemState S;
    typedef VelocityMeasurement M;

    std::shared_ptr<robots::robotbase::RobotBaseDefaultKinematics> baseKinematics_;

    explicit VelocityMeasurementModel(
            const std::shared_ptr<robots::robotbase::RobotBaseConfiguration>& baseConfig_) :
    baseKinematics_(new robots::robotbase::RobotBaseDefaultKinematics(*baseConfig_)) {
        this->H.setIdentity();
        this->V.setIdentity();
    }
    /**
    * This is the measurement model for the wheel's velocities
    */
    M h(const S& x) const {
        M measurement;
        utility::types::TaskVelocity taskVelocities
            {x.xdot(), x.ydot(), .0, .0, .0, x.yawdot()};
        auto wheels = baseKinematics_->getWheelsVelocity(taskVelocities);
        if (!wheels) {
            std::cout << "kinematics wrong" << std::endl;
            return measurement;
        }
        auto wheelsVelocities = wheels.get();
        if (wheelsVelocities.size() != 4) {
            std::cout << "size of wheels wrong" << std::endl;
            return measurement;
        }
        measurement.frontLeft() = wheelsVelocities[0];
        measurement.frontRight() = wheelsVelocities[1];
        measurement.backLeft() = wheelsVelocities[2];
        measurement.backRight() = wheelsVelocities[3];
        return measurement;
    }
};

class AccelerationMeasurementModel : public Kalman::LinearizedMeasurementModel<SystemState,
AccelerationMeasurement, Kalman::StandardBase> {
 public:
    //! State type shortcut definition
    typedef SystemState S;

    //! Measurement type shortcut definition
    typedef  AccelerationMeasurement M;

    /**
    * This is the measurement model for measuring the acceleration of our
    * robot.
    */
    AccelerationMeasurementModel() {
        this->H.setIdentity();
        this->V.setIdentity();
    }
    /**
     * Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const {
        M measurement;
        // Measurement is given by the actual robot acceleration
        measurement.accelerationX() = x.xdotdot();
        measurement.accelerationY() = x.ydotdot();
        measurement.accelerationZ() = x.zdotdot();
//       measurement.yaw() += x.yawdot();
        return measurement;
    }
};

}  // namespace robotposeestimator
}  // namespace applications
}  // namespace crf
