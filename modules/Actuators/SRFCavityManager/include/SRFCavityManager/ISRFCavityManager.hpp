/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Adrien Luthi CERN EN/SMM/MRO 2023
 *
 *  ===============================================================================================
 */

#pragma once

#include <string>

#include "crf/expected.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "SRFCavityManager/CavitySpecifications.hpp"

namespace crf::actuators::srfcavityManager {

class ISRFCavityManager : public utility::commoninterfaces::IInitializable {
 public:
    ~ISRFCavityManager() override = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Set the Cavity Orientation
     *
     * @param orientation
     */
    virtual void setCavityOrientation(CavityOrientation orientation) = 0;
    /**
     * @brief Get the Cavity Orientation
     *
     * @return CavityOrientation
     */
    virtual CavityOrientation getCavityOrientation() = 0;
    /**
     * @brief Set the Cavity Type
     *
     * @param cavityLength
     */
    virtual bool setCavityType(double cavityLength) = 0;
    /**
     * @brief Get the Cavity Type
     *
     * @return CavityType
     */
    virtual CavityType getCavityType() = 0;
    /**
     * @brief Rotate the cavity to its origin by taking the shortest path.
     *
     * @return true if the rotation is a success
     * @return false if it did not reach the origin with error code in get_response()
     */
    virtual crf::expected<bool> moveToOrigin() = 0;
    /**
     * @brief Set the new Position angle in rad. The new position can be relative to the current
     *        position or absolute between [0, 2pi].
     *
     * @param position New position to set.
     * @param absolute By default set to false. If absolute is true, the motion is absolute between
     *                 [0, 2pi]. If false, the new position will be added to the current one
     *                 (relative).
     *
     * @return true if the motion is a success
     * @return false if the motion failed with error code in get_response()
     */
    virtual crf::expected<bool> setPosition(double position, bool absolute = false) = 0;
    /**
     * @brief If the STO is off, this function set the motor to the velocity mode and enable it for
     *        operation.
     *
     * @return true if the STOs have been activated
     * @return false if it fails with error code in get_response()
     */
    virtual crf::expected<bool> enableMotor() = 0;
    /**
     * @brief Get the Position of the cavity regarding its origin (always between [0, 2pi]).
     *
     * @return true if the request succeeds, then the position is in .value()
     * @return false if the request fails with error code in get_response()
     */
    virtual crf::expected<double> getPosition() = 0;
    /**
     * @brief Set the angular velocity [rad/sec] to the motor and waits until it is constant
     *        (blocking function).
     * @param velocity This is the angular velocity target in [rad/sec].
     *
     * @return true if the motion is a success
     * @return false if the motion failed with error code in get_response()
     */
    virtual crf::expected<bool> setVelocity(double velocity) = 0;
    /**
     * @brief Get the current angular velocity [rad/sec].
     *
     * @return float if the request is a success and an error code in get_response() otherwise.
     */
    virtual crf::expected<double> getVelocity() = 0;
    /**
     * @brief Blocking function that stop the cavity rotation and waits until it is stopped. If it
     *        does not stopped after 'stopTimeoutSec' (settable in the JSON config file) it returns
     *        an error.
     *
     * @return true if the motor stopps in time or an error code in get_response() otherwise.
     */
    virtual crf::expected<bool> stop() = 0;
    /**
     * @brief Indicates if the cavity is turning.
     *
     * @return true if the cavity is turning, false if the cavity is stopped and an error code
     *         in get_response() if the request failed.
     */
    virtual crf::expected<bool> isTurning() = 0;
};

}  // namespace crf::actuators::srfcavityManager
