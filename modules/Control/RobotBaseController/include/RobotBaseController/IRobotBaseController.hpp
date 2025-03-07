/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>
#include <future>
#include <optional>

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/TaskTypes/TaskSignals.hpp"

#define SE2_SPACE_SIZE 3
#define LINEAR_DIM_ID 0
#define TRANSVERSAL_DIM_ID 1
#define ROTATION_DIM_ID 5

namespace crf::control::robotbasecontroller {

class IRobotBaseController : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IRobotBaseController() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief
     * @param
     * @return
     */
    virtual std::future<bool> setPosition(
        const crf::utility::types::TaskPose& targetPosition) = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual std::future<bool> setPosition(
        const std::vector<crf::utility::types::TaskPose>& targetPosition) = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual bool setVelocity(
        const crf::utility::types::TaskVelocity& targetVelocity) = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual bool setVelocity(
        const std::vector<crf::utility::types::TaskVelocity>& targetVelocity) = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual bool interruptTrajectory() = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual crf::utility::types::TaskPose getPosition() = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual crf::utility::types::TaskVelocity getVelocity() = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual bool setMaximumVelocity(
        const utility::types::TaskVelocity& maxVelocity) = 0;
    /**
     * @brief
     * @param
     * @return
     */
    virtual bool setMaximumAcceleration(
        const utility::types::TaskAcceleration& maxAcceleration) = 0;
};

}  // namespace crf::control::robotbasecontroller
