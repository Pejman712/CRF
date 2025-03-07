#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <utility>
#include <vector>

#include "CommonInterfaces/IInitializable.hpp"
#include "Dynamixel/DynamixelConfiguration.hpp"

namespace crf {
namespace devices {
namespace dynamixelstepper {

class IDynamixel : public utility::commoninterfaces::IInitializable{
 public:
    virtual ~IDynamixel() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * [Dynamixel::writeDynamixel writes an update parameter to the motor by flags. Currently, it
     * just set the target position * ]
     * @param  index         The Dynamixel's location on the chain
     * @param  goal_position The target position at which the motor has to rotate
     * @return               True if the new position has been well written
     */
    virtual bool writeDynamixel(uint8_t, uint16_t) = 0;

    /**
     * [Dynamixel::readCurrentPosition Reads the current motor position]
     * @param  index The Dynamixel's location on the chain
     * @return       [description]
     */
    virtual int16_t readCurrentPosition(uint8_t) = 0;

    virtual std::vector<uint16_t> getZeroPosition() = 0;
    virtual DynamixelConfiguration getConfiguration() = 0;
};

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
