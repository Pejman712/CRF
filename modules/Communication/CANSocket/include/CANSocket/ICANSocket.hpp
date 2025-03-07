/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO 2017
 *         Thomas Breant CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "CommonInterfaces/IInitializable.hpp"

// Forward declaration
class can_frame;

namespace crf {
namespace communication {
namespace cansocket {

class ICANSocket: public utility::commoninterfaces::IInitializable {
 public:
    ~ICANSocket() override = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /*
     * @brief Allows to write a CAN frame on the bus.
     * @param A pointer on the frame that you want to write.
     * @return -1 if the writing failed, number of bytes written otherwise.
     */
    virtual int write(can_frame* frame) = 0;
    /*
     * @brief Allows to read a CAN frame on the bus.
     * @param A pointer on the frame which you want to store the read frame.
     * @return -1 if the reading failed, number of bytes read otherwise.
     */
    virtual int read(can_frame* frame) = 0;
    /*
     * @brief Gives the interface name.
     * @return The name of the interface.
     */
    virtual std::string getName() const = 0;
};

}  // namespace cansocket
}  // namespace communication
}  // namespace crf
