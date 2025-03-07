/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include "CommonInterfaces/IInitializable.hpp"

// FWDs
namespace cv {
class Mat;
}  // namespace cv

namespace crf {
namespace sensors {
namespace thermalcamera {

/**
* Interface class for thermal camera devices.
* Known subclasses: OptrisPi
*/
class IThermalCamera: public utility::commoninterfaces::IInitializable {
 public:
    IThermalCamera() = default;
    virtual ~IThermalCamera() = default;

    /**
    * Returns openCv matrix representing a thermal image.
    * Matrix datatype is float.
    * Each matrix element is the thermal value in [C] (Celsius)
    * This method will return empty matrix is no thermal data is available
    */
    virtual cv::Mat getFrame() = 0;
    /**
    * Send the thermal data through provided IPC
    * Returns true on success
    * Returns false on failure (e.g. empty matrix provided)
    */
    virtual bool publishFrame(const cv::Mat& frame) = 0;
    /**
    * This method is introduced just for the example/debugging purpose
    * Displays one thermal frame
    * Returns true on success and false on failure
    */
    virtual bool showFrame() = 0;
    /**
    * This method is introduced just for the example/debugging purpose
    * Displays the video stream
    * Returns true on success and false on failure
    */
    virtual bool streamVideo() = 0;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
