/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
 */

#pragma once

#include <string>
#include <vector>
#include "crf/expected.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "MotionCapture/MotionCaptureMarker.hpp"

namespace crf::sensors::motioncapture {

/**
 * @ingroup group_motion_capture
 * @brief Interface class for all Motion Capture devices.
 * 
 */
class IMotionCapture : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IMotionCapture() = default;
    /**
     * @brief Initializes the Motion Capture sensor. The initialize method is responsible
     *        to configure the option of the sensor.
     * @return True the initialization ended correctly.
     * @return False otherwise.
     */
    bool initialize() override = 0;
    /**
     * @brief Deinitialize the Motion Capture sensor. The deinitialize method is responsible of
     *        deinitializing the sensor. The sensor is brought to a safe state.
     * @return True the deinitialization ended correctly.
     * @return False otherwise.
     */
    bool deinitialize() override = 0;
    /**
     * @brief Function to get the list of the object names present in motion capture area.
     * @return The expected output is a std::vector containing the list of object names as
     *         std::string. If an error occurs during the reading, a crf::code will be returned.
     */
    virtual crf::expected<std::vector<std::string>> getObjectNames() = 0;
    /**
     * @brief Function to get the pose, i.e. translation [mm] and quaternion [rad] of a single
     *        object present in motion capture area. The pose is referred to a global frame
     *        to be set in the motion capture area and defined in the motion capture configuration.
     * @param objectName is the name of the object of whose pose is asked.
     * @return The expected output is a crf::utility::types::TaskPose containing 
     *         the translation and orientation of the object. If an error occurs during
     *         the reading, a crf::code will be returned.
     */
    virtual crf::expected<crf::utility::types::TaskPose> getObjectPose(
        const std::string objectName) = 0;
    /**
     * @brief Function to get the list of the markers which constitute the detected rigid object.
     *        Each markers has a name, the global position in the scene [mm] and a flag to notify
     *        wheater the marker is occluded or not.
     * @param objectName is the name of the object for which we are asking the data.
     * @return The expected output is a std::vector containing the data relative to each marker.
     *         A marker object has three fields: 
     *         - name: Is the name of the marker, it corresponds to "Object Name" + "Numeric Index"
     *         - position: Is the position of the marker referred to a global frame defined in the scene
     *         - occluded: Is a flag to understand if the marker is occluded or not
     *         If an error occurs during the reading, a crf::code will be returned.
     */
    virtual crf::expected<std::vector<MotionCaptureMarker>> getObjectMarkers(
        const std::string objectName) = 0;
};

}  // namespace crf::sensors::motioncapture
