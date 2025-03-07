#pragma once
/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales & Sergio Villanueva Lorente CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <visp/vpMatrix.h>
#include <array>
#include <opencv2/core/core.hpp>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "RobotArmController/IRobotArmController.hpp"
#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"
#include "Cameras/ICamera.hpp"
#include "Cameras/RealSenseCamera/RealSenseCamera.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "Types/JointPositions.hpp"

namespace crf {
namespace applications {
namespace handeyecalibration {

/*
 * Class that gets the data to do the HandEye Calibration of a RobotArm
 */
class DataAcquisitionRobotArmRealsense {
 public:
    DataAcquisitionRobotArmRealsense(
        std::shared_ptr<crf::robots::robotarmcontroller::RobotArmVelocityController> controller,
        std::shared_ptr<crf::sensors::cameras::RealSenseCamera> camera,
        const std::string &pathFilename);
    ~DataAcquisitionRobotArmRealsense();

    /* This method obtains and saves in /bin directory the data needed to perform the next steps in
     * the hand-eye calibration algorithm:
     *  - Robot poses from each point of view saved as vpPoseVector in .yaml files
     *  - Camera RGB image from each point of view saved as .png files
     *  - Camera intrinsics as camera.yaml
     */
    bool acquireCalibrationData();

 private:
    /*
     * Save the instrinsic parameters of the camera as .yaml file
     */
    bool saveIntrinsicsParameters();
    /*
     * Save an image as png in the /bin directory
     */
    bool saveImage(const cv::Mat &image, const int &number);
    /*
     * Save the task position in the visp format as .yaml file
     */
    bool saveTaskPose(
        const crf::utility::types::TaskPose &position,
        const int &number);
    /*
     * Accepts the relevant values of a transformation matrix and return them in the VISP format
     */
    vpHomogeneousMatrix array2vispHomoMatrix(const std::array<float, 12> &matrix);

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::robots::robotarmcontroller::RobotArmVelocityController> controller_;
    std::shared_ptr<crf::sensors::cameras::RealSenseCamera> camera_;
    std::vector<crf::utility::types::JointPositions> positionsList_;
};

}  // namespace handeyecalibration
}  // namespace applications
}  // namespace crf
