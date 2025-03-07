/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales & Sergio Villanueva Lorente CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <fstream>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>
#include <librealsense2/rs.hpp>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/io/vpImageIo.h>

#include "EventLogger/EventLogger.hpp"
#include "HandEyeCalibration/DataAcquisitionRobotArmRealSense.hpp"
#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"
#include "RobotArmController/IRobotArmController.hpp"
#include "Types/JointPositions.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

namespace crf {
namespace applications {
namespace handeyecalibration {

DataAcquisitionRobotArmRealsense::DataAcquisitionRobotArmRealsense(
    std::shared_ptr<crf::robots::robotarmcontroller::RobotArmVelocityController> controller,
    std::shared_ptr<crf::sensors::cameras::RealSenseCamera> camera,
    const std::string &pathFilename) :
    logger_("DataAcquisitionRobotArmRealsense"),
    controller_(controller),
    camera_(camera) {
    logger_->debug("CTor");

    // Read the path from the json file
    std::ifstream config(pathFilename);
    nlohmann::json jConfig;
    config >> jConfig;
    int jointsNumber = controller_->getJointPositions().size();
    int positionsSize = jConfig.at("positionsSize").get<int>();
    for (int i = 0; i < positionsSize; i++) {
        crf::utility::types::JointPositions pos(jointsNumber);
        for (int j = 0; j < jointsNumber; j++) {
            std::string position = "position" + std::to_string(i);
            std::string joint = "joint" + std::to_string(j);
            float jointPositions = jConfig.at("positions").at(position).at(joint).get<float>();
            pos(j) = jointPositions;
        }
        positionsList_.push_back(pos);
    }
    logger_->info("The path was loaded correctly");
}

DataAcquisitionRobotArmRealsense::~DataAcquisitionRobotArmRealsense() {
    logger_->debug("DTor");
}

bool DataAcquisitionRobotArmRealsense::acquireCalibrationData() {
    logger_->debug("acquireCalibrationData");
    if (!saveIntrinsicsParameters()) {
            logger_->error("Unable to acquire the calibration data");
            return false;
    }
    unsigned int number = 1;
    for (unsigned int pathIndex = 0; pathIndex < positionsList_.size(); pathIndex++) {
        std::vector<crf::utility::types::JointPositions> path;
        path.push_back(positionsList_[pathIndex]);
        std::future<bool> result = controller_->setPosition(positionsList_[pathIndex]);
        if (!result.get()) {
            logger_->error("Unable to send the next position to robot");
            return false;
        }

        if (!saveImage(camera_->getFrame(), number)) {
            logger_->error("Unable to acquire the calibration data");
            return false;
        }
        if (!saveTaskPose(controller_->getTaskPose(), number)) {
            logger_->error("Unable to acquire the calibration data");
            return false;
        }

        number++;
    }
    return true;
}

bool DataAcquisitionRobotArmRealsense::saveIntrinsicsParameters() {
    logger_->debug("saveIntrinsicsParameters");
    auto colorCameraMatrix = camera_->getColorCameraMatrix();
    auto colorDistortionMatrix = camera_->getColorCameraMatrix();
    cv::Mat image = camera_->getFrame();
    vpImage<vpRGBa> imageVISP;
    vpImageConvert::convert(image, imageVISP);
    if (imageVISP.getNumberOfPixel() == 0) {
      logger_->error("Unable to capture image from the camera");
      return false;
    }
    unsigned int width = imageVISP.getWidth();
    unsigned int height = imageVISP.getHeight();
    logger_->info("Image size: {} x {}", width, height);
    vpCameraParameters camParameters;
    vpXmlParserCamera camParametersXML;
    std::cout << colorDistortionMatrix->size << " size of the matrix" << std::endl;
    double u0 = colorCameraMatrix->at<float>(0, 2);
    double v0 = colorCameraMatrix->at<float>(1, 2);
    double px = colorCameraMatrix->at<float>(0, 0);
    double py = colorCameraMatrix->at<float>(1, 1);
    double kdu = 0;
    camParameters.initPersProjWithDistortion(px, py, u0, v0, -kdu, kdu);
    camParametersXML.save(camParameters, "camera.xml", "Camera", width, height);
    logger_->info("Save: camera.xml");
    return true;
}

bool DataAcquisitionRobotArmRealsense::saveImage(const cv::Mat &image, const int &number) {
    logger_->debug("saveImage");
    std::vector<int> compressionParams;
    // compressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compressionParams.push_back(9);
    std::string name = "image-" + std::to_string(number) + ".png";
    if (!cv::imwrite(name, image, compressionParams)) {
        logger_->error("Unable to save the image");
        return false;
    }
    logger_->info("Save: {}", name);
    return true;
}

bool DataAcquisitionRobotArmRealsense::saveTaskPose(
    const crf::utility::types::TaskPose &position,
    const int &number) {
    logger_->debug("saveTaskPose");

    vpPoseVector fPe(array2vispHomoMatrix(position.getPosRotMatrix()));
    std::string name = "pose_fPe_" + std::to_string(number) + ".yaml";
    if (!fPe.saveYAML(name, fPe)) {
        logger_->error("Unable to save the task position");
        return false;
    }
    logger_->info("Save: {}", name);
    return true;
}

vpHomogeneousMatrix DataAcquisitionRobotArmRealsense::array2vispHomoMatrix(
    const std::array<float, 12> &matrix) {
    logger_->debug("array2vispHomoMatrix");
    vpHomogeneousMatrix output;
    output[0][0] = matrix[3];
    output[0][1] = matrix[4];
    output[0][2] = matrix[5];
    output[1][0] = matrix[6];
    output[1][1] = matrix[7];
    output[1][2] = matrix[8];
    output[2][0] = matrix[9];
    output[2][1] = matrix[10];
    output[2][2] = matrix[11];
    output[0][3] = matrix[0];
    output[1][3] = matrix[1];
    output[2][3] = matrix[2];
    output[3][0] = 0.0;
    output[3][1] = 0.0;
    output[3][2] = 0.0;
    output[3][3] = 1.0;
    return output;
}

}  // namespace handeyecalibration
}  // namespace applications
}  // namespace crf
