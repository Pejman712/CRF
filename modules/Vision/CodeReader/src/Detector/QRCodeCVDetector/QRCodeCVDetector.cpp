/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include "CodeReader/Detector/QRCodeCVDetector/QRCodeCVDetector.hpp"

namespace crf::vision::codereader {

QRCodeCVDetector::QRCodeCVDetector(
    const cv::Size& cameraResolution, const double& widthQR, const double& heightQR):
    cameraResolution_(cameraResolution),
    widthQR_(widthQR),
    heightQR_(heightQR),
    detected_(false),
    parametersProvided_(true),
    logger_("QRCodeCVDetector") {
    logger_->debug("CTor");
    if (cameraResolution.empty()) {
        logger_->warn("Constructor parameters not provided, some functions might not work");
        parametersProvided_ = false;
    }
}

QRCodeCVDetector::~QRCodeCVDetector() {
    logger_->debug("DTor");
}

bool QRCodeCVDetector::detect(const cv::Mat &frame) {
    logger_->debug("detect");
    if (!frame.data || frame.empty()) {
        logger_->warn("No data in frame");
        return false;
    }
    frame_ = frame;
    cv::QRCodeDetector qrDetect = cv::QRCodeDetector();
    try {
        detected_ = qrDetect.detect(frame, positions_);
    } catch (cv::Exception &e) {
        std::cout << "Exception during detection: " << e.what() << std::endl;
        return false;
    }
    if (detected_ && (positions_.empty())) {
        logger_->warn("The detection was erroneus");
        return false;
    }
    return detected_;
}

std::vector<crf::utility::types::TaskPose> QRCodeCVDetector::getCodeTaskPose() {
    logger_->debug("getCodeTaskPose");
    if (!parametersProvided_) {
        throw std::logic_error(
            "Parameters not provided, can't obtain TaskPose");
    }
    if (!detected_) {
        throw std::runtime_error(
            "Trying to access the QR code position before detecting one");
    }

    // Camera parameters
    vpCameraParameters cam(cameraResolution_.height, cameraResolution_.width,
        frame_.cols/2, frame_.rows/2);

    // Real dimensions of the QR code
    std::vector<vpPoint> point;
    point.push_back(vpPoint(-widthQR_/2, -heightQR_/2, 0));
    point.push_back(vpPoint(widthQR_/2, -heightQR_/2, 0));
    point.push_back(vpPoint(widthQR_/2, heightQR_/2, 0));
    point.push_back(vpPoint(-widthQR_/2, heightQR_/2, 0));

    std::vector<vpImagePoint> ip;
    std::array<float, 8> points = getPositionInImage();
    for (uint64_t i = 0; i < 4; i++) {
        ip.push_back(vpImagePoint(points[2*i], points[2*i+1]));
    }

    vpPose pose;
    double x = 0, y = 0;
    for (unsigned int i = 0; i < point.size(); i++) {
        vpPixelMeterConversion::convertPoint(cam, ip.at(i), x, y);
        point[i].set_x(x);
        point[i].set_y(y);
        pose.addPoint(point[i]);
    }

    vpHomogeneousMatrix cMo;
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag) {
        cMo = cMo_dem;
    } else {
        cMo = cMo_lag;
    }

    pose.computePose(vpPose::VIRTUAL_VS, cMo);

    double q0 = vpQuaternionVector(cMo.getRotationMatrix()).t()[0];
    double q1 = vpQuaternionVector(cMo.getRotationMatrix()).t()[1];
    double q2 = vpQuaternionVector(cMo.getRotationMatrix()).t()[2];
    double q3 = vpQuaternionVector(cMo.getRotationMatrix()).t()[3];

    /*
     *  Return the traslation matrix and trasform quaternions into
     *  Roll, Pitch and Yaw
     *  (jplayang)
     */

    crf::utility::types::TaskPose result({
        cMo.getTranslationVector().t()[0],
        cMo.getTranslationVector().t()[1],
        cMo.getTranslationVector().t()[2]},
        crf::math::rotation::CardanXYZ({
        atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2)),
        asin(2*(q0*q2+q1*q3)),
        atan2(2*(q0*q3+q2*q1), 1-2*(q2*q2 + q3*q3))
    }));
    return std::vector<crf::utility::types::TaskPose>({result});
}

std::array<float, 8> QRCodeCVDetector::getPositionInImage() {
    logger_->debug("getPositionInImage");
    if (!detected_) {
        throw(std::runtime_error(
            "Trying to access the QR code position before detecting one"));
    }
    std::array<float, 8> result;
    for (uint8_t i = 0; i < result.size(); i++) {
        result[i] = positions_.at<float>(i);
    }
    return result;
}

}  // namespace crf::vision::codereader
