/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Drame CERN BE/CEM/MRO 2022
 *
 * ====================================================================
*/

#include "CodeReader/Detector/ArUcoCVDetector/ArUcoCVDetector.hpp"

namespace crf::vision::codereader {

ArUcoCVDetector::ArUcoCVDetector(const int& dictionaryId,
        const cv::Mat& intrinsicmtx, const cv::Mat& distortionmtx,
        const double& markerLength):
    logger_("ArucoDetection"),
    detected_(false),
    dictionaryId_(dictionaryId),
    markerLength_(markerLength),
    intrinsicmtx_(intrinsicmtx),
    distortionmtx_(distortionmtx) {
    logger_->debug("CTor");
}

ArUcoCVDetector::~ArUcoCVDetector() {
    logger_->debug("DTor");
}

cv::aruco::DetectorParameters ArUcoCVDetector :: parameterSetUp(const bool &detectInvertedMarker,
        const cv::aruco::CornerRefineMethod &cornerRefinementMethod,
        const int &cornerRefinementWinSize,
        const int &cornerRefinementMaxIterations,
        const double &cornerRefinementMinAccuracy,
        const int &adaptiveThreshWinSizeMin,
        const int &adaptiveThreshWinSizeMax,
        const int &adaptiveThreshWinSizeStep) {
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
    parameters.detectInvertedMarker = detectInvertedMarker;
    parameters.cornerRefinementMethod = cornerRefinementMethod;
    parameters.cornerRefinementWinSize = cornerRefinementWinSize;
    parameters.cornerRefinementMaxIterations = cornerRefinementMaxIterations;
    parameters.cornerRefinementMinAccuracy = cornerRefinementMinAccuracy;
    parameters.adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
    parameters.adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
    parameters.adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;
    return parameters;
}

bool ArUcoCVDetector::detect(const cv::Mat& frame) {
    logger_->debug("detect");
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    detected_ = false;
    arucoPositions_.clear();
    markerCorners_.clear();

    if (frame.empty()) {
        logger_->error("Empty frame");
        return false;
    }
    cv::Mat image(frame.rows, frame.cols, CV_8UC1);
    cv::cvtColor(frame, image, cv::COLOR_RGB2GRAY);
    // Detection parameters
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictionaryId_);

    cv::aruco::DetectorParameters parameters = ArUcoCVDetector::parameterSetUp(
       detectInvertedMarker_, cornerRefinementMethod_, cornerRefinementWinSize_,
       cornerRefinementMaxIterations_, cornerRefinementMinAccuracy_, adaptiveThreshWinSizeMin_,
       adaptiveThreshWinSizeMax_, adaptiveThreshWinSizeStep_);
    cv::aruco::ArucoDetector detector(dictionary, parameters);

    detector.detectMarkers(image, markerCorners_, markerIds_, rejectedCandidates);

    int nMarkers = markerCorners_.size();

    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
    std::vector<crf::utility::types::TaskPose> arucoPositions(nMarkers);
    if (nMarkers <= 0) {
        logger_->info("No aruco marker detected");
        return false;
    }
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength_/2.f, markerLength_/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength_/2.f, markerLength_/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength_/2.f, -markerLength_/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength_/2.f, -markerLength_/2.f, 0);

    // Calculate pose for each marker
    for (int i = 0; i < nMarkers; i++) {
        cv::solvePnP(objPoints, markerCorners_.at(i), intrinsicmtx_, distortionmtx_,
        rvecs.at(i), tvecs.at(i));
        crf::utility::types::TaskPose result(
            {tvecs[i][0], tvecs[i][1], tvecs[i][2]},
            crf::math::rotation::CardanXYZ({
            rvecs[i][0], rvecs[i][1], rvecs[i][2]
        }));
        arucoPositions[i] = result;
    }

    arucoPositions_ = arucoPositions;
    detected_ = true;
    return true;
}

std::vector<crf::utility::types::TaskPose> ArUcoCVDetector::getCodeTaskPose() {
    logger_->debug("getCodeTaskPose");
    if (!detected_) {
        throw(std::runtime_error("Trying to access the Aruco code position before detecting one"));
    }
    return arucoPositions_;
}

std::array<float, 8> ArUcoCVDetector::getPositionInImage() {
    logger_->debug("getPositionInImage");
    if (!detected_) {
        throw(std::runtime_error("Trying to access the Aruco code position before detecting one"));
    }
    std::array<float, 8> result;
    for (uint8_t i = 0; i < (result.size())/2; i++) {
        result[2*i] = markerCorners_[0][i].x;
        result[2*i+1] = markerCorners_[0][i].y;
    }
    return result;
}

std::vector<int> ArUcoCVDetector::getMarkersIds() {
    logger_->debug("getMarkersIds");
    if (!detected_) {
        throw(std::runtime_error("Trying to access the Aruco Id position before detecting one"));
    }
    return markerIds_;
}

}  // namespace crf::vision::codereader

