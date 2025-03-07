/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Drame CERN BE/CEM/MRO 2022
 *
 * ====================================================================
*/

#pragma once

#include <iostream>
#include <vector>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include "EventLogger/EventLogger.hpp"
#include "CodeReader/Detector/IDetector.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

/**
 * @brief Class that is used to detect ArUco markers. Each ArUco marker
 * has it's own ID number and the number of the dictionary it comes from.
 * The number of the dictionary is defined by the size of the binary matrix
 * inside the ArUco marker (5x5, 6x6, 7x7 etc). The ID number is configured 
 * by the binary matrix inside the ArUco marker aka the number of white and
 * black squares will determine the ID of the ArUco marker.
*/

namespace crf::vision::codereader {

class ArUcoCVDetector : public IDetector {
 public:
  /**
   * @param  dictionaryId is the markers dictionnary number, can define 5x5, 6x6, 7x7 markers.
   * @param  markerLength is the length of the side of the marker
   * @param  intrinsicmtx is the intrinsic matrix of the camera
   * @param  distortionmtx is the distortion matrix of the camera
   **/
  explicit ArUcoCVDetector(const int& dictionaryId,
      const cv::Mat& intrinsicmtx, const cv::Mat& distortionmtx,
      const double& markerLength);
  ~ArUcoCVDetector();
  bool detect(const cv::Mat& frame) override;
  std::vector<crf::utility::types::TaskPose> getCodeTaskPose() override;
  std::array<float, 8> getPositionInImage() override;
  /**
   * @brief Function that returns marker IDs of the ArUco markers.
   * @return Vector of ints aka the values of the marker IDs
  */
  std::vector<int> getMarkersIds();

 protected:
 /**
  * @brief Function for setting up parameters for extracting ArUco marker candidates from
  * the image.(image processing) 
  * @param detectInvertedMarker is a boolean to check if there are any white markers to detect
  * @param cornerRefinementMethod is a corner refinement method
  * @param cornerRefinementWinSize is window size for the corner refinement process (in pixels)
  * @param cornerRefinementMaxIterations is maximum number of iterations 
  * for stop criteria of the corner refinement process
  * @param cornerRefinementMinAccuracy is minimum error for the stop criteria 
  * of the corner refinement process
  * @param adaptiveThreshWinSizeMin is minimum window size for adaptive thresholding 
  * before finding contours
  * @param adaptiveThreshWinSizeMax is maximum window size for adaptive thresholding 
  * before finding contours 
  * @param adaptiveThreshWinSizeStep increments from adaptiveThreshWinSizeMin to adaptiveThreshWinSizeMax 
   * during the thresholding.
 */
  cv::aruco::DetectorParameters parameterSetUp(const bool &detectInvertedMarker,
      const cv::aruco::CornerRefineMethod &cornerRefinementMethod,
      const int &cornerRefinementWinSize,
      const int &cornerRefinementMaxIterations,
      const double &cornerRefinementMinAccuracy,
      const int &adaptiveThreshWinSizeMin,
      const int &adaptiveThreshWinSizeMax,
      const int &adaptiveThreshWinSizeStep);

 private:
  std::vector<crf::utility::types::TaskPose> arucoPositions_;
  crf::utility::logger::EventLogger logger_;
  bool detected_;
  int dictionaryId_;
  double markerLength_;
  cv::Mat intrinsicmtx_;
  cv::Mat distortionmtx_;
  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_;

  /**
   * @brief Parameter to check if there is a white marker to detect
   * Generating a white marker can be done by inverting a normal marker 
   * with a tilde. (~markerImage)
   */
  const bool detectInvertedMarker_ = true;

  /**
   * @brief Corner refinment method. Possible values are:
   * CORNER_REFINE_NONE - no refinement 
   * CORNER_REFINE_SUBPIX - does subpixel refinement 
   * CORNER_REFINE_CONTOUR - uses contour-Points 
   * CORNER_REFINE_APRILTAG - uses the AprilTag2 approach
   * Default value for this parameter is CORNER_REFINE_NONE
  */
  const cv::aruco::CornerRefineMethod cornerRefinementMethod_ = cv::aruco::CORNER_REFINE_CONTOUR;
  /**
   * @brief Window size for the corner refinement process (in pixels)
   * Default value is 5
  */
  const int cornerRefinementWinSize_ = 5;
  /**
   * @brief Maximum number of iterations for stop criteria of the corner refinement process 
   * Default value is 30
  */
  const int cornerRefinementMaxIterations_ = 15;
  /**
   * @brief Minimum error for the stop criteria of the corner refinement process
   * Default value is 0.1
  */
  const double cornerRefinementMinAccuracy_ = 0.01;
  /**
   * @brief Minimum window size for adaptive thresholding before finding contours 
   * Default value is 3
  */
  const int adaptiveThreshWinSizeMin_ = 3;
  /**
   * @brief Maximum window size for adaptive thresholding before finding contours 
   * Default value is 23
  */
  const int adaptiveThreshWinSizeMax_ = 23;
  /**
   * @brief Increments from adaptiveThreshWinSizeMin to adaptiveThreshWinSizeMax 
   * during the thresholding.
   * Default value is 10
  */
  const int adaptiveThreshWinSizeStep_ = 5;
};

}  // namespace crf::vision::codereader
