/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

  // #pragma once

#ifndef HOMOGRAPHY_H  // NOLINT
#define HOMOGRAPHY_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>      // for cvtColor
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>   // for harris detector
#include <opencv2/xfeatures2d.hpp>  // SURF
#include <opencv2/highgui.hpp>      // for webCam


/*!
  A more elaborate class description.
*/
class Homography {
 public:
    Homography();

    // void setFrame(const cv::Mat &);
    void setPattern(const cv::Mat&);

    void setCameraMatrix(const cv::Mat &K) { this->camera_Matrix_ = K.clone(); }
    void setoriginRoiPoint(const cv::Point &);
    double targetDistance();
    std::vector<int> split(const std::string &, const char);
    void drawSceneBox(cv::Mat &);
    void drawBoundingBox(cv::Mat &);
    std::vector<cv::DMatch> maximumMatches();

    void filterMatches(double &, double &);
    std::vector<cv::Point2f> getScenePoints() const { return this->scene_; }
    std::vector<cv::Point2f> getBoundingBox() const { return bounding_corners_; }
    std::vector<cv::Point2f> getSceneBox() const { return scene_corners_; }
    std::vector<cv::Point2f> getGoodPointsPattern() const { return this->goodPointsPattern_; }
    std::vector<cv::Point2f> getGoodPointsFrame() const { return this->goodPointsFrame_; }
    std::vector<cv::Point> getGoodPointsObj() const { return this->obj_; }
    std::vector<cv::Point2f> getGoodPointsScene() const { return this->scene_; }
    cv::Point getOriginRoi() const { return this->originRoi_; }
    cv::Mat getCameraMatrix() const { return this->camera_Matrix_; }

    void fundamentalMatrix();
    bool runHomography(cv::Mat &, std::string &);  // main function

    ~Homography() {}

 private:
    // std::string name_;
    int scale_, delta_, numPatterns_;
    double focalLength_;

    std::vector<cv::KeyPoint> keyPointsPattern_, keyPointsFrame_;
    cv::Mat descriptorsPattern_, descriptorFrame_, camera_Matrix_;
    cv::Mat frame_, pattern_;
    std::vector<cv::DMatch> matches_;
    std::vector<cv::Point2f> bounding_corners_, scene_corners_, scene_;
    std::vector<cv::Point2f> goodPointsPattern_, goodPointsFrame_;
    std::vector<cv::Point> obj_;
    cv::Point originRoi_;

    void processPattern(bool);
    void edgesMap(bool);
    void findFeatures(bool);
    std::vector<cv::KeyPoint> DetectKeyPoints(bool);
    cv::Mat ComputeDescriptors(bool);
    std::vector<cv::DMatch> MatchTwoImage();
    std::vector<cv::DMatch> calculateDistance(const std::vector<cv::DMatch> &);

    int euclideanDistance(cv::Point2f &, cv::Point2f &);
    std::string calculateBoundingBox();
    std::string drawBox(const std::vector<cv::DMatch> &, cv::Mat &, std::string &);
    bool findIt(std::string &);
    bool isInsideOfBoundingBox(const int&);
};

#endif  // NOLINT
