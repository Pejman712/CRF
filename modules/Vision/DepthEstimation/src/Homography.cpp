/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <DepthEstimation/Homography.hpp>
#include <string>
#include <vector>

#define _depth CV_16S
#define _aument 1

Homography::Homography() : scale_(1), delta_(0), numPatterns_(1) {
    for (int i = 0; i < 4; i++) this->scene_corners_.push_back(cv::Point2f());
}

void Homography::setPattern(const cv::Mat &pattern) {
    this->pattern_ = pattern.clone();
    this->processPattern(true);

    // cv::imshow("PATTERN", this->pattern);
}

/**
*Applying SOBEL algorithm to get the edges map by each image
*param type boolean argument to indicate the pattern or the frame scene
**/
void Homography::edgesMap(bool type) {
    cv::Mat grayPattern;
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    cv::cvtColor((type == true ? this->pattern_ : this->frame_), grayPattern, CV_BGR2GRAY);
    // gradient X
    cv::Sobel(grayPattern, grad_x, _depth, 1, 0, 3, scale_, delta_, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    // gradient Y
    cv::Sobel(grayPattern, grad_y, _depth, 1, 0, 3, scale_, delta_, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, type == true ? pattern_ : frame_);
}


void Homography::setoriginRoiPoint(const cv::Point &op) {
    this->originRoi_.x = op.x;
    this->originRoi_.y = op.y;
}


/**
*Processing of each pattern just after create the instance of object
**/
void Homography::processPattern(bool type) {
    this->edgesMap(type);
    this->findFeatures(type);
}

/**
*findFeatures for each pattern
*param type
**/
void Homography::findFeatures(bool type) {
    if (type) {
        this->keyPointsPattern_ = DetectKeyPoints(type);
        this->descriptorsPattern_ = ComputeDescriptors(type);
    } else {
        this->keyPointsFrame_ = DetectKeyPoints(type);
        this->descriptorFrame_ = ComputeDescriptors(type);
    }
}

/**
*Detect key points for each images (patterns and frames)
*param type indicates if the image is a frame of scene or a pattern
**/
std::vector<cv::KeyPoint> Homography::DetectKeyPoints(bool type) {
    // auto featureDetector =
            // cv::ORB::create(1000, 1.05f, 2, 31, 0, 4, cv::ORB::HARRIS_SCORE, 5, 130);
    auto featureDetector = cv::xfeatures2d::SURF::create(1000);
    std::vector<cv::KeyPoint> keyPoints;
    featureDetector->detect(type == true ? this->pattern_ : this->frame_, keyPoints);

    return keyPoints;
}

/**
*Extracts descriptors from each images (patterns and frames)
*param type indicates if the image is a frame of scene or a pattern
**/
cv::Mat Homography::ComputeDescriptors(bool type) {
    type ? this->keyPointsFrame_.clear() : this->keyPointsPattern_.clear();
    cv::Mat descriptors;
    try {
        // cv::Ptr<cv::DescriptorExtractor> featureExtractor =
                // cv::xfeatures2d::BriefDescriptorExtractor::create();  // ORB");
        auto featureExtractor = cv::xfeatures2d::SurfDescriptorExtractor::create();  // ORB");
        featureExtractor->compute(type ? this->pattern_ : this->frame_,
            type ? this->keyPointsPattern_ : this->keyPointsFrame_, descriptors);
    } catch(cv::Exception e) { std::cerr << e.what() << std::endl; }

    return descriptors;
}

/**
*Calculates the keyPoints distance between pattern and frame scene
**/
std::vector<cv::DMatch> Homography::calculateDistance(const std::vector<cv::DMatch> &matches) {
    std::vector<cv::DMatch> goodMatches;

    double max_dist = 0, min_dist = 100;

    for (int i = 0; i < this->descriptorsPattern_.rows; i++) {
        if (matches[i].distance < min_dist) min_dist = matches[i].distance;
        if (matches[i].distance > max_dist) max_dist = matches[i].distance;
    }
    // for HARRIS on OpenCV 2.8
    for (int i = 0; i < this->descriptorsPattern_.rows; i++)
        if (matches[i].distance < (/*2.8*/3.6 * min_dist)) goodMatches.push_back(matches[i]);

    return goodMatches;
}

std::vector<cv::DMatch> Homography::MatchTwoImage() {
    auto matcher = cv::DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> matches;
    matcher->match(this->descriptorsPattern_, this->descriptorFrame_, matches);

    auto goodMatches = calculateDistance(matches);

    return goodMatches;
}

/** to calculate the homography */
double Homography::targetDistance() {
    cv::Mat transpose_camera_matrix = this->camera_Matrix_.t();  // transpose

    auto fundamental_matrix = cv::findFundamentalMat(obj_, scene_, CV_FM_RANSAC, 3, 0.99);
    auto esential_matrix = transpose_camera_matrix * fundamental_matrix * this->camera_Matrix_;
    std::cout << "TARGET DISTANCE FROM HOMOGRAPHY!?" << std::endl;
    // cv::Mat exitMat;
    // auto esential_matrix2 = cv::findEssentialMat(this->patternPoints, this->scenePoints,
                                            // cameraMatrix, CV_FM_RANSAC, 0.999, 1.0, exitMat);
    cv::SVD svd(esential_matrix);
    // cv::SVD svd2(esential_matrix2);
    cv::Matx33d W(0, -1, 0,   // HZ 9.13
                  1, 0, 0,
                  0, 0, 1);

    cv::Mat_<double> R = svd.u * cv::Mat(W).inv() * svd.vt;  // HZ 9.19

    cv::Matx33d Z(0, -1, 0,
                  1, 0, 0,
                  0, 0, 0);
    cv::Mat_<double> t = svd.vt.t() * cv::Mat(Z) * svd.vt;

    // -->> Xl is the X coordenate on the left image from 0
    // -->> Xr is the X coordenate on the right image from row.cols
    double distance = 0;  // (focalLength_ * translationOnXaxis) / (Xl - Xr)

    // cv::Mat_<double> t2 = svd2.vt.t() * cv::Mat(Z) * svd2.vt;

    /*std::cout << "Foundamental Matrix: " << fundamental_matrix << std::endl << std::endl;
    std::cout << "Esential Matrix: " << esential_matrix << std::endl;
    // std::cout << "Esential Matrix_2: " << esential_matrix2 << std::endl << std::endl;
    std::cout << "SVD_U: " << svd.u << std::endl;
    std::cout << "DET_U: " << cv::determinant(svd.u) << std::endl;
    // std::cout << "SVD_U_2: " << svd2.u << std::endl << std::endl;
    std::cout << "SVD_VT: " << svd.vt << std::endl;
    std::cout << "DET_VT: " << cv::determinant(svd.vt) << std::endl;
    // std::cout << "SVD_VT_2: " << svd2.vt << std::endl << std::endl;
    std::cout << "SVD_W: " << svd.w << std::endl;
    std::cout << "W: " << W << std::endl;
    // std::cout << "SVD_W_2: " << svd2.w << std::endl << std::endl;
    std::cout << "ROTATION: " << (cv::Mat)R << std::endl << std::endl;
    std::cout << "TRASLATION: " << (cv::Mat)t << std::endl;*/
    // std::cout << "TRASLATION_2: " << (cv::Mat)t2 << std::endl << std::endl << std::endl;
    return distance;
}

bool Homography::isInsideOfBoundingBox(const int &index) {
    if ( (this->keyPointsFrame_[index].pt.x < this->bounding_corners_[0].x) ||
         (this->keyPointsFrame_[index].pt.x > this->bounding_corners_[2].x) ||
         (this->keyPointsFrame_[index].pt.y < this->bounding_corners_[0].y) ||
         (this->keyPointsFrame_[index].pt.y > this->bounding_corners_[1].y) ) return false;

    return true;
}

bool Homography::runHomography(cv::Mat &scene, std::string &coordinatesBoundingBox) {  // NOLINT
    this->frame_ = scene.clone();    // setting frame on Homography object

    std::string coordinatesObject;

    if (!this->frame_.empty()) {
        this->edgesMap(false);
        this->findFeatures(false);          // keyPointsFrame_, descriptorFrame_);

        if (this->keyPointsFrame_.size() >= 4 && this->descriptorFrame_.rows >= 10) {
            this->matches_ = MatchTwoImage();  // this->descriptorFrame_);

            if ( this->matches_.size() > 4 ) {
                coordinatesBoundingBox = drawBox(this->matches_, scene, coordinatesObject);
                if ( this->findIt(coordinatesObject) ) return true;
            }
        }
    } else { std::cerr << "ERROR: Camera do not work" << std::endl; }

    return false;
}

int Homography::euclideanDistance(cv::Point2f &point1, cv::Point2f &point2) {  // NOLINT
    double resultado = pow((static_cast<int>(point1.x) - static_cast<int>(point2.x)), 2);
    resultado += pow((static_cast<int>(point1.y) - static_cast<int>(point2.y)), 2);

    return sqrt(resultado);
}

std::vector<int> Homography::split(const std::string &s, const char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<int> tokens;

    while (getline(ss, item, delim)) tokens.push_back(std::stoi(item));

    return tokens;
}

bool Homography::findIt(std::string &coordenates) {  // NOLINT
    auto numCoordenates = split(coordenates, ' ');

    if ((numCoordenates[2] <= (this->pattern_.cols * 1.2)
        && numCoordenates[2] >= (this->pattern_.cols * 0.9))
        && (numCoordenates[3] <= (this->pattern_.rows * 1.35)
        && numCoordenates[3] >= (this->pattern_.rows * 0.9))) {
        // std::cout << "It is POSITIVE" << std::endl;
        return true;
    }
    // std::cout << "It is NEGATIVE" << std::endl;
    return false;
}

void Homography::filterMatches(double &patternAverageX, double &frameAverageX) {  // NOLINT
    double average = 0, cant = 0;
    for (int i = 0; i < this->matches_.size(); i++) average += this->matches_[i].distance;
    average = average / this->matches_.size();

    for (int i = 0; i < this->matches_.size(); i++)
        if (isInsideOfBoundingBox(matches_[i].trainIdx) && matches_[i].distance < average) {
           goodPointsPattern_.push_back(this->keyPointsPattern_[ this->matches_[i].queryIdx ].pt);
           goodPointsFrame_.push_back(this->keyPointsFrame_[ this->matches_[i].trainIdx ].pt);

           patternAverageX  += this->keyPointsPattern_[ this->matches_[i].queryIdx ].pt.x;
           frameAverageX += this->keyPointsFrame_[ this->matches_[i].trainIdx ].pt.x;

           cant++;
        }
    if (cant > 0) {
        patternAverageX  /= cant;
        frameAverageX /= cant;
    } else { patternAverageX = frameAverageX = -1; }
}

std::string Homography::calculateBoundingBox() {
    this->bounding_corners_.push_back(cv::Point2f(
                            (this->scene_corners_[0].x < this->scene_corners_[3].x
                              ? this->scene_corners_[0].x : this->scene_corners_[3].x) - _aument,
                            (this->scene_corners_[0].y < this->scene_corners_[1].y
                              ? this->scene_corners_[0].y : this->scene_corners_[1].y) - _aument));
    this->bounding_corners_.push_back(cv::Point2f(
                            (this->scene_corners_[0].x < this->scene_corners_[3].x
                              ? this->scene_corners_[0].x : this->scene_corners_[3].x) - _aument,
                            (this->scene_corners_[3].y > this->scene_corners_[2].y
                              ? this->scene_corners_[3].y : this->scene_corners_[2].y) + _aument));
    this->bounding_corners_.push_back(cv::Point2f(
                            (this->scene_corners_[1].x > this->scene_corners_[2].x
                              ? this->scene_corners_[1].x : this->scene_corners_[2].x) + _aument,
                            (this->scene_corners_[0].y < this->scene_corners_[1].y
                              ? this->scene_corners_[0].y : this->scene_corners_[1].y) - _aument));
    this->bounding_corners_.push_back(cv::Point2f(
                            (this->scene_corners_[1].x > this->scene_corners_[2].x
                              ? this->scene_corners_[1].x : this->scene_corners_[2].x) + _aument,
                            (this->scene_corners_[3].y > this->scene_corners_[2].y
                              ? this->scene_corners_[3].y : this->scene_corners_[2].y) + _aument));

    return std::to_string(static_cast<int>(this->scene_corners_[0].x)) + " "
         + std::to_string(static_cast<int>(this->scene_corners_[0].y)) + " "
         + std::to_string((euclideanDistance(this->scene_corners_[0], this->scene_corners_[1]) < 20
                ? -1 : euclideanDistance(this->scene_corners_[0], this->scene_corners_[1]))) + " "
         + std::to_string((euclideanDistance(this->scene_corners_[0], this->scene_corners_[3]) < 20
                ? -1 : euclideanDistance(this->scene_corners_[0], this->scene_corners_[3])));
}

std::string Homography::drawBox
(const std::vector<cv::DMatch> &matches, cv::Mat &frame, std::string &coordinatesObject) {  // NOLINT
    this->obj_.clear();
    this->scene_.clear();
    for (int i = 0; i < matches.size(); i++) {
        // pattern matches
        this->obj_.push_back((cv::Point)this->keyPointsPattern_[matches[i].queryIdx].pt);
        // scene matches
        this->scene_.push_back(this->keyPointsFrame_[matches[i].trainIdx].pt);
    }
    cv::Mat H = cv::findHomography(this->obj_, this->scene_, CV_RANSAC);

    // Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0, 0);
    obj_corners[1] = cvPoint(this->pattern_.cols, 0);
    obj_corners[2] = cvPoint(this->pattern_.cols, this->pattern_.rows);
    obj_corners[3] = cvPoint(0, this->pattern_.rows);
    std::vector<cv::Point2f> scene_corners(4);

    try {
        perspectiveTransform(obj_corners, scene_corners, H);
        for (int i = 0; i < 4; i++) {
            this->scene_corners_[i].x = scene_corners[i].x;
            this->scene_corners_[i].y = scene_corners[i].y;
        }
    } catch(cv::Exception e) {
        std::cerr << e.what()
        << "On the execution time in Homography::drawBox function, Homography.cpp" << std::endl;
    }

    /** CHANGING POINTS REGARDING SCENE INSTEAD OF SQUARE */
    for (int i = 0; i < this->obj_.size(); i++) this->obj_[i] += this->originRoi_;

    coordinatesObject = calculateBoundingBox();

    // CONTROL O POSITION FOR REFERENCE POINT
    if (this->bounding_corners_[0].x < 0) this->bounding_corners_[0].x = 0;
    if (this->bounding_corners_[0].y < 0) this->bounding_corners_[0].y = 0;

    return std::to_string(static_cast<int>(this->bounding_corners_[0].x) ) + " "
        + std::to_string(static_cast<int>(this->bounding_corners_[0].y) ) + " "
        + std::to_string(this->bounding_corners_[2].x < this->frame_.cols ?
            euclideanDistance(this->bounding_corners_[0], this->bounding_corners_[2])
            : static_cast<int>(this->frame_.cols - this->bounding_corners_[0].x) ) + " "
        + std::to_string(this->bounding_corners_[1].y < this->frame_.rows ?
            euclideanDistance(this->bounding_corners_[0], this->bounding_corners_[1])
            : static_cast<int>(this->frame_.rows - this->bounding_corners_[0].y) );
}

void Homography::drawSceneBox(cv::Mat &frame) {  // NOLINT
    // Draw lines between the corners (the mapped object in the scene - image_2 )
    line(frame, this->scene_corners_[0], this->scene_corners_[1], cv::Scalar(255, 0, 0), 4);
    line(frame, this->scene_corners_[1], this->scene_corners_[2], cv::Scalar(255, 255, 0), 4);
    line(frame, this->scene_corners_[2], this->scene_corners_[3], cv::Scalar(255, 0, 0), 4);
    line(frame, this->scene_corners_[3], this->scene_corners_[0], cv::Scalar(255, 0, 255), 4);
}

void Homography::drawBoundingBox(cv::Mat &frame) {  // NOLINT
    line(frame, this->bounding_corners_[0], this->bounding_corners_[2], cv::Scalar(0, 0, 255), 4);
    line(frame, this->bounding_corners_[2], this->bounding_corners_[3], cv::Scalar(0, 0, 255), 4);
    line(frame, this->bounding_corners_[3], this->bounding_corners_[1], cv::Scalar(0, 0, 255), 4);
    line(frame, this->bounding_corners_[0], this->bounding_corners_[1], cv::Scalar(0, 0, 255), 4);
}
