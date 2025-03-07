/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#ifndef SELECTAREA_H  // NOLINT
#define SELECTAREA_H

#include <opencv/cv.h>
#include <opencv2/core/types_c.h>

#include <opencv2/tracking.hpp>  // for tracker
#include <opencv2/tracking/tracker.hpp>

#include <opencv2/video/background_segm.hpp>

#include <cstdlib>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <stdexcept>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <DepthEstimation/InsulateFrameWorks.hpp>
#include <DepthEstimation/Homography.hpp>
#include <DepthEstimation/ScrewDetectionPackets.hpp>
#include <DepthEstimation/NormalDistribution.hpp>

#include <RobotArm/RobotArmPackets.hpp>
// #include <SchunkArm/SchunkArmPackets.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <IPC/MMAP.hpp>
// #include "IPC/MMAP.hpp"

/**
 * @brief      Class to make use of area selected.
 */
class SelectArea {
 private:
    cv::Mat frame, camera_Matrix, distCoeffs;  // Current frame and camera calibration
    int focus;                                 // accountant to know the manual focus position
    int originWidthMarc, originHeightMarc, widthMarc, heightMarc;
    int theRadiusCenter = 0, negativeFrames = 11, countErrors;
    int quadrants[4];                // to calculate the orientation of the screw per quadrants
    int thresholdSquareError = 110;  // threshold to calculate the correct perform of the Rois
    int numberOfEstimates;  // Start the normal distribution & take care about the Roi's behaviour
    double initSharpness = -1;  // To compare with the current sharpness on the autonomous focus
    cv::Point theScrew, previousScrew;  // The center of the screw found
    cv::Point centerCoordinates;        // The intersection point of a square/trapezoid (usually
                                        // the main Roi) useful for the homography (trapezoid)
    bool choosedArea = false, screwFound = false, resizeMark = false;
    bool maximumFocus = false, enableFocus = false, firstCircle = false, wantToFollowIt = false;
    bool patternFromBB = false, existPattern = false, newPattern = false;
    cv::Rect_<double> pattern_corners;           // Bounding box of the current pattern, it used
                                                 // just to show the original position of pattern
    Eigen::Matrix4f originMatrix,                // Matrix of the robot from the pattern frame
                    currentMatrix,               // Matrix of the robot from the current frame
                    transformationMatrix;        // inverse of originMatrix * currentMatrix
    std::vector<cv::Point2f> matchesFrame;       // matches coordenates from the current frame
    std::vector<cv::Point> matchesPattern;       // matches coordenates from the pattern
    std::vector<bool> avoidRoiToDepthStimation;  // benchmarks available for the depth-estimation
    double leftX, rightX, depthTarget;  // Used by SURF-based Depth-Estimation algorithm, average
                                        // euclidean distance between the matches (deprecated)
    cv::Point originRoi;    // Used on myHomography to fix the patterns' feature-points when
                            // the pattern is given by an interest area instead of a picture loaded
    std::string coordinatesBoundingBox;          // necessary to call myHomography.runHomography
    cv::Ptr<cv::Tracker> tracker[5];             // vector of trackers (one per each roi)
    cv::Rect_<double> roi[5];                    // vector of Rois used to Depth-Estimation

    InsulateFrameWorks myInsulate;               // to find objects withing the interest area
    Homography myHomography;                     // homography, SURF, tracking recover, etc
    NormalDistribution myNormalDistribution;     // to decide when the depth is estimated

    /**
     * @brief      autonomous focus.
     *             Ussing the Sobel's border map from InsulateFrameWork.hpp, pixels average gives the sharpness.
     */
    void focusFrame();

    /**
     * @brief      comparing sharpness to improve it
     * @param[in]  currentSharpness  pixels average got by the Sobel's border map
     */
    void compareSharpness(double currentSharpness);

    /**
     * @brief      This function takes care the whole interest area be on the scene.
     * @param      ptLeftUp     The point left up
     * @param      ptRightDown  The point right down
     */
    void correctPositionPoints(cv::Point &ptLeftUp, cv::Point &ptRightDown);  // NOLINT

    /**
     * @brief      Call a calculateRoi from the origin position, set the choosedArea on true and set the origin position on myHomography.
     * @param      center     Area of interest's center point
     * @param      centerRoi  Area of interest's ROI
     */
    void createRoiFromCenter(const cv::Point &center, cv::Rect_<double> &centerRoi);  // NOLINT

    /**
     * @brief      Calculates the main roi, create the Tracker and start it.
     * @param      ptLeftUp     The point left up
     * @param      ptRightDown  The point right down
     */
    void calculateRoi(const cv::Point &ptLeftUp, const cv::Point &ptRightDown);  // NOLINT

    /**
     * @brief      Calculates the insulated rois from the main roi's dimension, creat the trackers 
     *             for each, start them and, keeps the origin position rois as a reference in order
     *             to take care about their correct behavior on the checkRoiStatus method.
     * @param[in]  ptLeftUp     The point left up from the main roi
     * @param[in]  ptRightDown  The point right down from the main roi
     */
    void calculateInsulateRois(const cv::Point &ptLeftUp, const cv::Point &ptRightDown);

    /**
     * @brief      { function_description }
     * @return     { description_of_the_return_value }
     */
    Eigen::MatrixXf updateTracker();

    /**
     * @brief      { function_description }
     */
    void checkRoiStatus();

    /**
     * @brief      { function_description }
     * @param[in]  currentPoint  The current point
     * @return     { description_of_the_return_value }
     */
    cv::Point updatePositionCircle(const cv::Point &currentPoint);

    /**
     * @brief      { function_description }
     * @param      circles  The circles
     * @return     { description_of_the_return_value }
     */
    bool compareMeasures(cv::Vec3f &circles);  // NOLINT

    /**
     * @brief      { function_description }
     */
    void cameraDesviation();

    /**
     * @brief      { function_description }
     * @return     { description_of_the_return_value }
     */
    int compareQuadrants();

    /**
     * @brief      { function_description }
     * @return     { description_of_the_return_value }
     */
    int maximumQuadrant();

    /**
     * @brief      { function_description }
     */
    void printQuadrants();

    /**
     * @brief      { function_description }
     */
    void printMatrix();

    /**
     * @brief      { function_description }
     * @param[in]  option  The option
     */
    void restartTheTracker(bool option = false);

    /**
     * @brief      Calculates the rotations.
     * @param      rotInX  The rot in x
     * @param      rotInY  The rot in y
     * @param      rotInZ  The rot in z
     * @param[in]  opt     The option, is there transformatrix?
     * @param[in]  opt2    The option 2, current matrix = true & origin matrix = false
     * @return     The rotations.
     */
    Eigen::Matrix4f calculateRotations(
        double &rotInX, double &rotInY, double &rotInZ, bool opt, bool opt2 = false);  // NOLINT

    /**
     * @brief       keeping mind the translation when there is rotation (rotation arc), regarding to the the camera position (7cm = 0.07) respect to the end-effector without extrinsic calibration
     * @param       translationX translation in X made by the robot
     * @param       translationY translation in Y made by the robot
     * @param       rotInX       rotation in X axys made by the robot
     * @param       rotInY       rotation in Y axys made by the robot
     * @return boolean to confirm
     */
    bool translationWithoutCalibration(
        double &translationX, double &translationY, const double rotInX, const double rotInY);  // NOLINT

    bool filteringOutliers(std::vector<double>& distanceX,  // NOLINT
        std::vector<double>& distanceY, const double average_distX, const double average_distY);  // NOLINT


    cv::Point pointPatternProjection(const int);
    cv::Point pointFrameProjection(const int);
    float triangulationBySURF(const cv::Point &, const cv::Point &);
    double distanceInXByPoint(const double&, const cv::Point &, const cv::Point &, const double &);
    float triangulatePosition(const int &, const cv::Point &, const cv::Point &,
        float &, float &, const double &, const double &);
    double distanceInYByPoint
    (const double &, const double &, const cv::Point &, const cv::Point &, double, const double &);
    void benchmarkCorrelation(const int, cv::Point &, cv::Point &);
    int16_t numberOfBenchmarks(const std::vector<double> &);

    /**
     * @brief      Calculates the distance.
     * @return     The distance.
     */
    float calculateDistance();
    void printDepthEstimation();

    /**
     * @brief      Removes bad points.
     *             Ussing the distance average of the matches in X axis, the method remove the matches with bigest distance
     *             in order to encrease the accuracy since it is working with metallic pieces and althoug the matches will
     *             be on the bounding box these could appeare in a wrong position.
     */
    void removeBadPoints();

    /**
     * @brief      { function_description }
     * @param[in]  o1    The o 1
     * @param[in]  p1    The p 1
     * @param[in]  o2    The o 2
     * @param[in]  p2    The p 2
     * @param[out] r     { parameter_description }
     * @return     { description_of_the_return_value }
     */
    cv::Point intersection
    (const cv::Point2f &o1, const cv::Point2f &p1, const cv::Point2f &o2, const cv::Point2f &p2);

 public:
    /**
     * @brief      { function_description }
     */
    SelectArea();

    /**
     * @brief      Destroys the object.
     */
    ~SelectArea() {}

    /**
     * @brief      { function_description }
     * @param      robot_pose  The robot pose
     * @return     { description_of_the_return_value }
     */
    Eigen::MatrixXf searchTheScrew(const float robot_pose[4][4]);

    void setFrame(const cv::Mat &);

    /**
     * @param
     */
    void setCameraParameters(const char* fileName);

    /**
     * @brief      Sets the from scratch.
     *
     * @param[in]  percentageReduction  The percentage reduction
     * @param[in]  cols                 The cols
     * @param[in]  rows                 The rows
     * @param[out] center               The center
     * @param[out] centerRoi            The center of a Roi when the are is chosen by drag&drop
     */
    void setFromScratch(int percentageReduction, int cols, int rows,
        const cv::Point &center, cv::Rect_<double> &centerRoi);  // NOLINT

    /**
     * @brief      Sets the threshold error.
     * @param[in]  n     { parameter_description }
     */
    inline  void setThresholdError(const int n) { this->thresholdSquareError = n; }

    /**
     * @brief      Sets the @p choosedArea.
     * @param[in]  ch    { parameter_description }
     */
    inline void setChoosedArea(const bool ch) { this->choosedArea = ch; }

    /**
     * @brief      Sets the exist pattern.
     * @param[in]  ch    { parameter_description }
     * @return     { description_of_the_return_value }
     */
    inline bool setExistPattern(const bool ch) { this->existPattern = ch; }

    /**
     * @brief      Gets the frame.
     * @return     The frame.
     */
    inline cv::Mat getFrame() const { return this->frame; }

    /**
     * @brief      Gets the choosed area.
     * @return     The choosed area.
     */
    inline bool getChoosedArea() const { return this->choosedArea; }

    /**
     * @brief      Gets the exist pattern.
     * @return     The exist pattern.
     */
    inline bool getExistPattern() const { return this->existPattern; }

    /**
     * @brief      Gets the follow iterator.
     * @return     The follow iterator.
     */
    inline bool getFollowIt() const { return this->wantToFollowIt; }

    /**
     * @brief      Gets the screw found.
     * @return     The screw found.
     */
    inline bool getScrewFound() const { return this->screwFound; }

    /**
     * @brief      Gets the center coordinates.
     * @return     The center coordinates.
     */
    inline cv::Point getCenterCoordinates() const {
        return cv::Point(this->choosedArea ? this->roi[0].x + ( this->roi[0].width /2 ) :
            0, this->choosedArea ? this->roi[0].y + (this->roi[0].height/2) : 0);
    }

    /**
     * @brief      Gets the depth target.
     * @return     The depth target.
     */
    inline double getDepthTarget() const { return this->depthTarget; }

    /**
     * @brief      Gets the roi area.
     * @return     The roi area.
     */
    inline int getRoiArea() const { return this->roi[0].area(); }

    /**
     * @brief      Gets the camera matrix.
     * @return     The camera matrix.
     */
    inline cv::Mat getCameraMatrix() const { return this->camera_Matrix; }

    /**
     * @brief      Gets the distance coeffs.
     * @return     The distance coeffs.
     */
    inline cv::Mat getDistCoeffs() const { return this->distCoeffs; }

    /**
     * @brief      Gets the canny frame.
     * @return     The canny frame.
     */
    inline cv::Mat getCannyFrame() { return this->myInsulate.getCannyFrame(); }

    /**
     * @brief      { function_description }
     */
    void publish_on_the_GUI();

    /**
     * @brief      { function_description }
     */
    void manualFocusIsDisabled();

    /**
     * @brief      Function for Aumented Reallity, draws the center of the screne marcs and the error among the center scene and the main Roi's center.
     * @param[in]  scenebox     The scenebox
     * @param[in]  boundingBox  The bounding box
     * @return     { description_of_the_return_value }
     */
    Eigen::MatrixXf printInformation(bool scenebox = false, bool boundingBox = false);

    /**
     * @brief      Function for Aumented Reallity.
     *             Draws the main roi and its insulates ones.
     * @param[out] frame  The frame
     */
    void drawRois(cv::Mat &frame);  // NOLINT

    /**
     * @brief      Function to run the Homography's runHomography method.
     *             Calls to homography main function, if the errors 
     *             are more than 5 the system change the patern.
     * @param[out] frame       The frame
     * @param[in]  robot_pose  The current robot pose
     */
    void runHomography(cv::Mat &frame, const float robot_pose[4][4]);  // NOLINT
};

#endif  // NOLINT
