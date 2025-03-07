//#pragma once
#ifndef INSULATEFRAMEWORK_H
#define INSULATEFRAMEWORK_H

#include <iostream>
#include <vector>
#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"    	//for cvtColor
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"		//for webCam

#include <cmath>

/*!
  A more elaborate class description.
*/
class InsulateFrameWorks
{
  private:
	
	std::vector<cv::KeyPoint>* areaDevice, areaSticker;
    cv::Mat frame, halfFrame, binaryFrame, sobelFrame, sobelHalfFrame, sobelBinaryFrame, cannyFrame, cannyBinaryFrame, laplacianFrame, dilation, erosion, opening, closing;
    float equivalenceLeft, equivalenceRight;
	int scale, delta, radius;
	cv::Point center;

  public:

	InsulateFrameWorks();
	InsulateFrameWorks(double, double, double);
	InsulateFrameWorks(cv::Mat &, int, int);

	double equivalenceCalculate(double, double);
	void edgesSobel(const bool);
	void edgesCanny(const bool);
	void edgesLaplacian(const bool);
	void setFrame(const cv::Mat);	
	void setHalfFrame(const cv::Mat);
	void isBiggest(int &, int &, int, int);
	void drawBiggestContour(const std::vector< std::vector<cv::Point> > &, int, const std::vector<cv::Vec4i> &, const bool);
	void drawCircles(const std::vector<cv::Vec3f> &);

	cv::Mat getFrame() const;
	cv::Mat getHalfFrame() const;
	cv::Mat getSobelFrame() const;
	cv::Mat getSobelHalfFrame() const;
	cv::Mat getSobelBinaryFrame() const;
	cv::Mat getCannyFrame() const;
	cv::Mat getCannyBinaryFrame() const;
	cv::Mat getLaplacianFrame() const;
	cv::Mat getDilation() const;
	cv::Mat getErosion() const;
	cv::Mat getClosing() const;
	cv::Mat getOpening() const;
	float getEquivalence(const bool) const;
	cv::Point getCenterPoint() const;
	int getRadiusCenter() const;

	std::vector<cv::Point> findBiggestContour(const bool);
	cv::Vec3f findCircles();
	void cornerHarrisDetection();
	void morphoTransform();
	bool intersection(const cv::Point2f &, const cv::Point2f &, const cv::Point2f &, const cv::Point2f &, cv::Point &);

	~InsulateFrameWorks();
};

#endif
