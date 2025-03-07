#include <iostream>
#include <vector>
#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"    //for cvtColor
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"


/*!
  A more elaborate class description.
*/
class InsulateFrameWorks
{
  private:
	
	std::vector<cv::KeyPoint>* areaDevice, areaSticker;
    cv::Mat frame, halfFrame, binaryFrame, sobelFrame, sobelBinaryFrame, cannyFrame, cannyBinaryFrame, laplacianFrame, dilation, erosion, opening, closing;
    float equivalenceLeft, equivalenceRight;
	int scale, delta;

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
	void drawBiggestContour(const std::vector< std::vector<cv::Point> > &, int, const std::vector<cv::Vec4i>, const bool);

	cv::Mat getFrame() const;
	cv::Mat getHalfFrame() const;
	cv::Mat getSobelFrame() const;
	cv::Mat getSobelBinaryFrame() const;
	cv::Mat getCannyFrame() const;
	cv::Mat getCannyBinaryFrame() const;
	cv::Mat getLaplacianFrame() const;
	cv::Mat getDilation() const;
	cv::Mat getErosion() const;
	cv::Mat getClosing() const;
	cv::Mat getOpening() const;
	float getEquivalence(const bool) const;

	std::vector<cv::Point> thresh_callback(const bool);
	void cornerHarrisDetection();	// int, void* );
	void DrawBox(std::vector<cv::KeyPoint> &);
	void morphoTransform();

	~InsulateFrameWorks();
};