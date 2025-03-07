#include "InsulateFrameWorks/InsulateFrameWorks.hpp"

#define _depth CV_16S
#define _aument 25			//aument the bounding box
#define _thresh 350
#define thresh 100
#define _maxKeyPoints 500
#define _cannyMask 3
//#define _maxKeyPoints 200

cv::RNG rng(12345);

InsulateFrameWorks::InsulateFrameWorks() : scale(1), delta(0), equivalenceLeft(0.05) {}
InsulateFrameWorks::InsulateFrameWorks(double aDevice, double aLeftSticker, double aRightSticker) : scale(1), delta(0)
{
	this->equivalenceLeft = this->equivalenceCalculate(aDevice, aLeftSticker) ;
	this->equivalenceRight = this->equivalenceCalculate(aDevice, aRightSticker) ;
}

InsulateFrameWorks::InsulateFrameWorks(cv::Mat &frame, int scale = 1, int delta = 0) : scale(scale), delta(delta) 
{
	cv::cvtColor(frame , this->frame, CV_BGR2GRAY); 
	cv::threshold(this->frame, this->binaryFrame, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
}

double InsulateFrameWorks::equivalenceCalculate(double aDevice, double aSticker)
{
	return aSticker/aDevice;
}

/**
*Applying SOBEL algorithm to get the edges map by each image_1
**/
void InsulateFrameWorks::edgesSobel(const bool opt = false)
{   
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    cv::GaussianBlur( this->frame, this->frame, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

    //gradient X
    cv::Sobel(!opt ? this->frame : this->binaryFrame, grad_x, _depth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    //gradient Y
    cv::Sobel(!opt ? this->frame : this->binaryFrame, grad_y, _depth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
   	cv::convertScaleAbs(grad_y, abs_grad_y);

    cv::addWeighted (abs_grad_x, 0.5, abs_grad_y, 0.5, 0, !opt ? this->sobelFrame : this->sobelBinaryFrame);
    //cv::addWeighted (abs_grad_x, 0.1, abs_grad_y, 0.8, 0, this->frame);
}

void InsulateFrameWorks::edgesCanny(const bool opt = false)
{
	//cv::GaussianBlur(!opt ? this->frame : this->binaryFrame, !opt ? this->frame : this->binaryFrame, cv::Size(7,7), 1.5, 1.5);
    cv::Canny(!opt ? this->frame : this->halfFrame, this->cannyFrame, thresh / 2, /*30*//*255*/thresh * 2);//_cannyMask);
}

void InsulateFrameWorks::edgesLaplacian(const bool opt = false)
{
	cv::Mat laplacianPiece;

	cv::GaussianBlur(!opt ? this->frame : this->binaryFrame, !opt ? this->frame : this->binaryFrame, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
    cv::Laplacian(!opt ? this->frame : this->binaryFrame, laplacianPiece, /*int depth*/ CV_16S, /*int ksize(mask)*/ 3, /*double scale*/ 1, /*double delta*/ 0, /*int borderType*/ cv::BORDER_DEFAULT );
    cv::convertScaleAbs(laplacianPiece, this->laplacianFrame);
}

void InsulateFrameWorks::setFrame(const cv::Mat frame) 
{ 
	this->frame = frame.clone();	
	//cv::cvtColor(frame , this->frame, CV_BGR2GRAY);
	//cv::threshold(this->frame, this->binaryFrame, 200, 255, CV_THRESH_BINARY);				// | CV_THRESH_OTSU); 
}

void InsulateFrameWorks::setHalfFrame(const cv::Mat frame)
{
	this->halfFrame = frame.clone();
}

cv::Mat InsulateFrameWorks::getFrame() const { return this->frame; }
cv::Mat InsulateFrameWorks::getHalfFrame() const { return this->halfFrame; }
cv::Mat InsulateFrameWorks::getSobelFrame() const { return this->sobelFrame; }
cv::Mat InsulateFrameWorks::getSobelBinaryFrame() const { return this->sobelBinaryFrame; }
cv::Mat InsulateFrameWorks::getCannyFrame() const { return this->cannyFrame; }
cv::Mat InsulateFrameWorks::getCannyBinaryFrame() const { return this->cannyBinaryFrame; }
cv::Mat InsulateFrameWorks::getLaplacianFrame() const { return this->laplacianFrame; }
cv::Mat InsulateFrameWorks::getDilation() const { return this->dilation; }
cv::Mat InsulateFrameWorks::getErosion() const { return this->erosion; }
cv::Mat InsulateFrameWorks::getClosing() const { return this->closing; }
cv::Mat InsulateFrameWorks::getOpening() const { return this->opening; }
float InsulateFrameWorks::getEquivalence(const bool opt) const 
{ 
	if(!opt) return this->equivalenceLeft;
	return this->equivalenceRight;
}

void InsulateFrameWorks::isBiggest(int &maximumArea, int &position, int contourArea, int index)
{
	if (maximumArea < contourArea)
	{
		maximumArea = contourArea;
		position = index;
	}
}

void InsulateFrameWorks::drawBiggestContour(const std::vector< std::vector<cv::Point> > &contours, int index, const std::vector<cv::Vec4i> hierarchy, const bool opt = false)
{
  	cv::Scalar color = CV_RGB(0, 255, 0);
   	cv::drawContours( !opt ? this->frame : this->halfFrame, contours, index, color, 2, 8, hierarchy, 0, cv::Point() );
}

/** @function thresh_callback */
std::vector<cv::Point> InsulateFrameWorks::thresh_callback(const bool opt = false)
{
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	/// Find contours
	findContours( this->cannyFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	std::vector<cv::Point> approx;
	int maximumArea = -1, position = 0;

	for( int i = 0; i< contours.size(); i++ )
	{
		// reducir el nuemro de puntos usando del algoritmo de Douglas-Peucker
		cv::approxPolyDP(cv::Mat(contours[i]), approx, /*EPSILON*/ cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
		if (approx.size() == 4) isBiggest(maximumArea, position, cv::contourArea(contours[i]), i);		//if it finds a mark
	}

	drawBiggestContour(contours, position, hierarchy, opt);
  	
  	return contours[position];
}

void InsulateFrameWorks::morphoTransform()
{
	int erosion_size = 2.5;   
	cv::Mat element = cv::getStructuringElement(/*cv::MORPH_CROSS,*/cv::MORPH_RECT,
                      cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
                      cv::Point(erosion_size, erosion_size) );

	//cv::dilate(this->binaryFrame, this->binaryFrame, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	cv::dilate(this->cannyFrame, this->dilation, element);		// cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	cv::erode(this->cannyFrame, this->erosion, element);

  	cv::morphologyEx( this->cannyFrame, this->closing, cv::MORPH_CLOSE, element );
  	cv::morphologyEx( this->cannyFrame, this->opening, cv::MORPH_OPEN, element );
	//cv::closing(this->frame, this->closing, element);
	/*this->erosion =
	this->opening =
	this->closing =*/
}

/* @function cornerHarris_demo */
void InsulateFrameWorks::cornerHarrisDetection()	//int, void* )
{
    cv::Mat dst, dst_norm;
    std::vector<cv::KeyPoint> keyPoints;
    dst = cv::Mat::zeros( this->binaryFrame.size(), CV_32FC1 );
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    cv::cornerHarris( this->binaryFrame, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv:;convertScaleAbs( dst_norm, this->binaryFrame );
    for( int j = 0; j < dst_norm.rows ; j++ )
    {
    	for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                cv::circle( this->binaryFrame, cv::Point( i, j ), 5,  cv::Scalar(0), 2, 8, 0 );
                keyPoints.push_back(cv::KeyPoint(i, j, 1));
            }
        }
        if ( keyPoints.size() > _maxKeyPoints) 
        {
            keyPoints.clear();
            break;
        }
    }

    if (!keyPoints.empty()) DrawBox(keyPoints);
}

void InsulateFrameWorks::DrawBox(std::vector<cv::KeyPoint> &matches)
{
	std::vector<cv::Point2f> obj, scene;
//std::cout << "AKI NO PETA??? --> " << matches.size() << std::endl;
    /*for (int i = 0; i < matches.size(); i++)
    {
        //obj.push_back(matches[i].queryIdx.pt);
        scene.push_back(matches[i].trainIdx.pt);
    }

    cv::Mat H = findHomography(obj, scene, CV_RANSAC);
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); 
    obj_corners[1] = cvPoint( this->patterns[index].cols, 0 );
    obj_corners[2] = cvPoint( this->patterns[index].cols, this->patterns[index].rows ); 
    obj_corners[3] = cvPoint( 0, this->patterns[index].rows );
    std::vector<cv::Point2f> scene_corners(4);*/
}

InsulateFrameWorks::~InsulateFrameWorks() {}
