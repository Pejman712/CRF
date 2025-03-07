#include <DepthEstimation/InsulateFrameWorks.hpp>

#define _depth      CV_16S
#define _aument     25          //aument the bounding box
#define _thresh     350
#define thresh      100
#define _maxKeyPoints   500
#define _cannyMask  3

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

/** @function edgesSobel
*Applying SOBEL algorithm to get the edges map by each image
*param opt indicates if we want work with a original frame or the HALF (distinct to original library) frame
**/
void InsulateFrameWorks::edgesSobel(const bool opt = false)
{   
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    cv::GaussianBlur( !opt ? this->frame : this->halfFrame, !opt ? this->frame : this->halfFrame, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

    //gradient X
    cv::Sobel(!opt ? this->frame : this->halfFrame, grad_x, _depth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    //gradient Y
    cv::Sobel(!opt ? this->frame : this->halfFrame, grad_y, _depth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    cv::addWeighted (abs_grad_x, 0.5, abs_grad_y, 0.5, 0, !opt ? this->sobelFrame : this->sobelHalfFrame);
    //cv::addWeighted (abs_grad_x, 0.1, abs_grad_y, 0.8, 0, this->frame);
}

/** @function edgesCanny
*Applying CANNY algorithm to get the edges map by each image
*param opt indicates if we want work with a original frame or half frame (bottom or one side)
**/
void InsulateFrameWorks::edgesCanny(const bool opt = false)
{
    //cv::GaussianBlur(!opt ? this->frame : this->binaryFrame, !opt ? this->frame : this->binaryFrame, cv::Size(7,7), 1.5, 1.5);
    cv::Canny(!opt ? this->frame : this->halfFrame, this->cannyFrame, thresh / 1.25, /*30*//*255*/thresh * 1.5);//_cannyMask);
}

/** @function edgesLaplacian
*Algorithm to get the edge map throught the laplacian 
*Laplacian measures changes in the gradient throught the second derivate
*This changes may be an edge
*param opt indicates if we want work with a original frame or the binarized frame
**/
void InsulateFrameWorks::edgesLaplacian(const bool opt = false)
{
    cv::Mat laplacianPiece;

    cv::GaussianBlur(!opt ? this->frame : this->binaryFrame, !opt ? this->frame : this->binaryFrame, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
    cv::Laplacian(!opt ? this->frame : this->binaryFrame, laplacianPiece, /*int depth*/ CV_16S, /*int ksize(mask)*/ 3, /*double scale*/ 1, /*double delta*/ 0, /*int borderType*/ cv::BORDER_DEFAULT );
    cv::convertScaleAbs(laplacianPiece, this->laplacianFrame);
}

void InsulateFrameWorks::setFrame(const cv::Mat frame) { this->frame = frame.clone(); }
void InsulateFrameWorks::setHalfFrame(const cv::Mat frame){ this->halfFrame = frame.clone(); }

cv::Mat InsulateFrameWorks::getFrame() const { return this->frame; }
cv::Mat InsulateFrameWorks::getHalfFrame() const { return this->halfFrame; }
cv::Mat InsulateFrameWorks::getSobelFrame() const { return this->sobelFrame; }
cv::Mat InsulateFrameWorks::getSobelHalfFrame() const { return this->sobelHalfFrame; }
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
cv::Point InsulateFrameWorks::getCenterPoint() const { return this->center; }
int InsulateFrameWorks::getRadiusCenter() const { return this->radius; }

/** @function isBiggest
*Compare if the new area is bigger than the rest
*param maximumArea size area from the biggest square
*param position index from largest square
*param contourArea size from new square area
*param index index from new square area
**/
void InsulateFrameWorks::isBiggest(int &maximumArea, int &position, int contourArea, int index)
{
    if (maximumArea < contourArea)
    {
        maximumArea = contourArea;
        position = index;
    }
}

/** @function drawBiggestContour
*Draw the biggest square found on frame
*param contours array with all found contours
*param index index to the biggest contour
*param hierarchy containing information about the image topology, it is compulsory to use this parameter in drawContours function
*param opt indicates if we want work with a original frame or half frame (bottom or one side)
**/
void InsulateFrameWorks::drawBiggestContour(const std::vector< std::vector<cv::Point> > &contours, int index, const std::vector<cv::Vec4i> &hierarchy, const bool opt = false)
{
    cv::Scalar color = CV_RGB(0, 255, 0);
    cv::drawContours( !opt ? this->frame : this->halfFrame, contours, index, color, /*thickness*/ 2, /*lineType*/ 8, hierarchy, /*maxLevel*/ 0, cv::Point() );  //maxLevel = 0 only the specified contour is drawn
}

/** @function findBiggestContour
*Find the square contours and returns the biggest one, it uses the edgemap from canny algorithm
*param opt indicates to @drawBiggestContour which frame we want to use
**/
std::vector<cv::Point> InsulateFrameWorks::findBiggestContour(const bool opt = false)
{
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    /// Find contours
    findContours( this->cannyFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    std::vector<cv::Point> approx;
    int maximumArea = -1, position = 0;

    for( int i = 0; i< contours.size(); i++ )
    {
        // reduce the points number using Douglas-Peucker algorithm
        cv::approxPolyDP(cv::Mat(contours[i]), approx, /*EPSILON*/ cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
        if (approx.size() == 4) isBiggest(maximumArea, position, cv::contourArea(contours[i]), i);      //if it finds a mark
    }

    drawBiggestContour(contours, position, hierarchy, opt);
    
    return contours[position];
}

/** @function drawCircles
*It draws circles in the original frame
*param circles has an array with all circles found in frame
**/
void InsulateFrameWorks::drawCircles(const std::vector<cv::Vec3f> &circles)
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
        //cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        this->center.x = cvRound(circles[i][0]);
        this->center.y = cvRound(circles[i][1]);
        this->radius = cvRound(circles[i][2]);
        // circle center
        cv::circle( this->halfFrame, this->center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( this->halfFrame, this->center, this->radius, cv::Scalar(0,0,255), 3, 8, 0 );
     }
}

/** @function findCircles
*
**/
cv::Vec3f InsulateFrameWorks::findCircles()
{
    cv::Mat grayHalfFrame;
    cv::cvtColor(this->halfFrame, grayHalfFrame, CV_BGR2GRAY);
    
    this->edgesCanny(true);
    
    std::vector<cv::Vec3f> circles;
    /** This method use internal Canny */
    cv::HoughCircles(grayHalfFrame, circles, CV_HOUGH_GRADIENT, 
        /*The inverse ratio of resolution*/             1, 
        /*Minimum distance between detected centers*/   grayHalfFrame.rows / 8, 
        /*Upper threshold for Canny*/                   150, 
        /*Threshold for center detection*/              50, 
        /*Minimum radio*/                               5, 
        /*Maximum radio*/                               50);

    drawCircles(circles);
    if (circles.size() == 0) return cv::Vec3f(-1, -1, -1);
    uint index = 0;
    if (circles.size() > 1)
    {
        double maxArea = -1;
        for (int i = 0; i < circles.size(); i++)
        {   
            if ( maxArea < (M_PI * pow(cvRound(circles[i][2]), 2)) )
            {
                maxArea = M_PI * pow(cvRound(circles[i][2]), 2);
                index = i;
            }
        }
    }

    drawCircles(circles);
    return circles[index];
}

/** @function morphoTransform
*
**/
void InsulateFrameWorks::morphoTransform()
{
    int erosion_size = 2.5;   
    cv::Mat element = cv::getStructuringElement(/*cv::MORPH_CROSS,*/cv::MORPH_RECT,
                      cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
                      cv::Point(erosion_size, erosion_size) );

    //cv::dilate(this->binaryFrame, this->binaryFrame, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    cv::dilate(this->cannyFrame, this->dilation, element);      // cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    cv::erode(this->cannyFrame, this->erosion, element);

    cv::morphologyEx( this->cannyFrame, this->closing, cv::MORPH_CLOSE, element );
    cv::morphologyEx( this->cannyFrame, this->opening, cv::MORPH_OPEN, element );
}

/** @function cornerHarrisDetection
*
**/
void InsulateFrameWorks::cornerHarrisDetection()    //int, void* )
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
                cv::circle( /*this->binaryFrame*/this->frame, cv::Point( i, j ), 5,  cv::Scalar(0), 2, 8, 0 );
                keyPoints.push_back(cv::KeyPoint(i, j, 1));
            }
        }
        if ( keyPoints.size() > _maxKeyPoints) 
        {
            keyPoints.clear();
            break;
        }
    }
}


//-- Points for Line1 = (o1, p1), points for Line2 = (o2, p2)
bool InsulateFrameWorks::intersection(const cv::Point2f &o1, const cv::Point2f &p1, const cv::Point2f &o2, const cv::Point2f &p2, cv::Point &r){
    cv::Point2f x = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;

    if (fabs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;

    return true;
}

InsulateFrameWorks::~InsulateFrameWorks() {}
