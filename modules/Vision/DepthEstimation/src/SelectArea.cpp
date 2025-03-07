/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include <DepthEstimation/SelectArea.hpp>

#define PERCENTAGE_AREA     0.05    // 35%
#define RESIZE_MARK         0.5
#define RESIZE_SIZE         5
#define NEGATIVES_F         50      // 3
#define NUM_DEAD_ESTIMATES  30
#define DISTANCE_THRESHOLD  0.35    // 0.05

#define N_SAMPLES       5
#define N_ROIS          5
#define N_PARTIAL_DIST  40

static bool _newOriginMatrix = false, _loockForCircle = false, _option = false;
std::shared_ptr<IPC> _mmap_camera_status;   // AR and meta-data publisher

Packets::PacketHeader _header;
Packets::PacketDimensionOfInterestArea _command;

static std::vector<int> _roiReference(10, -1);  // coordinates of the benchmarks' initial position
double _rotInZ, _rotInY, _rotInX;

// GLOBAL VARIABLES FOR SURF-BASED DEPTH ESTIMATION
std::vector<double> _totalDisntances;
std::vector<double> _partialDistances;
double _realDistance = 0.80;
// -----------------------------------------------

static bool printDepth = false, _allowFiltering = false;

void SelectArea::publish_on_the_GUI() {
    _command.is_tracking = true;
    _command.square_center[0] = this->roi[0].x;
    _command.square_center[1] = this->roi[0].y;
    _command.square_size[0] = this->roi[0].width;
    _command.square_size[1] = this->roi[0].height;

    auto test =  _mmap_camera_status->write(_command.serialize(), _command.getHeader());
}

SelectArea::SelectArea() {
    /*cv::Point previousScrew(-1, -1);
    this->previousScrew = previousScrew;*/
    this->previousScrew = cv::Point(-1, -1);
}

void SelectArea::setFrame(const cv::Mat &frame) {
    this->frame = frame.clone();
    for (int i = 0; i < 5; i++)
        cv::rectangle(this->frame, this->roi[i], cv::Scalar(255, 0, 0), 2, 1);
}

void SelectArea::compareSharpness(double currentSharpness) {
    if ( this->initSharpness != 0  &&  /*(this->initSharpness < currentSharpness) and*/
        (abs(this->initSharpness - currentSharpness) > 4)) {
        this->focus++;

        switch (this->focus) {
            case 1:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=40");
                break;
            case 2:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=50");
                break;
            case 3:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=55");
                break;
            case 4:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=60");
                break;
            case 5:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=65");
                break;
            case 6:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=70");
                break;
            case 7:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=75");
                break;
            case 8:
                system("v4l2-ctl -d /dev/video0 -c focus_absolute=80");
                this->maximumFocus = true;
                break;
        }
        this->initSharpness = currentSharpness;
    }
    if (this->initSharpness == 0 ) this->initSharpness = currentSharpness;
}

void SelectArea::correctPositionPoints(cv::Point &ptLeftUp, cv::Point &ptRightDown) {  // NOLINT
    if ( ptLeftUp.x < 0 || ptLeftUp.y < 0 ) {
        if ( ptLeftUp.x < 0 ) {
            ptRightDown.x -= ptLeftUp.x;  // - * - == +
            ptLeftUp.x = 0;
        }
        if ( ptLeftUp.y < 0 ) {
            ptRightDown.y -= ptLeftUp.y;
            ptLeftUp.y = 0;
        }
    }
    if ( ptRightDown.x > this->frame.cols || ptRightDown.y > this->frame.rows ) {
        if ( ptRightDown.x > this->frame.cols ) {
            ptLeftUp.x -= (abs(ptRightDown.x - this->frame.cols));
            ptRightDown.x = this->frame.cols;
        }
        if ( ptRightDown.y > this->frame.rows ) {
            ptLeftUp.y -= (abs(ptRightDown.y - this->frame.rows));
            ptRightDown.y = this->frame.rows;
        }
    }
}

void SelectArea::calculateInsulateRois(const cv::Point &ptLeftUp, const cv::Point &ptRightDown) {
    for (int i = 1; i < N_ROIS; i++) this->tracker[i] = cv::TrackerKCF::create();

    this->roi[1] = cv::Rect_<double> (ptLeftUp.x + 5, ptLeftUp.y + 5,
        (abs(ptLeftUp.x - ptRightDown.x) / 2) - 10, (abs(ptLeftUp.y - ptRightDown.y) / 2) - 10);
    this->roi[2] = cv::Rect_<double> (ptLeftUp.x + (abs(ptLeftUp.x - ptRightDown.x) / 2) + 5,
        ptLeftUp.y + 5, (abs(ptLeftUp.x - ptRightDown.x) / 2) - 10,
        (abs(ptLeftUp.y - ptRightDown.y) / 2) - 10);
    this->roi[3] = cv::Rect_<double> (ptLeftUp.x + 5, ptLeftUp.y +
        (abs(ptLeftUp.y - ptRightDown.y) / 2) + 5, (abs(ptLeftUp.x - ptRightDown.x) / 2) - 10,
        (abs(ptLeftUp.y - ptRightDown.y) / 2) - 10);
    this->roi[4] = cv::Rect_<double> (ptLeftUp.x + (abs(ptLeftUp.x - ptRightDown.x) / 2) + 5,
        ptLeftUp.y + (abs(ptLeftUp.y - ptRightDown.y) / 2) + 5,
        (abs(ptLeftUp.x - ptRightDown.x) / 2) - 10, (abs(ptLeftUp.y - ptRightDown.y) / 2) - 10);

    for (int i = 0; i < N_ROIS; i++) {
        _roiReference[i * 2]      = this->roi[i].x;
        _roiReference[(i*2) + 1]  = this->roi[i].y;
    }

    for (int i = 1; i < N_ROIS; i++) this->tracker[i]->init ( this->frame, this->roi[i] );
}

void SelectArea::calculateRoi(const cv::Point &ptLeftUp, const cv::Point &ptRightDown) {
    // this->tracker[0] = cv::Tracker::create( "KCF" );
    this->tracker[0] = cv::TrackerKCF::create();        // change from new OpenCV release
    this->roi[0] = cv::Rect_<double> (ptLeftUp.x, ptLeftUp.y,
        abs(ptLeftUp.x - ptRightDown.x), abs(ptLeftUp.y - ptRightDown.y));

    this->tracker[0]->init(this->frame, this->roi[0]);

    this->calculateInsulateRois(ptLeftUp, ptRightDown);

    this->screwFound = false;
    this->existPattern = true;
}

/**
* Function to calculate the Roi's centre from a Roi given
*/
/*void SelectArea::calculateRoiCenter(cv::Rect_<double> &center)
{
    auto ptLeftUp    = cv::Point(center.x, center.y);
    auto ptRightDown = cv::Point(center.x + center.width, center.y + center.height);
    this->createRoiFromCenter(ptLeftUp, ptRightDown);
}*/

/**
* Function to calculate the Roi's centre from a mouse click given
*/
/*void SelectArea::calculateRoiCenter(cv::Point &center)
{
    auto ptLeftUp    = cv::Point(center.x - this->widthMarc, center.y - this->heightMarc);
    auto ptRightDown = cv::Point(center.x + this->widthMarc, center.y + this->heightMarc);
    this->createRoiFromCenter(ptLeftUp, ptRightDown);
}*/

void SelectArea::createRoiFromCenter(const cv::Point &center, cv::Rect_<double> &centerRoi) {
    cv::Point ptLeftUp, ptRightDown;
    this->originRoi = this->myHomography.getOriginRoi();
    try {
        if ( centerRoi.x == -1 && centerRoi.y == -1 ) {  // area of interest from center point
            ptLeftUp      = cv::Point(center.x - this->widthMarc, center.y - this->heightMarc);
            ptRightDown   = cv::Point(center.x + this->widthMarc, center.y + this->heightMarc);
        } else {  // area of interest from Roi
            ptLeftUp    = cv::Point(centerRoi.x, centerRoi.y);
            ptRightDown = cv::Point(centerRoi.x + centerRoi.width, centerRoi.y + centerRoi.height);
        }

        correctPositionPoints(ptLeftUp, ptRightDown);
        calculateRoi(ptLeftUp, ptRightDown);
        this->choosedArea = true;

        this->originRoi.x = ptLeftUp.x;
        this->originRoi.y = ptLeftUp.y;
        this->myHomography.setoriginRoiPoint(this->originRoi);
    } catch (cv::Exception e) {}
}

Eigen::MatrixXf SelectArea::updateTracker() {
    /*if ( this->screwFound )
    { printDepth = false;
        this->theScrew = updatePositionCircle( this->myInsulate.getCenterPoint() );
        this->theRadiusCenter = this->myInsulate.getRadiusCenter();

    // Create the square, from the circle's radius as soon as found the circle, only 
        if ( !this->firstCircle ) this->widthMarc = this->heightMarc = this->theRadiusCenter + 10;
        this->firstCircle = true;

        createRoiFromCenter(this->theScrew);

        if ( this->theRadiusCenter > 0 )
        {
            cv::circle(this->frame, this->theScrew, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
            cv::circle(this->frame, this->theScrew, this->theRadiusCenter, cv::Scalar(0, 0, 255), 3, 8, 0);
            this->enableFocus = true;
        }
        this->cameraDesviation();
    }*/
    // this->tracker->update ( this->frame, this->roi );
    for (int i = 0; i < N_ROIS; i++) {
        try {
            if ( !this->tracker[i]->update(this->frame, this->roi[i]) )
                throw std::invalid_argument("¡¡¡TRACKER ERROR!!!");
        } catch(cv::Exception e) {/*throw an error*/}
    }

    if ( ++this->numberOfEstimates > NUM_DEAD_ESTIMATES ) checkRoiStatus();
    if ( 0 < this->calculateDistance() < DISTANCE_THRESHOLD ) _loockForCircle = true;

    /** TRY TO CALCULATE THE NEW PATTERN FROM ITS OWN BOUNDINGBOX */
    if ( this->newPattern ) {
        for (int i = 0; i < N_ROIS; i++) this->avoidRoiToDepthStimation.push_back(true);
        this->pattern_corners = this->roi[0];
        this->myHomography.setPattern(this->frame(this->roi[0]));
        this->countErrors = 0;
        this->newPattern = false;

        this->calculateInsulateRois(cv::Point(this->roi[0].x, this->roi[0].y),
                                    cv::Point(this->roi[0].x + this->roi[0].width,
                                              this->roi[0].y + this->roi[0].height) );
        // set the origin matrix
        for (int i = 0; i < 4; i++)
          for (int j = 0; j < 4; j++)
            this->originMatrix = this->currentMatrix;
        _newOriginMatrix = true;

        // this->originMatrix = this->calculateRotations(a, b, c, true, false);
        std::cout << std::endl << "<<<<NEW PATTERN SO, NEW SQUARES>>>>" << std::endl << std::endl;
    }

    this->previousScrew = this->theScrew;
    return printInformation(true);
}

/**
* Detect wrong behaviours of little rois
*/
void SelectArea::checkRoiStatus() {
    std::vector<int> euclideanDisntacePoint;
    int averagePoints = 0;

    for (int i = 0; i < N_ROIS; i++) {
        euclideanDisntacePoint.push_back(sqrt(pow(_roiReference[i*2] - this->roi[i].x, 2)
            + pow(_roiReference[(i* 2) + 1] - this->roi[i].y, 2)) );
        averagePoints += euclideanDisntacePoint[i];
    }
    averagePoints /= N_ROIS;
    for (int i = 1; i < N_ROIS; i++) {
        if ( euclideanDisntacePoint[i] >
            abs(static_cast<int>(averagePoints*(this->thresholdSquareError)/100)) ) {
            // this->newPattern = true;
            this->avoidRoiToDepthStimation[i] = false;
        }
    }
}

Eigen::MatrixXf SelectArea::printInformation(bool scenebox/*=false*/, bool boundingBox/*=false*/) {
    Eigen::MatrixXf err(2, 1);
    intersection(cv::Point2f(this->roi[0].x, this->roi[0].y),
                cv::Point2f(this->roi[0].x + this->roi[0].width,
                this->roi[0].y + this->roi[0].height),
                cv::Point2f(this->roi[0].x + this->roi[0].width,
                this->roi[0].y), cv::Point2f(this->roi[0].x,
                this->roi[0].y + this->roi[0].height));  // , this->centerCoordinates);

    err(0, 0) = (this->frame.cols / 2) - this->centerCoordinates.x;
    err(1, 0) = (this->frame.rows / 2) - this->centerCoordinates.y;
    // err = std::to_string( (this->frame.cols / 2) - centerCoordinates.x ) + ","
    // + std::to_string( (this->frame.rows / 2) - centerCoordinates.y );

    // cv::rectangle ( this->frame, this->roi, cv::Scalar( 255, 0, 0 ), 2, 1);

    putText(this->frame, "X: " + std::to_string(static_cast<int>(this->frame.cols / 2) -
        this->centerCoordinates.x)+ ", Y:" +std::to_string(static_cast<int>(this->frame.rows / 2) -
        this->centerCoordinates.y), cvPoint(this->roi[0].x - 20, this->roi[0].y - 30),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 255), 1, CV_AA);
    intersection(cv::Point2f(0, 0), cv::Point2f(this->frame.cols, this->frame.rows),
                cv::Point2f(this->frame.cols, 0), cv::Point2f(0, this->frame.rows));

    // up left corner
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x-14, this->centerCoordinates.y -14),
            cv::Point2f(this->centerCoordinates.x - 5,  this->centerCoordinates.y - 14),
            cv::Scalar(0, 255, 0), 2);
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x-14, this->centerCoordinates.y -14),
            cv::Point2f(this->centerCoordinates.x - 14, this->centerCoordinates.y - 5),
            cv::Scalar(0, 255, 0), 2);

    // up right corner
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x+14, this->centerCoordinates.y -14),
            cv::Point2f(this->centerCoordinates.x + 5,  this->centerCoordinates.y - 14),
            cv::Scalar(0, 255, 0), 2);
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x+14, this->centerCoordinates.y -14),
            cv::Point2f(this->centerCoordinates.x + 14, this->centerCoordinates.y - 5),
            cv::Scalar(0, 255, 0), 2);

    // down left corner
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x-14, this->centerCoordinates.y +14),
            cv::Point2f(this->centerCoordinates.x - 5,  this->centerCoordinates.y + 14),
            cv::Scalar(0, 255, 0), 2);
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x-14, this->centerCoordinates.y +14),
            cv::Point2f(this->centerCoordinates.x - 14, this->centerCoordinates.y + 5),
            cv::Scalar(0, 255, 0), 2);

    // down right corner
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x+14, this->centerCoordinates.y +14),
            cv::Point2f(this->centerCoordinates.x + 5,  this->centerCoordinates.y + 14),
            cv::Scalar(0, 255, 0), 2);
    cv::line(this->frame, cv::Point2f(this->centerCoordinates.x+14, this->centerCoordinates.y +14),
            cv::Point2f(this->centerCoordinates.x + 14, this->centerCoordinates.y + 5),
            cv::Scalar(0, 255, 0), 2);

    // center point
    cv::circle(this->frame, this->centerCoordinates, 3, cv::Scalar(255, 100, 255), 1, 8, 0);
    // Current pattern for debug
    // cv::rectangle ( this->frame, this->pattern_corners, cv::Scalar( 0, 0, 159 ), 6, 1);

    /** DRAW SCENEBOX AND BOUNDINGBOX */
    /*if ( this->wantToFollowIt)
    {
        if (scenebox)    this->myHomography.drawSceneBox(this->frame);
        if (boundingBox) this->myHomography.drawBoundingBox(this->frame);
    }*/
    return err;
}

void SelectArea::drawRois(cv::Mat& frame) {
    for (int i=0; i < N_ROIS; i++) cv::rectangle(frame, this->roi[i], cv::Scalar(255, 0, 0), 2, 1);
}


void SelectArea::manualFocusIsDisabled() {
    putText(this->frame, "MANUAL FOCUS DISABLED", cvPoint(this->frame.cols/4, this->frame.rows/4),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 255), 1, CV_AA);
}

//-- Points for Line1 = (o1, p1), points for Line2 = (o2, p2)
cv::Point SelectArea::intersection
(const cv::Point2f &o1, const cv::Point2f &p1, const cv::Point2f &o2, const cv::Point2f &p2) {
    cv::Point2f x  = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;

    // if (abs(cross) < /*EPS*/1e-8)
    //    return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    this->centerCoordinates = o1 + d1 * t1;

    return this->centerCoordinates;
}

cv::Point SelectArea::updatePositionCircle(const cv::Point &currentPoint) {
    cv::Point newPoint(currentPoint);

    newPoint.x += this->roi[0].x;
    newPoint.y += this->roi[0].y;

    return newPoint;
}

void SelectArea::restartTheTracker(bool option) {
    cv::Point center;

    if ( !option ) {
        std::vector<cv::Point2f> boundingBox = this->myHomography.getSceneBox();
        center = this->intersection(boundingBox[0], boundingBox[2], boundingBox[1], boundingBox[3]);
    } else {
        center.x = this->theScrew.x;
        center.y = this->theScrew.y;

        this->widthMarc  = this->originWidthMarc;
        this->heightMarc = this->originHeightMarc;
    }

    if ((center.x - this->widthMarc) > 0 && (center.y - this->heightMarc) > 0 &&
        (center.x + this->widthMarc) < this->frame.cols &&
        (center.y - this->heightMarc) < this->frame.rows) {
        cv::Rect_<double> centerRoi(center.x, center.y, this->widthMarc, this->heightMarc);
        this->createRoiFromCenter(center, centerRoi);
    }
}

/** 
* call to homography main function 
* if the errors are more than 5 the system change the patern
*/
void SelectArea::runHomography(cv::Mat &frame, const float robot_pose[4][4]) {
    // set the current matrix
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) this->currentMatrix(i, j) = robot_pose[i][j];

    try {
        auto workHomography = this->myHomography.runHomography(this->frame,
                                                               this->coordinatesBoundingBox);
        this->myHomography.filterMatches(this->leftX, this->rightX);
        if ( !workHomography) { this->countErrors++;
        } else {
            this->countErrors = 0;
            // this->myHomography.targetDistance();
        }
        // if ( this->countErrors > NEGATIVES_F) this->newPattern = true;

        if ( !this->choosedArea && workHomography) this->restartTheTracker();

        this->printMatrix();
    } catch (cv::Exception e) {}
}

Eigen::MatrixXf SelectArea::searchTheScrew(const float robot_pose[4][4]) {
    // std::cout <<"X-POS: " <<  robot_pose[0][3] << std::endl;
    for (int i = 0; i < 4; i++)
       for (int j = 0; j < 4; j++) this->currentMatrix(i, j) = robot_pose[i][j];

    if ( this->resizeMark ) {
        /** Right-Down point */
        int x = (this->roi[0].x + this->roi[0].width + RESIZE_SIZE) > this->frame.cols ?
                this->frame.cols : this->roi[0].x + this->roi[0].width + RESIZE_SIZE;
        int y = (this->roi[0].y + this->roi[0].height + RESIZE_SIZE) > this->frame.rows ?
                this->frame.rows : this->roi[0].y + this->roi[0].height + RESIZE_SIZE;
        cv::Point auxY = cv::Point(x, y);
        /** Left-Up point */
        x = this->roi[0].x - RESIZE_SIZE < 0 ? 0 : this->roi[0].x - RESIZE_SIZE;
        y = this->roi[0].y - RESIZE_SIZE < 0 ? 0 : this->roi[0].y - RESIZE_SIZE;
        cv::Point auxX = cv::Point(x, y);

        calculateRoi(auxX, auxY);

        this->widthMarc  += RESIZE_SIZE;
        this->heightMarc += RESIZE_SIZE;
        this->resizeMark = false;
    }

    /*try{
        this->myInsulate.setHalfFrame(this->frame(this->roi[0]));
    }catch(cv::Exception e)
    { 
        this->choosedArea = false; 
        Matrix<float> error(2,1,0.0);

        return error;
    }

    if ( _loockForCircle ) try
    {   
        this->myInsulate.setHalfFrame( 
            this->frame(cv::Rect_<double>( this->roi[0].x, this->roi[0].y, this->roi[0].width / 2, this->roi[0].height)) ); //LEFT-HALF PICTURE
        auto circles = this->myInsulate.findCircles();

        if ( circles[0] != -1 ) {
            if ( compareMeasures(circles) )
            { 
                this->screwFound = true; 
                this->existPattern = false;
                this->negativeFrames = 0;
            }
        }else if ( ++this->negativeFrames > 5 )
        {
            if ( !this->wantToFollowIt ) this->restartTheTracker(true);

            this->wantToFollowIt = true;
        }else this->wantToFollowIt = false;
    } catch(cv::Exception e){}*/
    // this->wantToFollowIt = ( this->negativeFrames > 10) ? true : false;

    /** thread to publish data */
    std::thread t(&SelectArea::publish_on_the_GUI, this);
    t.join();

    /*Matrix<float> auxMatrix;
    try { auxMatrix = updateTracker(); }
    catch (const std::invalid_argument& e){sleep(2); throw e;}*/

    return updateTracker();
}

/*Matrix<float> searchTheScrew_OLD()
{
    if ( _resizeMark )
    {   //-- Right-Down point
        int x = (_roi.x + _roi.width + RESIZE_SIZE) > _frame.cols ? _frame.cols : _roi.x + _roi.width + RESIZE_SIZE;
        int y = (_roi.y + _roi.height + RESIZE_SIZE) > _frame.rows ? _frame.rows : _roi.y + _roi.height + RESIZE_SIZE;
        cv::Point auxY = cv::Point (x, y);
        //-- Left-Up point 
        x = _roi.x - RESIZE_SIZE < 0 ? 0 : _roi.x - RESIZE_SIZE;
        y = _roi.y - RESIZE_SIZE < 0 ? 0 : _roi.y - RESIZE_SIZE;
        cv::Point auxX = cv::Point (x, y);

        calculateRoi(auxX, auxY);

        _widthMarc  += RESIZE_SIZE;
        _heightMarc += RESIZE_SIZE;
        _resizeMark = false;
    }

    try{
        _myInsulate.setHalfFrame(_frame(_roi));
    }catch(cv::Exception e)
    { 
    _choosedArea = false; 
      //_error(0,0) = 0.0;
      //_error(1,0) = 0.0;
      //std::cout << _error(0,0) << ", " << _error(1,0) << std::endl;
    Matrix<float> error(2,1,0.0);
      return error;
    }
    auto circles = _myInsulate.findCircles();

    if ( circles[0] != -1 ) {
        if ( compareMeasures(circles) ) _screwFound = true;
    }
    else if ( ++_negativeFrames > NEGATIVES_F and _enableFocus ){ focusFrame(); std::cout << "NEGATIVES; " << _negativeFrames << std::endl; }

   return updateTracker();
}*/

bool SelectArea::compareMeasures(cv::Vec3f &circles) {
    std::vector<cv::Vec3f> aux;
    aux.push_back(circles);
    this->myInsulate.drawCircles(aux);

    // this->negativeFrames = 0;
    // this->initSharpness = -1;
    // int trackerArea = this->roi[0].area();

    if ( (M_PI * ( pow(this->myInsulate.getRadiusCenter(), 2)) ) >=
        (this->roi[0].area() * PERCENTAGE_AREA) ) {
        if ( (M_PI * ( pow(this->myInsulate.getRadiusCenter(), 2)) ) >=
            (this->roi[0].area() * RESIZE_MARK) )
            this->resizeMark = true;
        return true;
    }

    return false;
}

void SelectArea::cameraDesviation() {
    // cv::imshow( "Canny", this->getCannyFrame() );

    switch (this->compareQuadrants()) {
        case 0:
            // std::cout << "TOP LEFT" << std::endl;
            break;
        case 1:
            // std::cout << "TOP RIGHT" << std::endl;
            break;
        case 2:
            // std::cout << "BOTTOM LEFT" << std::endl;
            break;
        default:
            // std::cout << "BOTTOM RIGHT" << std::endl;
            break;
    }
    this->printQuadrants();
    for (int i = 0; i < 4; i++) this->quadrants[i] = 0;
}

int SelectArea::compareQuadrants() {
    cv::Mat screwArea = this->getCannyFrame();
    // std::cout << "TYPE: " << screwArea.type() << std::endl;
    for (int i = 5; i < screwArea.rows - 5; i++) {
        for (int j = 5; j < screwArea.cols - 5; j++) {
            /** Euclidean distance between (j, i) and the center of picture */
            if ((sqrt (pow((j - (screwArea.cols / 2 )), 2) + pow((i - (screwArea.rows / 2 )), 2)) )
                < (this->theRadiusCenter + 10) ) {
                screwArea.at<uchar>(i, j) = 0;
                continue;
            }

            if ( static_cast<int>(screwArea.at<uchar>(i, j)) == 255 ) {
                if ( i < (screwArea.rows / 2) )  // the half top
                    j < (screwArea.cols / 2) ? this->quadrants[0]++ : this->quadrants[1]++;
                else                             // the half bottom
                    j < (screwArea.cols / 2) ? this->quadrants[2]++ : this->quadrants[3]++;
            }
        }
    }

    return maximumQuadrant();
}

int SelectArea::maximumQuadrant() {
    int index = 0, max = this->quadrants[0];
    for (int i = 1; i < 4; i++) {
        if ( this->quadrants[i] > max ) {
            max = this->quadrants[i];
            index = i;
        }
    }

    return index;
}

void SelectArea::printQuadrants() {
    for (int i = 0; i < 4; i++)
        std::cout << "Quadrant " << i+1 << ": " << this->quadrants[i] << std::endl;
}

void SelectArea::printMatrix() {
    /*std::cout << "ORIGIN MATRIX" << std::endl;
    std::cout << this->originMatrix << std::endl;
    
    std::cout << std::endl << "CURRENT MATRIX" << std::endl;
    std::cout << this->currentMatrix << std::endl;*/

    auto inverOriginMatrix = this->originMatrix.inverse();

    // std::cout << std::endl << "INVERSE MATRIX" << std::endl;
    // std::cout << inverOriginMatrix << std::endl;

    this->transformationMatrix = inverOriginMatrix * this->currentMatrix;
    /*std::cout << std::endl << "TRANSFORMATION MATRIX" << std::endl;
    std::cout << this->transformationMatrix << std::endl;*/

    if (_newOriginMatrix) this->calculateRotations(_rotInX, _rotInY, _rotInZ, false, false);
    _newOriginMatrix = false;

    this->calculateDistance();
}


Eigen::Matrix4f SelectArea::calculateRotations(double &rotInX, double &rotInY, double &rotInZ,
    bool opt/*transformatrix =false or not*/, bool opt2/*current = true o originmatrix = false*/) {  // NOLINT
    Eigen::Matrix4f AuxMatrix;
    if (opt) {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                AuxMatrix = opt2 ? this->currentMatrix : this->originMatrix;
    }

    KDL::Rotation rotMatrix;
    KDL::Frame newframe;

    for (int i = 0; i < 3; i++) {
        newframe.p(i) = opt ? AuxMatrix(3, i) : this->transformationMatrix(3, i);
        for (int j = 0; j < 3; j++)
            newframe.M(i, j) = opt ? AuxMatrix(i, j) : this->transformationMatrix(i, j);
    }
    rotMatrix = newframe.M;
    if ( opt2 ) {
        rotMatrix.GetEulerZYX(rotInZ, rotInX, rotInY);
        newframe.M.DoRotZ(rotInZ);
    } else {
        rotMatrix.GetEulerZYX(_rotInZ, _rotInX, _rotInY);
        newframe.M.DoRotZ(_rotInZ);
    }

    // std::cout << std::endl << "X-AXIS: " << rotInX << std::endl << "Y-AXIS: " <<
    // rotInY << std::endl << "Z-AXIS: " << rotInZ << std::endl;
    // std::cout << "The new matrix after rotation: " << newframe << std::endl;
    Eigen::Matrix4f currentMatrixRotated;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) currentMatrixRotated(i, j) = newframe.M(i, j);
        currentMatrixRotated(i, 3) = newframe.p(i);
        currentMatrixRotated(i, 3) = opt ? AuxMatrix(i, 3) : this->transformationMatrix(i, 3);
    }
    currentMatrixRotated(3, 3) = 1;

    return currentMatrixRotated;
}

  // calculates the camera's arc rotation if there is not extrinsic calibration
  // 0.07 are the centimetres from the camera to the end-effector
bool SelectArea::translationWithoutCalibration(
    double &translationX, double &translationY, const double rotInX, const double rotInY) {
    translationX = translationX < 0 ?
        fabs(translationX) - (2*0.07*sin(rotInY/2)) : fabs(translationX) + (2*0.07*sin(rotInY/2));
    translationY = translationY > 0 ?
        fabs(translationY) - (2*0.07*sin(rotInX/2)) : fabs(translationY) + (2*0.07*sin(rotInX/2));
    return true;
}


bool SelectArea::filteringOutliers(std::vector<double>& distanceX, std::vector<double>& distanceY,
    const double average_distX, const double average_distY) {
    for (int i = 0; i < distanceX.size(); i++)
        if (distanceX[i] < ((average_distX/distanceX.size()) * 0.5))
            distanceX.erase(distanceX.begin() + i);
    for (int i = 0; i < distanceY.size(); i++)
        if (distanceY[i] < ((average_distY/distanceY.size()) * 0.5))
            distanceY.erase(distanceY.begin() + i);
    return true;
}

cv::Point SelectArea::pointPatternProjection(const int index) {
    // if _option is true, it calculates the correlaction from SURF key.points
    if (_option) return cv::Point(static_cast<int>(this->matchesPattern[index].x),
                    static_cast<int>(this->matchesPattern[index].y));

    int totalPatternXKP = _roiReference[index * 2] + ((index > 0) ?
        (this->roi[0].width / 4) - 5 : this->roi[0].width / 2);
    int totalPatternYKP = ((index > 0) ? (-5 + this->roi[0].height / 4) :
        this->roi[0].height / 2) + _roiReference[(index*2) + 1];

    return cv::Point(static_cast<int>(totalPatternXKP), static_cast<int>(totalPatternYKP));
}

cv::Point SelectArea::pointFrameProjection(const int index) {
    // if _option is true, it calculates the correlaction from SURF key.points
    if (_option) return cv::Point(static_cast<int>(this->matchesFrame[index].x),
                    static_cast<int>(this->matchesFrame[index].y));

    double totalFrameXKP = this->roi[index].x + this->roi[index].width/2;
    double totalFrameYKP = this->roi[index].y + this->roi[index].height/2;

    return cv::Point(static_cast<int>(totalFrameXKP), static_cast<int>(totalFrameYKP));
}

/** to be checked */
float SelectArea::triangulationBySURF(const cv::Point &point1, const cv::Point &point2) {
    float K1 = point1.x / 624.24;
    float K2 = point1.y / 624.24;
    float K3 = point2.x / 624.24;
    float K4 = point2.y / 624.24;
    float z1Numerator = this->transformationMatrix(0, 3) - K3 * this->transformationMatrix(2, 3);
    float z1Denominator = K3 * (this->transformationMatrix(2, 0) * K1 +
        this->transformationMatrix(2, 1) * K2 + this->transformationMatrix(2, 2)) -
        this->transformationMatrix(0, 0) * K1 - this->transformationMatrix(0, 1) *
        K2 - this->transformationMatrix(0, 2);

    float return_value[3];
    return_value[2] = z1Numerator/z1Denominator;
    return_value[0] = K1* return_value[2];
    return_value[1] = K2* return_value[2];

    // this->myHomography.targetDistance();  // to calculate the homography
    return return_value[2];
}

  // deprecated
/* void SURFAverage() {
    std::vector<double> totalDisntances.push_back(_TOTAL - this->transformationMatrix(2,3) );
    std::vector<double> partialDistances.push_back(_TOTAL - this->transformationMatrix(2,3) );
    int16_t cont++;

    If (cont == N_PARTIAL_DIST) {
        double averageDistances = 0;
        for( auto it = partialDistances.begin(); it != partialDistances.end(); it++ ) averageDistances += *it;

        std::cout << "???????????????????? DISTANCE: " << averageDistances << std::endl;
        std::cout << "#################### DISTANCE: " << averageDistances/N_PARTIAL_DIST << std::endl;

        cont = 0;
        partialDistances.clear();
    }
    int isExact = 0;
    for( auto it = totalDisntances.begin(); it != totalDisntances.end(); it++ )
        if ( fabs(*it - _realDistance) < 0.01) isExact++;
    std::cout << "Global precision: There is " << isExact << " correct measures of " << totalDisntances.size() << std::endl;
}*/

double SelectArea::distanceInXByPoint(const double &translationX, const cv::Point &point1,
const cv::Point &point2, const double &YRotation) {
    if (translationX > 0.001) {  // more than 1 millimeter
        // hypotenuse on first image
        float ha = sqrt(pow(point1.x - this->myHomography.getCameraMatrix().at<double>(0, 2), 2)
            + pow(this->myHomography.getCameraMatrix().at<double>(0, 0), 2) );
        // hypotenuse on second image
        float hb = sqrt(pow(point2.x - this->myHomography.getCameraMatrix().at<double>(0, 2), 2)
            + pow(this->myHomography.getCameraMatrix().at<double>(0, 0), 2) );

        return triangulatePosition(0, point1, point2, ha, hb, YRotation, translationX);
    }
    return 0;
}

/** axis = 0 -> X, axis = 1 -> Y */
float SelectArea::triangulatePosition
(const int &axis, const cv::Point &point1, const cv::Point &point2, float &ha,  // NOLINT
 float &hb, const double &rotation, const double &translation) {  // NOLINT
    // swaping the hypotenuses on the triangulation axis dependendt estimation system
    if ((axis ? point1.x : point1.y) < (axis ? point2.x : point2.y)) {
        float aux = ha;
        ha = hb;
        hb = aux;
    }
    float alpha = M_PI/2 - asin(( (axis ? point1.x : point1.y) -
    this->myHomography.getCameraMatrix().at<double>(axis, 2))/ha);
    float beta  = -(M_PI/2 + rotation - asin(( (axis ? point2.x : point2.y) -
                        this->myHomography.getCameraMatrix().at<double>(axis, 2))/hb));
    float gamma = M_PI - alpha - beta;
    if ((axis ? point1.x : point1.y) < (axis ? point2.x : point2.y)) alpha = beta;

    return -translation * sin(alpha) / sin(gamma);
}

double SelectArea::distanceInYByPoint(const double &translationX, const double &translationY,
const cv::Point &point1, const cv::Point &point2, double distanceX, const double &XRotation) {
    float haY, hbY, alphaY, betaY, gammaY;
    if ( translationY > 0.001 ) {  // more than 1 millimeter
        if (translationX > 0.001 && XRotation < 0.01) {  // calculate the hypotenuse
            return sqrt(pow(distanceX, 2) + pow(translationY, 2));
        } else {  // triangulate the position
            // hypotenuse on first image
            haY = sqrt(pow(point1.y - this->myHomography.getCameraMatrix().at<double>(1, 2), 2)
                + pow(this->myHomography.getCameraMatrix().at<double>(1, 1), 2));
            // hypotenuse on second image
            hbY = sqrt(pow(point2.y - this->myHomography.getCameraMatrix().at<double>(1, 2), 2)
                + pow(this->myHomography.getCameraMatrix().at<double>(1, 1), 2));

            return triangulatePosition(1, point1, point2, haY, hbY, XRotation, translationY);
        }
    }
    return 0;
}

/**
 * @param   _option, if it is true, it uses SURF solution
 */
void SelectArea::benchmarkCorrelation
(const int index, cv::Point &point1, cv::Point &point2) {  // NOLINT
    // matchesPattern and matchesFrame are always the same size
    // for (int i = 0 ; i < option ? this->matchesPattern.size() : N_ROIS; i++) {
        // if the benchmark is an outlier, it is not used to depth estimation
    if ( this->avoidRoiToDepthStimation[index] || _option ) {
        // BENCHMARKS FROM HOMOGRAPHY-BASED TRACKING (center of each square)
        point1 = pointPatternProjection(index);
        point2 = pointFrameProjection(index);
    }
}

int16_t SelectArea::numberOfBenchmarks(const std::vector<double> &distanceX) {
    int cant = 0;
    std::cout << "NUMBER OF BENCHMARKS: " << distanceX.size() << " ---> ";
    for (int i= 0; i < N_ROIS; i++) if (this->avoidRoiToDepthStimation[i] == false) { cant++; }
    std::cout << "REAL: " << cant << std::endl;

    return cant;
}

float SelectArea::calculateDistance() {
    // this->removeBadPoints();  // FOR ALGORITHM 2 (SURF's homography + tracking)

    // if ( this->matchesPattern.size() != 0)  // FOR ALGORITHM 2
    if ( _roiReference[0] >= 0 ) {  // for tracking's homography
        double XRotation, YRotation, ZRotation;
        this->calculateRotations(XRotation, YRotation, ZRotation, false, true);

        double translationX = fabs(this->transformationMatrix(1, 3));
        double translationY = fabs(this->transformationMatrix(0, 3));
        this->translationWithoutCalibration(translationX, translationY, XRotation, YRotation);

        if (translationX > 0.001 || translationY > 0.001) {
            double average_distX = 0;
            double average_distY = 0;
            double distanceInX, distanceInY;

            std::vector<double> distanceX, distanceY;

            // matchesPattern and matchesFrame are always the same size
            for (int i = 0 ; i < /*this->matchesPattern.size()*/N_ROIS; i++) {
                // if the benchmark is false, it is not used to depth estimation
                if ( this->avoidRoiToDepthStimation[i] || _option ) {
                    cv::Point point1, point2;
                    this->benchmarkCorrelation(i, point1, point2);

                    distanceInX = this->distanceInXByPoint(translationX, point1,
                                point2, YRotation);
                    distanceInY = this->distanceInYByPoint(translationX, translationY,
                                    point1, point2, distanceInX, XRotation);

                    if (distanceInX >= 0.1 && distanceInX < 7) {  // from 1 cm to 7 meters
                        distanceX.push_back(distanceInX);
                        average_distX += distanceInX;
                    }
                    if (distanceInY >= 0.1 && distanceInY < 7) {
                        distanceY.push_back(distanceInY);
                        average_distY += distanceInY;
                    }
                }
            }

            // Filtering outliers
            this->filteringOutliers(distanceX, distanceY, average_distX, average_distY);
            // IF THERE IS NOT BENCHMARKS, WE NEED TO TAKE A NEW PATTERN AND RESTART THE SYSTEM
            if ( this->numberOfBenchmarks(distanceX) <= 1) this->newPattern = true;
            // if ( !_allowFiltering ) _allowFiltering = true;
            // if ( (distanceX.size() == 0) and _allowFiltering) this->newPattern = true;

            average_distX = average_distY = 0;
            for (int i = 0; i < distanceX.size(); i++) average_distX += distanceX[i];
            for (int i = 0; i < distanceY.size(); i++) average_distY += distanceY[i];
            auto xDistance = average_distX/distanceX.size();
            auto yDistance = average_distY/distanceY.size();

            // CALCULATE AVERAGES FOR TEST USSING SURF-HOMOGRAPHY
            // SURFAverage();  // deprecated

            if ( (xDistance > 0.1 && xDistance < 7) || (yDistance > 0.1 && yDistance < 7)
                && !this->myNormalDistribution.getIsEstimated() ) {
                if ( ++this->numberOfEstimates > NUM_DEAD_ESTIMATES ) {
                    this->myNormalDistribution.runNormalDistribution(xDistance,
                        yDistance, XRotation < 0.01);
                }
            }
        }
    }
    this->printDepthEstimation();

    return this->myNormalDistribution.getDepthEstimation();
}

void SelectArea::printDepthEstimation() {
    if ( this->myNormalDistribution.getIsEstimated() ) {
        std::cout << ">>>>>>>DISTANCE: " << this->myNormalDistribution.getDepthEstimation() <<
        ", WITH " << this->myNormalDistribution.getStandardDeviation() << " OF ERROR" << std::endl;

        putText(this->frame, "CURRENT DEPTH TARGET -> " +
            std::to_string(this->myNormalDistribution.getDepthEstimation()) + "cm.",
            cvPoint(20, 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cvScalar(0, 0, 255), 1, CV_AA);

        putText(this->frame, "ERROR OF -> " +
            (this->myNormalDistribution.getStandardDeviation() == 0 ? " <1" :
            std::to_string(this->myNormalDistribution.getStandardDeviation())) + "%",
            cvPoint(20, 80), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cvScalar(0, 0, 255), 1, CV_AA);
    }
}

void SelectArea::removeBadPoints() {
    if (this->matchesPattern.size() > 0) {
        this->matchesPattern.clear();
        this->matchesFrame.clear();
    }
    this->matchesPattern = this->myHomography.getGoodPointsObj();
    this->matchesFrame   = this->myHomography.getGoodPointsScene();

    for (int i = 0; i < this->matchesPattern.size(); i++) {
        if ((this->matchesPattern[i].x - this->matchesFrame[i].x) > (this->leftX - this->rightX)) {
              this->matchesPattern.erase(this->matchesPattern.begin() + (i+1));
              this->matchesFrame.erase(this->matchesFrame.begin() + (i+1));
        }
    }
    double lX, rX;
    for (int i = 0; i < matchesPattern.size(); i++) {
        lX += this->matchesPattern[i].x;
        rX += this->matchesFrame[i].x;
    }
}

void SelectArea::setCameraParameters(const char* fileName) {
    std::string nameFich(fileName);

    cv::FileStorage fs(nameFich, cv::FileStorage::READ);
    fs["Camera_Matrix"] >> this->camera_Matrix;
    fs["Distortion_Coefficients"] >> this->distCoeffs;

    _header.length = sizeof(Packets::PacketDimensionOfInterestArea);
    _header.type = Packets::PACKET_DIMENSION_OF_INTEREST_AREA_TYPE;

    // MMAP TO TEST THE CORRECT PERFORM OF THE NEW PACKAGE
    _mmap_camera_status = MMAP::CreateWriterPtr("/tmp/camera_status");

    _command.camera_resolution_x = 640;
    _command.camera_resolution_y = 480;
    _command.is_tracking = false;
    auto test = _mmap_camera_status->write(_command.serialize(), _command.getHeader());

    fs.release();

    this->myHomography.setCameraMatrix(this->camera_Matrix);
}

void SelectArea::setFromScratch(int percentageReduction,
    int cols, int rows, const cv::Point &center, cv::Rect_<double> &centerRoi) {  //NOLINT
    if ( !this->avoidRoiToDepthStimation.empty() ) this->avoidRoiToDepthStimation.clear();

    try {
        if (centerRoi.x == -1 && centerRoi.y == -1) {
            this->widthMarc     = this->originWidthMarc   = cols / percentageReduction;
            this->heightMarc    = this->originHeightMarc  = rows / percentageReduction;
        } else {
            this->widthMarc     = this->originWidthMarc   = centerRoi.width;
            this->heightMarc    = this->originHeightMarc  = centerRoi.height;
        }
        this->maximumFocus  = this->firstCircle       = this->patternFromBB = false;
        this->focus         = this->numberOfEstimates = 0;      // = _TOTAL = 0;
        this->countErrors   = 11;

        for (int i = 0; i < N_ROIS-1; i++) this->quadrants[i] = 0;

        // this->setFrame(frame);
        this->newPattern = true;

        this->createRoiFromCenter(center, centerRoi);
        this->myNormalDistribution.setNumMeasures(690);
    } catch (cv::Exception e) {}
}
