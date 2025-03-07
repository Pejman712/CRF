/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
*/

#include "HandEyeCalibration/HandEye.hpp"

#include <nlohmann/json.hpp>
#include <limits>
#include <vector>
#include <opencv2/core/core.hpp>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <cstdio>
#include <stdlib.h>
#include <string>

#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/vision/vpPose.h>
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/detection/vpDetectorQRCode.h>

namespace crf {
namespace applications {
namespace handeyecalibration {

HandEye::HandEye(const std::string& configFileName):_log("HandEye") {
    // Get current directory
    std::string directory = __FILE__;
    directory = directory.substr(0, directory.find("cpproboticframework"));

    // Read configuration names from .json
    std::ifstream config(configFileName);
    nlohmann::json jConfig;
    config >> jConfig;
    _inputImageFilename = directory + jConfig.at("inputImageFilename").get<std::string>();
    _inputPoseFilename = directory + jConfig.at("inputPoseFilename").get<std::string>();
    _outputFilename = directory + jConfig.at("outputFilename").get<std::string>();
    _intrinsicFile = directory + jConfig.at("intrinsicFile").get<std::string>();
    _cameraName = jConfig.at("cameraName").get<std::string>();
    _numberOfFrames = 0;
    _log->info("Configuration names initialized");
}

HandEye::~HandEye() {
    // Remove files created during execution
    for (unsigned int i = 1; i <= _numberOfFrames; i++) {
        std::string ss_cPo;
        ss_cPo = "pose_cPo_";
        ss_cPo.append(std::to_string(i));
        ss_cPo.append(".yaml");
        if (!boost::filesystem::exists(ss_cPo)) {
            _log->warn("Can't find pose_cPo_{0} file!", i);
            _log->warn("Impossible to delete it");
        } else {
            const char * c = ss_cPo.c_str();
            std::remove(c);
            _log->info("pose_cPo_ {0} file removed!", i);
        }
    }
}

bool HandEye::estimateChessboardPose(int chessboardWidth,
    int chessboardHeight, double chessboardSquareSize) {
    if (!checkChessboardParameters(chessboardWidth, chessboardHeight, chessboardSquareSize)) {
        _log->warn("Invalid chessboard parameters");
        return false;
    }
    vpVideoReader reader;
    reader.setFileName(_inputImageFilename);
    vpImage<vpRGBa> I;
    reader.open(I);
    std::vector<vpPoint> corners_pts = calcChessboardCorners(chessboardWidth,
        chessboardHeight, chessboardSquareSize);
    vpCameraParameters cam;
    vpXmlParserCamera parser;
    if (parser.parse(cam, _intrinsicFile, _cameraName,
        vpCameraParameters::perspectiveProjWithDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
        _log->warn("Failed to load XML intrinsics file");
        return false;
    }
    _log->info("cam:\n {}", cam);
    int imgNumber = 0;
    while (!reader.end()) {
        imgNumber++;
        reader.acquire(I);
        cv::Mat matImg;
        vpImageConvert::convert(I, matImg);
        cv::Size chessboardSize(chessboardWidth, chessboardHeight);
        std::vector<cv::Point2f> corners2D;
        bool found = cv::findChessboardCorners(matImg, chessboardSize, corners2D,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        vpHomogeneousMatrix cMo;
        if (!found) {
            _log->warn("Chessboard Corners not found for image {}", imgNumber);
            return false;
        }
        cv::Mat matImg_gray;
        cv::cvtColor(matImg, matImg_gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(matImg_gray, corners2D, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));
        for (size_t i = 0; i < corners_pts.size(); i++) {
            vpImagePoint imPt(corners2D[i].y, corners2D[i].x);
            double x = 0.0, y = 0.0;
            vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
            corners_pts[i].set_x(x);
            corners_pts[i].set_y(y);
        }
        vpPose pose;
        pose.addPoints(corners_pts);
        vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;
        double r_dementhon = std::numeric_limits<double>::max();
        double r_lagrange = std::numeric_limits<double>::max();
        bool pose_dementhon = pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
        if (pose_dementhon) {
            r_dementhon = pose.computeResidual(cMo_dementhon);
        }
        bool pose_lagrange = pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
        if (pose_lagrange) {
            r_lagrange = pose.computeResidual(cMo_lagrange);
        }
        cMo = (r_dementhon < r_lagrange) ? cMo_dementhon : cMo_lagrange;
        if (!pose.computePose(vpPose::VIRTUAL_VS, cMo)) {
            _log->warn("Problem when computing final pose using VVS");
            return false;
        }
        cv::drawChessboardCorners(matImg, chessboardSize, corners2D, found);
        vpImageConvert::convert(matImg, I);
        vpPoseVector pose_vec(cMo);
        std::stringstream ss;
        ss << "pose_cPo_" << reader.getFrameIndex() << ".yaml";
        _log->info("Save {}", ss.str());
        pose_vec.saveYAML(ss.str(), pose_vec);
    }
    _numberOfFrames = imgNumber;
    return true;
}
bool HandEye::estimateQRCodePose(double codeWidthInMeters, double codeHeightInMeters) {
    if (!checkQRCodeParameters(codeWidthInMeters, codeHeightInMeters)) {
        _log->error("Invalid QR Code parameters");
        return false;
    }

    vpVideoReader reader;
    reader.setFileName(_inputImageFilename);
    vpImage<unsigned char> I;
    reader.open(I);

    vpCameraParameters cam;
    vpXmlParserCamera parser;
    if (parser.parse(cam, _intrinsicFile, _cameraName,
        vpCameraParameters::perspectiveProjWithDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
        _log->warn("Failed to load XML intrinsics file");
        return false;
    }
    _log->info("cam:\n {}", cam);

    // 3D model of the QRcode
    std::vector<vpPoint> points;
    points.push_back(vpPoint(-codeWidthInMeters/2, -codeWidthInMeters/2, 0));
    points.push_back(vpPoint(codeWidthInMeters/2, -codeWidthInMeters/2, 0));
    points.push_back(vpPoint(codeWidthInMeters/2,  codeWidthInMeters/2, 0));
    points.push_back(vpPoint(-codeWidthInMeters/2,  codeWidthInMeters/2, 0));

    bool init = true;

    vpDetectorQRCode detector;
    int imgNumber = 0;

    while (!reader.end()) {
        imgNumber++;
        reader.acquire(I);

        bool status = detector.detect(I);
        int objectsFound = detector.getNbObjects();
        _log->info("Image {0}: {1}  bar code detected", imgNumber, objectsFound);
        if (objectsFound > 1) {
            _log->error("Impossible to estimate pose of more than 1 QR Code at the same time");
            return false;
        }

        // true if at least one QRcode is detected
        if (!status) {
            _log->error("No QRCode detected in image{0}", imgNumber);
            return false;
        }
        // get the four corners location in the image
        std::vector<vpImagePoint> p = detector.getPolygon(0);
        // resulting pose is available in cMo
        vpHomogeneousMatrix cMo = computeQRPose(points, p, cam, init);

        vpPoseVector pose_vec(cMo);
        std::stringstream ss;
        ss << "pose_cPo_" << reader.getFrameIndex() << ".yaml";
        _log->info("Save {}", ss.str());
        pose_vec.saveYAML(ss.str(), pose_vec);

        std::string message = detector.getMessage(0);
        _log->debug("QR Message {0}: \n {1}", imgNumber, message);
        _log->debug("cMo {0}; \n {1}", imgNumber, cMo);
    }

    _numberOfFrames = imgNumber;
    return true;
}

vpHomogeneousMatrix HandEye::computeQRPose(const std::vector<vpPoint>& points,
    const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam, bool init) {
    assert(points.size() == ip.size());
    vpPose pose;
    double x = 0, y = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
        vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
        vpPoint newPoint(points[i]);
        newPoint.set_x(x);
        newPoint.set_y(y);
        pose.addPoint(newPoint);
    }
    vpHomogeneousMatrix cMo;
    if (init == true) {
      vpHomogeneousMatrix cMo_dem;
      vpHomogeneousMatrix cMo_lag;
      pose.computePose(vpPose::DEMENTHON, cMo_dem);
      pose.computePose(vpPose::LAGRANGE, cMo_lag);
      double residual_dem = pose.computeResidual(cMo_dem);
      double residual_lag = pose.computeResidual(cMo_lag);
      if (residual_dem < residual_lag)
        cMo = cMo_dem;
      else
        cMo = cMo_lag;
    }
    pose.computePose(vpPose::VIRTUAL_VS, cMo);
    return cMo;
}

bool HandEye::estimateRobotCameraTransform() {
    unsigned int ndata = _numberOfFrames;
    if (ndata <= 8) {
        _log->warn("Number of frames not valid, should be at least 8");
        return false;
    }
    std::vector<vpHomogeneousMatrix> cMo(ndata);
    std::vector<vpHomogeneousMatrix> wMe(ndata);
    vpHomogeneousMatrix eMc;
    for (unsigned int i = 1; i <= ndata; i++) {
        std::ostringstream ss_fPe, ss_cPo;
        if (_inputPoseFilename.size() > 200) {
            _log->warn("Input pose filename should be shorter than 200 characters");
            return false;
        }
        const char * c = _inputPoseFilename.c_str();
        char buffer[200];
        snprintf(buffer, sizeof(buffer), c, i);
        ss_fPe << buffer;
        ss_cPo << "pose_cPo_" << i << ".yaml";
        _log->info("Use fPe={0} , cPo={1}", ss_fPe.str(), ss_cPo.str());
        vpPoseVector wPe;
        if (wPe.loadYAML(ss_fPe.str(), wPe) == false) {
            _log->warn("Unable to read data from: {}", ss_fPe.str());
            return false;
        }
        wMe[i - 1] = vpHomogeneousMatrix(wPe);
        vpPoseVector cPo;
        if (cPo.loadYAML(ss_cPo.str(), cPo)  == false) {
            _log->warn("Unable to read data from: {}", ss_cPo.str());
            return false;
        }
        cMo[i-1] = vpHomogeneousMatrix(cPo);
    }
    vpHandEyeCalibration::calibrate(cMo, wMe, eMc);
    std::ofstream file_eMc(_outputFilename);
    eMc.save(file_eMc);
    _log->info("Save eMc.txt");
    _log->info("Output: Hand-eye calibration result: eMc estimated ");
    _log->info("eMC:\n {}", eMc);
    vpThetaUVector ePc(eMc);
    _log->info("theta U (deg): {0} {1} {2}", vpMath::deg(ePc[0]),
        vpMath::deg(ePc[1]), vpMath::deg(ePc[2]));
    return true;
}

std::vector<vpPoint> HandEye::calcChessboardCorners(int width,
    int height, double squareSize) {
    std::vector<vpPoint> corners;
    corners.resize(0);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            vpPoint pt;
            pt.set_oX(j*squareSize);
            pt.set_oY(i*squareSize);
            pt.set_oZ(0.0);
            corners.push_back(pt);
        }
    }
    return corners;
}

bool HandEye::checkChessboardParameters(int chessboardWidth,
    int chessboardHeight, double chessboardSquareSize) {
    _log->info("Parameters:");
    _log->info("CHESSBOARD_WIDTH= {}", chessboardWidth);
    _log->info("CHESSBOARD_HEIGHT= {}", chessboardHeight);
    _log->info("CHESSBOARD_SQUARE_SIZE= {}", chessboardSquareSize);
    _log->info("INPUT_IMAGE_FILENAME= {}", _inputImageFilename);
    _log->info("INPUT_POSE_FILENAME= {}", _inputPoseFilename);
    _log->info("INTRINSIC_FILE= {}", _intrinsicFile);
    _log->info("CAMERA_NAME= {}", _cameraName);
    if (chessboardWidth <= 0) {
        _log->warn("chessboardWidth not valid");
        return false;
    }
    if (chessboardHeight <= 0) {
        _log->warn("chessboardHeight not valid");
        return false;
    }
    if (chessboardSquareSize <= 0) {
        _log->warn("chessboardSquareSize not valid");
        return false;
    }
    if (chessboardWidth % 2 == 0 && chessboardHeight % 2 == 0) {
        _log->warn("chessboardWidth and chessboardHeight could not be both even");
        return false;
    }
    if (chessboardWidth % 2 != 0 && chessboardHeight % 2 != 0) {
        _log->warn("chessboardWidth and chessboardHeight could not be both odd");
        return false;
    }
    return true;
}

bool HandEye::checkQRCodeParameters(double codeWidthInMeters, double codeHeightInMeters) {
    _log->info("Parameters:");
    _log->info("QR CODE WIDTH IN METERS= {}", codeWidthInMeters);
    _log->info("QR CODE HEIGHT IN METERS= {}", codeHeightInMeters);
    _log->info("INPUT_IMAGE_FILENAME= {}", _inputImageFilename);
    _log->info("INPUT_POSE_FILENAME= {}", _inputPoseFilename);
    _log->info("INTRINSIC_FILE= {}", _intrinsicFile);
    _log->info("CAMERA_NAME= {}", _cameraName);

    if (codeWidthInMeters <= 0) {
        _log->error("codeWidthInMeters not valid");
        return false;
    }

    if (codeHeightInMeters <= 0) {
        _log->error("codeHeightInMeters not valid");
        return false;
    }

    return true;
}
}  // namespace handeyecalibration
}  // namespace applications
}  // namespace crf
