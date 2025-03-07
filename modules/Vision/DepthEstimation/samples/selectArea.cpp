/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <DepthEstimation/SelectArea.hpp>
#include <CommUtility/CommunicationPacket.hpp>
#include <RobotArm/RobotArmPackets.hpp>
#include <IPC/FIFO.hpp>
#include <IPC/MMAP.hpp>

#include <stdexcept>
#include <netinet/in.h>
#include <string>
#include <algorithm>
#include <vector>
#include <memory>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <nlohmann/json.hpp>

#define SIZE_AREA       9  // 8
#define K_VEL           0.0005
#define K_POS           0.0001

// set this environmental variable to 1 to enable the choice the ROI from GUI
#define AREA_FROM_GUI "AREA_FROM_GUI"

using json = nlohmann::json;
  // using namespace KDL;

int     _mouseParam = CV_EVENT_FLAG_LBUTTON;
int     _previousPosition, _countDisable;       // for manual focus
int     _focus = 0, _negativeFrames = 0, _clickArea = 0;
double  _initSharpness = -1;
float   _focusLevel;

// OpenCV's global variables
cv::Mat              _frame;
cv::VideoCapture     _cap;
cv::Point            _centerPoint(0, 0);
cv::Ptr<cv::Tracker> _tracker;
cv::Rect_<double>    _myArea, _areaAux;

bool _choosedArea,  _mButtonPressed, _maximumFocus, _disableFocus, _drawIt, _follow;
bool _firstCircle, _tipeOfSetFromScratch, _stopProgram, _GUISelection;

  // IPC's global variables
  // std::shared_ptr<IPC> _fifo_tcp;    // reading the next position
std::shared_ptr<IPC> _fifo_robot;  // sending command to the robot
std::shared_ptr<IPC> _fifo_GUI;    // reading the fifo mouse
std::shared_ptr<IPC> _mmap_robot;  // reading the position of the robot
  // Packets::PacketHeader _header_tcp;

InsulateFrameWorks _myInsulate;
SelectArea _mySelectArea;
Eigen::MatrixXf _error(2, 1);

int _cont = 0;   /** variable to test the robustness of the system */

void receiver(const Packets::SchunkArmStatusPacket &);
void onMouse(int, int, int, int, void*);
void restarMouseParameters();
void focusFrame();
void compareSharpness(double);
bool moveToNewPosition(uint8_t);
void calculateRoi(const uint16_t &, const uint16_t &);
bool initParams(int argc, char ** argv) {
    if (argc < 7) {
        std::cerr << "ERROR: not enough parameters" << std::endl;
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << " [1] Name of FIFO tcp (Es. '/tmp/fifo_tcp')" <<std::endl;
        std::cout << " [2] Name of FIFO GUI (Es. '/tmp/fifo_GUI')" <<std::endl;
        std::cout << " [3] Name of FIFO where send the command "
                      << " (Es. '/tmp/fifo_schunkarm_slave')" <<std::endl;
        std::cout << " [4] Name of MMAP where read the command "
                      << " (Es. '/tmp/mmap_schunkarm_slave')" <<std::endl;
        std::cout << " [5] Camera used to carry out the task" <<std::endl;
        std::cout << "      ... USB Camera (Es. 0)" <<std::endl;
        std::cout << "      ... Video Streaming  "
                     << "(Es. http://localhost:8080/?action=stream?dummy=param.mjpg)" <<std::endl;
        std::cout << "      ... Axis PTZ   (Es. http://192.168.0.90/mjpg/video.mjpg)" <<std::endl;
        std::cout << "      ... Mini Axis Camera "
                     << "(Es. http://192.168.0.91/mjpg/video.mjpg?camera=1)" <<std::endl;
        std::cout << " [6] Camera number for calibration parameters "
            << "(Es. '../workspaces/CamCalibration/DataCalibration/DataCam_LG-N.txt" << std::endl;
        return false;
    }

    _choosedArea = _stopProgram = _mButtonPressed = _maximumFocus = _disableFocus = false;
    _follow = _tipeOfSetFromScratch = _stopProgram = _GUISelection = _drawIt = false;

    //-- examples Axis camera "http://192.168.0.91/mjpg/video.mjpg?camera=1"  "http://128.141.185.206/mjpg/video.mjpg"  "http://localhost:8080/?action=stream?dummy=param.mjpg"
    //-- setting the camera to use
    std::string cameraOn(argv[5]);
    if ( cameraOn == "0" ) {
        if ( !_cap.open(0) ) {
            std::cerr << "ERROR: camera is missing" << std::endl;
            return false;
        }
    } else {
        if ( !_cap.open(cameraOn) ) {
            std::cerr << "ERROR: camera is missing" << std::endl;
            return false;
        }
    }
    return true;
}

void captureFrame() {
  while ( !_stopProgram ) {
    try {
        _cap >> _frame;
        _mySelectArea.setFrame(_frame);
    }catch(cv::Exception e) {}
  }
}

void clickedArea() {
    _GUISelection = true;
    restarMouseParameters();

    std::string buffer;
    Packets::PacketHeader header_GUI;

    while ( std::getenv(AREA_FROM_GUI) && !_stopProgram ) {
        _fifo_GUI->read(buffer, header_GUI);
        if (header_GUI.type == Packets::PACKET_AREA_CLICKED_COORDINATE_TYPE) {
            Packets::PacketAreaClickedCoordinate GUI_command;
            GUI_command.deserialize(buffer);

            cv::Point pt = cv::Point(GUI_command.x, GUI_command.y);
            // std::cout << "\t value="<< _frame.at<uchar>(x,y)<< std::endl;
            std::cout << "x = "<< pt.x <<"  \t y="<< pt.y << std::endl;
            _myArea.y = _myArea.x = -1;
            _mySelectArea.setFromScratch(SIZE_AREA, _frame.cols, _frame.rows, pt, _myArea);
        }
    }
}


int main(int argc, char ** argv) {
    if ( !initParams(argc, argv) ) return -1;
    _mySelectArea.setCameraParameters(argv[6]);

    std::thread theCaptureFrame(captureFrame);   // -- thread to visual feedback on real time
    // std::thread theFifoGUI( clickedArea );    // getting the click from the GUI
    cv::namedWindow("Scene", CV_WINDOW_NORMAL);
    // handler to mouse
    cv::setMouseCallback("Scene", onMouse, &_mouseParam);

    // Robot needed variables
    _fifo_GUI   = FIFO::CreateReaderPtr(argv[2]);   // reading the click's coordinate
    _fifo_robot = FIFO::CreateWriterPtr(argv[3]);   // sending command to the robot
    _mmap_robot = MMAP::CreateReaderPtr(argv[4]);   // reading the position of the robot

    std::shared_ptr<IPC> fifo = FIFO::CreateWriterPtr(argv[1]);

    Packets::SchunkArmStatusPacket status_robot;
    Packets::PacketHeader header_robot_status;
    header_robot_status.length = sizeof(Packets::PacketArmWorldCoordinate);
    header_robot_status.type = Packets::PACKET_ARM_WORLD_COORDINATE_TYPE;

    std::thread theFifoGUI;
    std::string buffer;

    while (1) {
        _mmap_robot->read(buffer, header_robot_status);  // getting ROBOT info
        status_robot.deserialize(buffer);
        // getting the click from the GUI
        if ( std::getenv(AREA_FROM_GUI) ) std::thread theFifoGUI( clickedArea );
        if (_choosedArea) {
            receiver(status_robot);
        } else {  // print message to the user
            if ( _clickArea >= 0 && _clickArea < 20 )
                putText(_frame, "CLICK TO SELECT AREA", cvPoint(30, 60),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1.8, cvScalar(0, 0, 255), 1, CV_AA);
            if ( _clickArea++ == 40 ) _clickArea = 0;
        }

        cv::imshow("Scene", _mySelectArea.getExistPattern() ? _mySelectArea.getFrame() : _frame);

        if ( !std::getenv(AREA_FROM_GUI) && _GUISelection ) theFifoGUI.join();
        if ( cv::waitKey(20) == 27 ) {
            _stopProgram = true;
            break;
        }
    }

    theCaptureFrame.join();  // -- catching the captureFrame thread
    if ( std::getenv(AREA_FROM_GUI) ) theFifoGUI.join();

    return 0;
}



void receiver(const Packets::SchunkArmStatusPacket &status_robot) {
    _error = _mySelectArea.searchTheScrew(status_robot.arm_position);
    // _error.transpose().print();

    Packets::PacketHeader header_command;
    header_command.length = sizeof(Packets::PacketArmMatrixWorldCoordinate);
    header_command.type = Packets::PACKET_ARM_WORLD_COORDINATE_TYPE;
    Packets::PacketArmMatrixWorldCoordinate command;

    for (int i=0; i < 4; i++)
        for (int j=0; j < 4; j++) command.T_matrix[i][j] = status_robot.arm_position[i][j];

    command.T_matrix[0][3] += (K_POS * _error(0, 0));  // HORIZONTAL
    command.T_matrix[1][3] += (K_POS * _error(1, 0));  // VERTICAL

    if ( (fabs(K_POS * _error(0, 0)) >= 0.0005) || ( fabs(K_POS * _error(1, 0)) >= 0.0005) )
        _fifo_robot->write(command.serialize(), command.getHeader());
}

    /*Packets::PacketArmWorldCoordinate packet_velocity;
    packet_velocity.tcp = 1;
    packet_velocity.task_coordinate[0] = 0.0;
    packet_velocity.task_coordinate[1] = 0.0;
    packet_velocity.task_coordinate[2] = 0.0;
    packet_velocity.task_coordinate[3] = 0.0;
    packet_velocity.task_coordinate[4] = 0.0;
    packet_velocity.task_coordinate[5] = 0.0;

    packet_velocity.task_coordinate[1] = K_VEL * error(0, 0);
    packet_velocity.task_coordinate[0] = K_VEL * error(1, 0);

    std::cout << "Velocities to the robot: " << packet_velocity.task_coordinate[0] << " "
        << packet_velocity.task_coordinate[1] << std::endl;
    fifo->write(packet_velocity.serialize(), header);*/

/**
* Handler of the mouse
* It takes the event from mouse and chooses the option to carry out
* @event, event captured
* @x & @y, cursor coordinates from the event
*/
void onMouse(int event, int x, int y, int, void* param) {
    switch (event) {
        case CV_EVENT_LBUTTONDOWN:
            system("v4l2-ctl -d /dev/video0 -c focus_auto=1");  // auto focus ON

            _follow = _drawIt = true;
            _myArea.x = _areaAux.x = x;
            _myArea.y = _areaAux.y = y;
            break;
        case CV_EVENT_MOUSEMOVE:
            if (_mButtonPressed) {  // changing focus by front2back mouse with roll button pressed
                _focusLevel = _previousPosition - y > 0 ? _focusLevel + 0.5 : _focusLevel - 0.51;
                system(("v4l2-ctl -d /dev/video0 -c focus_absolute="
                    + std::to_string(_focusLevel)).c_str());
                _previousPosition = y;
            } else if ( _follow ) {
                _tipeOfSetFromScratch = true;
                calculateRoi(x, y);
            }
            break;
        case CV_EVENT_MBUTTONDOWN:
            system("v4l2-ctl -d /dev/video0 -c focus_auto=0");  // auto focus off
            _mButtonPressed = true;
            _previousPosition = y;
            /** the depth of the focus is calculated by 
            forward/backward mouse movement while the MBUTTON keeps pressed */
            break;
        case CV_EVENT_MBUTTONUP:
            _mButtonPressed = false;
            _focusLevel = 30;  // average focus level
            break;
        case CV_EVENT_LBUTTONUP:
            _choosedArea = true;
            if ( !_tipeOfSetFromScratch ) _areaAux.x = _areaAux.y = -1;
            _mySelectArea.setFromScratch(SIZE_AREA, _frame.cols,
                _frame.rows, cv::Point(x, y), _myArea);
            _tipeOfSetFromScratch = _follow = false;
            break;
        case CV_EVENT_RBUTTONDOWN:
            system("v4l2-ctl -d /dev/video0 -c focus_auto=1");  // enabling the autofocus
            restarMouseParameters();
            break;
    }
}

/**
* To calculate the ROI on the drag & drop from onMouse
*/
void calculateRoi(const uint16_t &x, const uint16_t &y) {
    if ( ((x > _areaAux.x) && (y > _areaAux.y)) ) {
            _myArea.width   = x - _myArea.x;
            _myArea.height  = y - _myArea.y;
    } else {
        if ((x < _areaAux.x) && (y < _areaAux.y)) {
                _myArea.x = x;
                _myArea.y = y;
                _myArea.width   = abs(x - _areaAux.x);
                _myArea.height  = abs(y - _areaAux.y);
            }
        if (x < _areaAux.x && y > _areaAux.y) {
                _myArea.width   = abs(x - _areaAux.x);
                _myArea.x   = x;
                _myArea.height  = y - _myArea.y;
        } else if ( x > _areaAux.x && y < _areaAux.y ) {
            _myArea.height  = abs(y - _areaAux.y);
                _myArea.y   = y;
            _myArea.width   = x - _myArea.x;
        }
    }
}

void restarMouseParameters() {
    _disableFocus = true;
    _countDisable = 0;

    // -- testing how much strong is the system --//
    // _cont = 0;
    // moveToNewPosition(0);
    // -------------------------------------------//

    _mySelectArea.setExistPattern(false);
    _mySelectArea.setChoosedArea(false);
    _choosedArea = _drawIt = false;
    _myArea.x = _areaAux.x = 0;
    _myArea.y = _areaAux.y = 0;
}

void compareSharpness(double currentSharpness) {
    if ( _initSharpness != 0  && abs(_initSharpness - currentSharpness) > (4) ) {  // + _focus) )
        _focus++;

        switch (_focus) {
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
                _maximumFocus = true;
                break;
        }
        _negativeFrames = 0;

        _initSharpness = currentSharpness;
    }
    if ( _initSharpness == 0 ) _initSharpness = currentSharpness;
}

void focusFrame() {
    //-- Disable autofocus and set the absolute focus
    system("v4l2-ctl -d /dev/video0 -c focus_auto=0");
    system("v4l2-ctl -d /dev/video0 -c focus_absolute=30");

    _myInsulate.edgesSobel(true);
    cv::Scalar V = mean(abs(_myInsulate.getSobelHalfFrame()));
    std::cout << V[0] << std::endl;

    if ( !_maximumFocus ) compareSharpness(V[0]);
}

/**
* (DEMO) USED TO SHOW HOW MUCH STRONG IS THE DEPTH ESTIMATION 
* Moving arm using fix joints position and specific speed
* @option, parameter to indicate which position to send
*/
bool moveToNewPosition(uint8_t option) {
    std::cout << "MOVING ARM" << std::endl;  // sleep(1);
    Packets::PacketHeader header_robot;
    header_robot.length = sizeof(Packets::PacketArmJointByJointPositionsVel);
    header_robot.type = Packets::PACKET_ARM_JOINT_BY_JOINT_POSITION_VEL_TYPE;
    Packets::PacketArmJointByJointPositionsVel command;

    switch (option) {
        case 0:
            command.joint[0] = 0.179053;      // -1.53863;
            command.joint[1] = -0.246824;     // -0.0298451;
            command.joint[2] = 0.926683;      // 1.16677;
            command.joint[3] = -0.058835;     // -0.0591667;
            command.joint[4] = 1.86447;       // 1.81687;
            command.joint[5] = 0.0899019;     // 0.104214;
            command.joint[6] = 7.96417e-28;   // -4.7024e+11;
            command.joint[7] = -4.89565e+33;  // 3.99651e+17;
        break;
        case 1:
            command.joint[0] = 0.0906349;      // -1.51561;
            command.joint[1] = -0.108263;      // -0.0152716;
            command.joint[2] = 1.10513;        // 1.18539;
            command.joint[3] = -0.0491485;     // -0.0618894;
            command.joint[4] = 1.825;          // 1.81451;
            command.joint[5] = 0.00734784;     // 0.126257;
            command.joint[6] = 7.96417e-28;    // -4.7024e+11;
            command.joint[7] = -4.89565e+33;   // 3.99651e+17;
        break;
        case 2:
            command.joint[0] = 0.437484;       // -1.51561;
            command.joint[1] = -1.70679;       // -0.0152716;
            command.joint[2] = 1.17227;        // 1.18539;
            command.joint[3] = 0.51304;        // -0.0618894;
            command.joint[4] = -1.1851;        // 1.81451;
            command.joint[5] = -0.152472;      // 0.126257;
            command.joint[6] = 1.46714e-14;    // -4.7024e+11;
            command.joint[7] = 1.03194;        // 3.99651e+17;
        break;
        case 3:
            command.joint[0] = 0.0;  // 0.437484;     // -1.51561;
            command.joint[1] = 0.0;  // -1.70679;     // -0.0152716;
            command.joint[2] = 0.0;  // 1.17227;      // 1.18539;
            command.joint[3] = 0.0;  // 0.51304;      // -0.0618894;
            command.joint[4] = 0.0;  // -1.1851;      // 1.81451;
            command.joint[5] = 0.0;  // -0.152472;    // 0.126257;
            command.joint[6] = 0.0;  // 1.46714e-14;  // -4.7024e+11;
            command.joint[7] = 0.0;  // 1.03194;      // 3.99651e+17;
        break;
        case 4:  // for paper
            command.joint[0] = -0.0684518;
            command.joint[1] = 0.26433;
            command.joint[2] = -1.81321;
            command.joint[3] = -0.46993;
            command.joint[4] = -0.250961;
            command.joint[5] = 0.508083;
            command.joint[6] = -0.0170836;
            command.joint[7] = -1.68369e-25;
        break;
    } sleep(3);
    command.vel = 0.05;
    auto test =  _fifo_robot->write(command.serialize(), command.getHeader());
    std::cout << "  FIN MOVING ARM" << std::endl; sleep(1);

    return true;
}



/*void KUKA_connection() {
    // clear hints
    memset(&hints, 0, sizeof hints);

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    status = getaddrinfo(address, port, &hints, &res);
    if (status != 0) {
        fprintf(stderr, "Error getaddrinfo\n");
        exit(1);
    }

    socket_id = socket(res->ai_family, res->ai_socktype, 0);
    if (socket_id < 0) {
        fprintf(stderr, "Error socket \n");
        is_connect = false;
        exit(2);
    }

    status = connect(socket_id, res->ai_addr, res->ai_addrlen);
    if (status < 0) {
        fprintf(stderr, "Error connectioni \n");
        is_connect = false;
        exit(3);
    }

    std::cout << "Connect to the server! " << std::endl;
}

void KUKA_runHomography() {
    while (true) {
        struct sockaddr_storage sender;
        socklen_t sendsize = sizeof(sender);
        char message_buffer[1024];

        char size[sizeof(int)];
        int retcode = recv(socket_id, size, sizeof(size), 0);
        int64_t msgLen = atoi(size);

        retcode = recv(socket_id, message_buffer, msgLen, MSG_WAITALL);
        message_buffer[retcode] = 0;
        std::string sBuffer = std::string(message_buffer);

        std::cout << "Message:" << sBuffer << "::" << std::endl;
        nlohmann::json json_packet;
        try {
            json_packet = json::parse(sBuffer);
        } catch (const nlohmann::detail::parse_error& e) {
            std::cout << "Parse error: " << (sBuffer).length() << std::endl;
            std::cout << "Message: " << sBuffer << std::endl;
            continue;
        }
        float kukaPos[6];
        std::vector<float> temp1 = json_packet.at("taskPose").get<std::vector<float>>();
        std::copy(temp1.begin(), temp1.end(), kukaPos);

        Vector translation = Vector(kukaPos[0], kukaPos[1], kukaPos[2]);
        Rotation rot = Rotation::EulerZYX(kukaPos[3], kukaPos[4], kukaPos[5]);
        KDL::Frame pos = KDL::Frame(rot, translation);

        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 4; k++) {
                std::cout << pos(i, k) << "  ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;

        break;
    }
}*/
