/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <DepthEstimation/SelectArea.hpp>

#include <thread>
#include <string>
#include <memory>

#include <RobotArm/RobotArmPackets.hpp>

#include <IPC/FIFO.hpp>
#include <IPC/MMAP.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#define SIZE_AREA   9  // 8
#define K_VEL       0.0005
#define K_POS       0.0001

int _mouseParam = CV_EVENT_FLAG_LBUTTON;
int _previousPosition, _countDisable;
float  _focusLevel = 30;
cv::Mat _frame;
cv::VideoCapture _cap;
  // cv::Point _centerPoint(0, 0);
SelectArea _mySelectArea;
bool _choosedArea = false, _mButtonPressed = false, _disableFocus = false;
bool _takingDimensions = false, _drawIt = false, _follow = false;
cv::Rect_<double> _myArea, _areaAux;

  // int _x_vel, _y_vel;
int _cont = 0;   /** variable to test how much strong is the system */

Matrix<float> _error(2, 1, 0.0);
std::shared_ptr<IPC> _fifo_robot;  // sending command to the robot
std::shared_ptr<IPC> _fifo_GUI;    // reading the fifo mouse
std::shared_ptr<IPC> _mmap_robot;  // reading the position of the robot
KDL::Chain chain;
Packets::PacketHeader _header_tcp;

void printMatrixs(float[], float[], float[][4]);
void calculateRoi(const int16_t, const int16_t);
bool initParams(int argc, char ** argv) {
    if (argc < 7) {
        std::cerr << "ERROR: not enough parameters" << std::endl;
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << " [1] Name of FIFO tcp (Es. '/tmp/fifo_tcp')" <<std::endl;
        std::cout << " [2] Name of FIFO GUI (Es. '/tmp/fifo_GUI')" <<std::endl;
        std::cout << " [3] Name of FIFO where send the command " <<
            "(Es. '/tmp/fifo_schunkarm_slave')" <<std::endl;
        std::cout << " [4] Name of MMAP where read the command " <<
            "(Es. '/tmp/mmap_schunkarm_slave')" <<std::endl;
        std::cout << " [5] Camera used to carry out the task" <<std::endl;
        std::cout << " ... USB Camera      (Es. 0)" <<std::endl;
        std::cout << " ... Video Streaming " <<
            "(Es. http://localhost:8080/?action=stream?dummy=param.mjpg)" <<std::endl;
        std::cout << " ... Axis PTZ (Es. http://192.168.0.90/mjpg/video.mjpg)" <<std::endl;
        std::cout << " ... Mini Axis Camera " <<
            "(Es. http://192.168.0.91/mjpg/video.mjpg?camera=1)" << std::endl;
        std::cout << " [6] Calibration intrinsec parameters file " <<
            "(Es. '../workspaces/CamCalibration/DataCalibration/DataCam_LG-N.txt" << std::endl;
        return false;
    }

    /** 
    * Initialiazation of the Robot Chain
    * Schunk PowerBall 
    **/
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0, -M_PI/2, 0.205, 0.0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.350, M_PI, 0.0, -M_PI/2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0, -M_PI/2, 0.0, -M_PI/2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0, M_PI/2,  0.305, 0.0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0, -M_PI/2, 0.0, 0.0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0.0, 0.0, 0.340, -0.261555)));
    // Schunk PowerCube//deprecated
    // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        // KDL::Frame::DH(0.0,M_PI/2, 0.270, 0.0)));
    // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        // KDL::Frame::DH(0.190, M_PI, 0.0, M_PI/2)));
    // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        // KDL::Frame::DH(0.0, M_PI/2, 0.0, M_PI/2)));
    // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        // KDL::Frame::DH(0.0, -M_PI/2,  0.252, 0.0)));
    // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        // KDL::Frame::DH(0.0, M_PI/2, 0.0, 0.0)));
    // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        // KDL::Frame::DH(0.0, 0.0, 0.203, 0.0)));

    /** camera opening chosen */
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

/**
* (DEMO) USED TO SHOW HOW MUCH STRONG IS THE DEPTH ESTIMATION 
* Moving arm using fix joints position and specific speed
* @option, parameter to indicate which position to send
*/
bool moveToNewPosition(int16_t option) {
    // std::cout << "MOVING ARM" << std::endl;
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
            command.joint[0] = 0.0906349;     // -1.51561;
            command.joint[1] = -0.108263;     // -0.0152716;
            command.joint[2] = 1.10513;       // 1.18539;
            command.joint[3] = -0.0491485;    // -0.0618894;
            command.joint[4] = 1.825;         // 1.81451;
            command.joint[5] = 0.00734784;    // 0.126257;
            command.joint[6] = 7.96417e-28;   // -4.7024e+11;
            command.joint[7] = -4.89565e+33;  // 3.99651e+17;
        break;
        case 2:
            command.joint[0] = 0.437484;      // -1.51561;
            command.joint[1] = -1.70679;      // -0.0152716;
            command.joint[2] = 1.17227;       // 1.18539;
            command.joint[3] = 0.51304;       // -0.0618894;
            command.joint[4] = -1.1851;       // 1.81451;
            command.joint[5] = -0.152472;     // 0.126257;
            command.joint[6] = 1.46714e-14;   // -4.7024e+11;
            command.joint[7] = 1.03194;       // 3.99651e+17;
        break;
        case 3:
            command.joint[0] = 0.0;  // 0.437484;      // -1.51561;
            command.joint[1] = 0.0;  // -1.70679;      // -0.0152716;
            command.joint[2] = 0.0;  // 1.17227;       // 1.18539;
            command.joint[3] = 0.0;  // 0.51304;       // -0.0618894;
            command.joint[4] = 0.0;  // -1.1851;       // 1.81451;
            command.joint[5] = 0.0;  // -0.152472;     // 0.126257;
            command.joint[6] = 0.0;  // 1.46714e-14;   // -4.7024e+11;
            command.joint[7] = 0.0;  // 1.03194;       // 3.99651e+17;
        break;
    } sleep(3);
    command.vel = 0.05;
    auto test =  _fifo_robot->write(command.serialize(), command.getHeader());

    return true;
}

/**
* Handler of the mouse
* It takes the event from mouse and chooses the option to carry out
* @event, event captured
* @x & @y, cursor coordinates from the event
*/
void onMouse(int event, int x, int y, int, void* param) {
    switch (event) {
        case CV_EVENT_LBUTTONDOWN:
            system("v4l2-ctl -d /dev/video0 -c focus_auto=1");      // auto focus ON

            _takingDimensions = _follow = _drawIt = true;
            _myArea.x = _areaAux.x = x;
            _myArea.y = _areaAux.y = y;
            break;
        case CV_EVENT_MOUSEMOVE:
            if (_mButtonPressed) {   /** changing the depth of the focus by the mouse movement */
                _focusLevel = _previousPosition/*.y*/ - y > 0 ? _focusLevel+0.5 : _focusLevel-0.51;
                system(("v4l2-ctl -d /dev/video0 -c focus_absolute="
                    + std::to_string(_focusLevel)).c_str());
                _previousPosition /*.y*/ = y;
            } else { if ( _follow ) calculateRoi(x, y); }
            break;
        case CV_EVENT_MBUTTONDOWN:
            system("v4l2-ctl -d /dev/video0 -c focus_auto=0");      // auto focus OFF
            _mButtonPressed = true;
            _previousPosition = y;
            /** the depth of the focus is calculated by 
            forward/backward mouse movement while the MBUTTON keeps pressed */
            break;
        case CV_EVENT_MBUTTONUP:
            _mButtonPressed = false;
            _focusLevel = 30;           /** average focus level */
            break;
        case CV_EVENT_LBUTTONUP:
            _takingDimensions = _follow = false;
            _choosedArea = true;
            _mySelectArea.setFromScratch(SIZE_AREA, _frame.cols,
                _frame.rows, cv::Point(-1, -1), _myArea);  // , _frame);
            break;
        case CV_EVENT_RBUTTONDOWN:
            system("v4l2-ctl -d /dev/video0 -c focus_auto=1");      // auto focus ON
            _disableFocus = true;
            _countDisable = 0;      /** timer for the output printing */

            /**testing how much strong is the system */
            _cont = 0;
            // moveToNewPosition(0);
            /**--------------------------------------*/

            /** stop and cleaning of the depth estimation */
            _mySelectArea.setExistPattern(false);
            _mySelectArea.setChoosedArea(false);
            _choosedArea = _drawIt = false;
            _myArea.x = _areaAux.x = 0;
            _myArea.y = _areaAux.y = 0;
            break;
    }
}

/** calculating the area of interest while the left mouse's button keeps pressed */
void calculateRoi(const int16_t x, const int16_t y) {  // NOLINT
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
                _myArea.x       = x;
                _myArea.height  = y - _myArea.y;
        } else if ( x > _areaAux.x && y < _areaAux.y ) {
            _myArea.height  = abs(y - _areaAux.y);
            _myArea.y       = y;
            _myArea.width   = x - _myArea.x;
        }
    }
}

/** Axis transformation */
Matrix<float> armPositionForwardKinematic(Packets::PacketArmJointByJointPositions packet) {
    Matrix<float> TK_06(4, 4, 0.0);
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::JntArray jointPositions = KDL::JntArray(6);
    for (int i=0; i < 6; i++) jointPositions(i) = static_cast<double>(packet.joint[i]);
    KDL::Frame taskpos;

    int kinematics_status = fksolver.JntToTask(jointPositions, taskpos);

    for (int i=0; i < 3; i++) {
        for (int k=0; k < 3; k++) TK_06(i, k) = taskpos.M(i, k);
    }

    for (int i=0; i < 3; i++) TK_06(i, 3) = taskpos.p[i];
    TK_06(3, 3) = 1;

    return TK_06;
}

void receiver() {
    std::string buffer;
    Packets::SchunkArmStatusPacket status_robot;
    Packets::PacketHeader header_mmap_robot;

    while (1) {
        usleep(2000);
        _mmap_robot->read(buffer, header_mmap_robot);
        status_robot.deserialize(buffer);

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
}

/** the area of interest is coming from the GUI */
void clickedAra() {
    std::string buffer;
    Packets::PacketHeader header_GUI;

    while (1) {
        _fifo_GUI->read(buffer, header_GUI);
        if (header_GUI.type == Packets::PACKET_AREA_CLICKED_COORDINATE_TYPE) {
            Packets::PacketAreaClickedCoordinate GUI_command;
            GUI_command.deserialize(buffer);

            cv::Point pt = cv::Point(GUI_command.x, GUI_command.y);
            std::cout << "x = "<< pt.x <<"  \t y=" << pt.y << std::endl;
            _myArea.x = _myArea.y = -1;
            _mySelectArea.setFromScratch(SIZE_AREA, _frame.cols, _frame.rows, pt, _myArea);
        }
    }
}

void runHomography() {
    std::string buffer;
    Packets::RobotArmStatusPacket status_robot;

    Packets::PacketHeader header_robot_status;
    header_robot_status.length = sizeof(Packets::RobotArmStatusPacket);
    header_robot_status.type = Packets::ROBOT_ARM_STATUS_PACKET_TYPE;

    while (1) {
        _mmap_robot->read(buffer, header_robot_status);  // getting ROBOT info (position, velocity,)
        status_robot.deserialize(buffer);

        if ( _mySelectArea.getExistPattern() && _mySelectArea.getFollowIt() ) {
            try {
                _mySelectArea.runHomography(_frame, status_robot.arm_position);
            } catch (cv::Exception e) {}
        }
    }
}

/** capturing the frame in a different thread to reach the real time image */
void captureFrame() {
  while (1) {
    try {
        _cap >> _frame;
        _mySelectArea.setFrame(_frame);
    } catch(cv::Exception e) {}
  }
}

int main(int argc, char ** argv) {
    if ( !initParams(argc, argv) ) return -1;

    _mySelectArea.setCameraParameters(argv[6]);

    _cap >> _frame;
    std::thread theCaptureFrame(captureFrame);

    cv::namedWindow("Scene", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("Scene", onMouse, &_mouseParam);  // mouse handler

    /** Robot needed variables */
    _fifo_GUI   = FIFO::CreateReaderPtr(argv[2]);   // reading the click's coordinate
    _fifo_robot = FIFO::CreateWriterPtr(argv[3]);   // sending command to the robot
    _mmap_robot = MMAP::CreateReaderPtr(argv[4]);   // reading the position of the robot

    Packets::SchunkArmStatusPacket status_robot;
    Packets::PacketHeader header_robot_status;

    header_robot_status.length = sizeof(Packets::PacketArmWorldCoordinate);
    header_robot_status.type = Packets::PACKET_ARM_WORLD_COORDINATE_TYPE;

    _header_tcp.length = sizeof(Packets::PacketArmJointByJointPositions);
    _header_tcp.type = Packets::PACKET_ARM_JOINT_BY_JOINT_POSITION_TYPE;

    Packets::PacketArmWorldCoordinate packet_velocity;
    packet_velocity.tcp = 1;
    for (int i = 0; i < 6; i++) packet_velocity.task_coordinate[i] = 0.0;

    int clickArea = 0;
    // _x_vel = _y_vel = 0;

    std::thread theHomography(runHomography);
    std::thread th_receiver(receiver);
    // std::thread theFifoGUI( clickedAra );     // getting the click from the GUI

    /**testing how much strong is the system */
    // moveToNewPosition(0);
    // moveToNewPosition(2);
    // moveToNewPosition(3);
    /**--------------------------------------*/

    float origing_joints[8];
    bool existJoints = false;
    std::string cs_py_cmd, buffer;
    while (1) {
        _mmap_robot->read(buffer, header_robot_status);  // getting ROBOT info (position, velocity,)
        status_robot.deserialize(buffer);
        auto robot_pose = status_robot.arm_position;
        auto robot_pose_joints = status_robot.joints_position;
        auto robot_manipulability = status_robot.manipulability;
        // std::cout << "///////////////////////////////////////////////////////////" << std::endl;
        // std::cout << "     >> " << robot_manipulability << " <<  " << std::endl;
        // std::cout << "///////////////////////////////////////////////////////////" << std::endl;
        if (!existJoints) {
            for (int i=0; i < 8; i++) origing_joints[i] = robot_pose_joints[i]; existJoints = true;
        }


        printMatrixs(origing_joints, robot_pose_joints, robot_pose);

        if (_mySelectArea.getChoosedArea()) {
            //////////// JUST TO FIND THE SCREW (but it is working the tracking) //////////////////
            try {
                _error = _mySelectArea.searchTheScrew(robot_pose);
            } catch(cv::Exception e) {}
            //////////////////////// JUST TO FIND THE SCREW////////////////////////////////////////
        } else {  // waiting for a user click showing a message
            if ( clickArea >= 0 && clickArea < 20 )
                putText(_frame, "PLEASE, SELECT THE INTEREST AREA", cvPoint(30, 60),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1.8, cvScalar(0, 0, 255), 1, CV_AA);
            if ( clickArea++ == 40 ) clickArea = 0;
        }

        // printing the message that manual focus is disabled
        if ( _disableFocus ) {
            _mySelectArea.manualFocusIsDisabled();
            if (_countDisable++ > 50) _disableFocus = false;
        }

        /**if for test how much strong is the system*/
        // if ( _mySelectArea.getChoosedArea() && ++_cont == 70 ) moveToNewPosition(1);
        /** --------------------------------------- */
        if ( !_mySelectArea.getExistPattern() && _drawIt || _choosedArea )
            cv::rectangle(_frame, _myArea, cv::Scalar(255, 0, 0), 2, 1);

        cv::imshow("Scene", _mySelectArea.getExistPattern() ? _mySelectArea.getFrame() : _frame);

        if ( cv::waitKey(20) == 27 ) break;
    }

    return 0;
}

void printMatrixs(float origing_joints[], float robot_pose_joints[], float robot_pose[][4]) {
    // PRINTING JOINTS TO DO THE DEMO
    std::cout << "JOINTS POSITION FOR DEMO: ";
    for (int i = 0; i < 8; i++) std::cout << origing_joints[i] << ", ";
    std::cout << std::endl;
    for (int i = 0; i < 8; i++) std::cout << robot_pose_joints[i] << ", ";
    std::cout << std::endl;

    std::cout << "{";
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++)
        std::cout << robot_pose[i][j] << ",";
      std::cout << std::endl;
    } std::cout << "}" << std::endl;

    std::cout << "ROBOT MATRIX: " << robot_pose << std::endl;

    float r = sqrt(pow(robot_pose[0][3], 2) + pow(robot_pose[1][3], 2));
    std::cout << "Radius : " << r << std::endl;
}
