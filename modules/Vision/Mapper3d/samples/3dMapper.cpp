/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "IPC/MMAP.hpp"
#include "IPC/FIFO.hpp"
#include "Mapper3d/Mapper3d.hpp"
#include "SchunkArm/SchunkArmPackets.hpp"
#include "RobotArm/RobotArmPackets.hpp"
#include "ZMQ/Request.hpp"
#include "RGBDVisionUtility/PCLProtobufUtils.hpp"
#include "CameraViewer/CameraViewer.hpp"

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <fstream>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>
#include <visp3/vision/vpPose.h>
#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <vector>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
bool stopViz_ = false;
std::mutex mutex_;

typedef std::array<float, 6> JointPositions;

void visualization() {
    crf::applications::cameraviewer::CameraViewer viewer("3dmapper");

    while (!stopViz_) {
        mutex_.lock();
        if (cloud_->size() > 0) {
            viewer.viewPointCloud(cloud_, "RealSense");
        }
        mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char **argv) {
    if (argc < 7) {
        std::cout << "Few arguments provide:" <<std::endl;
        std::cout << "  [1] Name of MMAP  (Es. '/tmp/mmap_schunkarm')" <<std::endl;
        std::cout << "  [2] Name of FIFO  (Es. '/tmp/fifo_schunkarm')" <<std::endl;
        std::cout << "  [3] Path to trajectory Json file" << std::endl;
        std::cout << "  [4] Path to configuration Json file" << std::endl;
        std::cout << "  [5] Path to hand-eye transformation matrix" << std::endl;
        std::cout << "  [6] Camera server socket address" << std::endl;
        return -1;
    }

    // Read hand-eye transform from .txt
    vpHomogeneousMatrix M;
    Eigen::Matrix4f handeyeTransform;
    std::ifstream f(argv[5]);
    M.load(f);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            handeyeTransform(i, j) = M[i][j];
        }
    }
    std::cout << "Hand-eye transformation matrix: " << std::endl << handeyeTransform << std::endl;

    // Read trajectory positions from .json
    std::vector<JointPositions> position_vector;
    std::ifstream config(argv[3]);
    nlohmann::json jConfig;
    config >> jConfig;

    int positionsSize = jConfig.at("positionsSize").get<int>();

    for (int i = 0; i < positionsSize; i++) {
        JointPositions pos;
        for (int j = 0; j < 6; j++) {
            std::string position = "position" + std::to_string(i);
            std::string joint = "joint" + std::to_string(j);
            float jointPositions = jConfig.at("positions").at(position).at(joint).get<float>();
            pos[j] = jointPositions;
        }
        position_vector.push_back(pos);
    }

    std::shared_ptr<IPC> mmap;
    std::shared_ptr<IPC> fifo;
    mmap = MMAP::CreateReaderPtr(argv[1]);
    fifo = FIFO::CreateWriterPtr(argv[2]);

    fifo->open();
    mmap->open();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outPointCloud2
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    Eigen::Matrix4f cameraPose;

    std::string buffer;
    Packets::SchunkArmStatusPacket arm_status;
    Packets::PacketHeader header_status;

    crf::applications::mapper3d::Mapper3d merger(argv[4]);

    std::string socketAddress(argv[6]);
    crf::communication::zmqcomm::Request requester(socketAddress);
    requester.open();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_ = cloud;
    std::thread viz(&visualization);

    for (int i = 0; i < positionsSize; i++) {
        // Move robot to the next position
        Packets::PacketArmJointByJointPositions target_position;
        target_position.joint[0] = position_vector[i][0];
        target_position.joint[1] = position_vector[i][1];
        target_position.joint[2] = position_vector[i][2];
        target_position.joint[3] = position_vector[i][3];
        target_position.joint[4] = position_vector[i][4];
        target_position.joint[5] = position_vector[i][5];
        bool writeflag = fifo->write(target_position.serialize(), target_position.getHeader());
        if (!writeflag) {
            std::cout << "fifo write failed" <<std::endl;
            return false;
        }
        std::cout << "fifo write success" <<std::endl;
        std::cout << "MOVING TO THE NEXT POSITION" <<std::endl;
        sleep(5);

        // Capture pointcloud
        requester.write("PointCloud");
        std::string msg;
        requester.read(&msg);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud =
            crf::utility::rgbdvisionutility::PCLProtobufUtils::DeserializePointCloud(msg);
        std::cout << "PointCloud acquired" <<std::endl;

        // Capture pose and add hand eye transform
        Eigen::Matrix4f robot_pose;
        mmap->read(buffer, header_status);
        if (header_status.type == Packets::SchunkArmStatusPacket::type) {
            arm_status.deserialize(buffer);
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                robot_pose(j, k) = arm_status.arm_position[j][k];
                }
            }
        }
        cameraPose = robot_pose*handeyeTransform;

        // Update map
        if (!merger.updateMap(inPointCloud, cameraPose)) {
            std::cout << "Unable to update the map" <<std::endl;
        }

        outPointCloud2 = merger.getPointCloudMap().get();
        pcl::copyPointCloud(*outPointCloud2, *outPointCloud);
        std::cout << "waiting for lock" << std::endl;
        mutex_.lock();
        std::cout << "outPointCloud  size: " << outPointCloud->size() << " points" << std::endl;
        cloud_ = outPointCloud;
        mutex_.unlock();
        std::cout << "mutex_ unlock " << std::endl;
}

    requester.close();
    fifo->close();
    mmap->close();

    merger.savePointCloudToDisk("debugPC", true);
    merger.saveOctreeToDisk("debugOT");

    // If the letter 'q' is pressed the program process this frame pointcloud
    char key;
    bool endflag = false;
    std::cout << "KEYBOARD CONTROL"<< std::endl;
    while (!endflag) {
        std::cout << "Press 'q'+ ENTER to exit" << std::endl;
        std::cin >> key;
        if (key == 'q') endflag = true;
    }

    stopViz_ = true;
    viz.join();

    return 0;
}

