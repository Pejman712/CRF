/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <memory>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "CollisionDetector/SpaceType.hpp"
#include "CollisionDetector/ICollisionDetector.hpp"
#include "CollisionDetector/FCLCollisionDetector/FCLCollisionDetector.hpp"
#include "CollisionDetector/FCLCollisionDetector/FCLCollisionDetectorUtility.hpp"

#define DEFAULT_MAP_RESOLUTION 0.05
#define TASK_SPACE_SIZE 6

using crf::actuators::robotbase::RobotBaseConfiguration;
using crf::navigation::collisiondetector::SpaceType;
using crf::navigation::collisiondetector::FCLCollisionDetector;
using crf::navigation::collisiondetector::ICollisionDetector;

class FCLCollisionDetectorShould: public ::testing::Test {
 protected:
    FCLCollisionDetectorShould(): logger_("FCLCollisionDetectorShould"),
    robotBaseConfiguration_(new RobotBaseConfiguration()) {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~FCLCollisionDetectorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    void SetUp() override {
        std::string testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("modules/"));
        testDirName_ += "modules/Actuators/CHARMBot/config/";

        std::ifstream goodConfigurationStream(testDirName_ + "CHARMBot.json");
        ASSERT_TRUE(robotBaseConfiguration_->parse(nlohmann::json::parse(goodConfigurationStream)));
        logger_->info("test SetUp done");
    }
    octomap::OcTree generateOcTree(double resolution) {
        octomap::OcTree tree(resolution);
        // insert some measurements of occupied cells
        for (int x = -20; x < 20; x++) {
            for (int y = -20; y < 20; y++) {
                for (int z = -20; z < 20; z++) {
                    tree.updateNode(octomap::point3d(x * 0.005, y * 0.005, z * 0.005), true);
                }
            }
        }
        // insert some measurements of free cells
        for (int x = -30; x < 30; x++) {
            for (int y = -30; y < 30; y++) {
                for (int z = -30; z < 30; z++) {
                    tree.updateNode(octomap::point3d(x*0.02+2.0, y*0.02+2.0, z*0.02+2.0), false);
                }
            }
        }
        return tree;
    }
    std::vector<SpaceType> createDefaultSE2SpaceTypes() {
        std::vector<SpaceType> stateTypes;
        stateTypes.push_back(SpaceType::REALVECTOR_STATE_SPACE);
        stateTypes.push_back(SpaceType::REALVECTOR_STATE_SPACE);
        stateTypes.push_back(SpaceType::SO2_STATE_SPACE);
        return stateTypes;
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<RobotBaseConfiguration> robotBaseConfiguration_;
    std::vector<SpaceType> stateTypes_;
    std::unique_ptr<ICollisionDetector> sut_;
};

TEST_F(FCLCollisionDetectorShould, throwExceptionWhenIncorrectDimensionsProvided) {
    auto robotBaseConfiguration = std::make_shared<RobotBaseConfiguration>();
    ASSERT_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration,
                                                     stateTypes_)), std::runtime_error);
}

TEST_F(FCLCollisionDetectorShould, failAllMethodsIfStatesSizeInconsistentWithStateTypes) {
    ASSERT_NO_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration_,
                                                     createDefaultSE2SpaceTypes())));
    octomap::OcTree newTree = generateOcTree(DEFAULT_MAP_RESOLUTION);
    ASSERT_TRUE(sut_->updateMap(newTree));

    std::vector<float> state1 = {.0, .0, .0, .0, .0, .0};
    ASSERT_FALSE(sut_->clearance(state1));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state1));
}

TEST_F(FCLCollisionDetectorShould, failAllMethodsIfOctomapMapEmpty) {
    ASSERT_NO_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration_,
                                                        createDefaultSE2SpaceTypes())));
    octomap::OcTree newTree(DEFAULT_MAP_RESOLUTION);
    ASSERT_FALSE(sut_->updateMap(newTree));

    std::vector<float> state1 = {.0, .0, .0};
    ASSERT_FALSE(sut_->clearance(state1));

    ASSERT_FALSE(sut_->checkState(state1));

    std::vector<float> state2 = {2.0, 2.0, M_PI/2};
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(FCLCollisionDetectorShould, notWorkForOnlyRealVectorSpaceInMoreThanSE2StateSpace) {
    std::vector<SpaceType> stateTypes;
    for (int i = 0; i < TASK_SPACE_SIZE; i++) {
        stateTypes.push_back(SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_NO_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration_,
                                                        createDefaultSE2SpaceTypes())));

    octomap::OcTree newTree = generateOcTree(DEFAULT_MAP_RESOLUTION);
    ASSERT_TRUE(sut_->updateMap(newTree));
    std::vector<float> state1 = {2.0, 2.0, .0, .0, .0, .0};
    ASSERT_FALSE(sut_->checkState(state1));
}

TEST_F(FCLCollisionDetectorShould, returnFalseIfCollisionWithTheEnviroment) {
    ASSERT_NO_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration_,
                                                        createDefaultSE2SpaceTypes())));
    octomap::OcTree newTree = generateOcTree(DEFAULT_MAP_RESOLUTION);
    logger_->info("Points were added to octomap pointcloud");
    ASSERT_TRUE(sut_->updateMap(newTree));

    std::vector<float> state1 = {0.01, .0, .0};
    ASSERT_FALSE(sut_->checkState(state1));

    std::vector<float> state2 = {1, 1, M_PI/2};
    ASSERT_TRUE(sut_->checkState(state2));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));

    std::vector<float> state3 = {1.5, 1.5, M_PI/2};
    ASSERT_TRUE(sut_->checkState(state3));
    ASSERT_TRUE(sut_->checkMotion(state2, state3));
}

TEST_F(FCLCollisionDetectorShould, returnInfinityIfNoObject) {
    ASSERT_NO_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration_,
                                                        createDefaultSE2SpaceTypes())));
    octomap::OcTree newTree = generateOcTree(DEFAULT_MAP_RESOLUTION);
    logger_->info("Points were added to octomap pointcloud");

    std::vector<float> state1 = {0, 0, 0};
    ASSERT_FALSE(sut_->clearance(state1));
    ASSERT_TRUE(sut_->updateMap(newTree));
    ASSERT_FALSE(sut_->checkState(state1));
}

TEST_F(FCLCollisionDetectorShould, returnCorrectDistanceToClosestObject) {
    float offsetFromWheelsDistance = 0.3;
    ASSERT_NO_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration_,
                                        createDefaultSE2SpaceTypes(), offsetFromWheelsDistance)));
    octomap::OcTree tree(DEFAULT_MAP_RESOLUTION);
    octomap::point3d point(5, 0, 0);
    tree.updateNode(point, true);
    auto params = robotBaseConfiguration_->getRobotParameters();
    ASSERT_TRUE(sut_->updateMap(tree));
    std::vector<float> state1 = {10, 0, 0};
    ASSERT_TRUE(sut_->checkState(state1));
    float halfLengthRobot = (params.wheelsDistanceY + offsetFromWheelsDistance)/2;
    auto clearanceValid = sut_->clearance(state1);
    ASSERT_TRUE(clearanceValid);
    auto distanceToObject = clearanceValid.get();
    ASSERT_NEAR((state1[0] - halfLengthRobot - point(0)), distanceToObject, 1e-1);
}

TEST_F(FCLCollisionDetectorShould, replaceCurrentMapWithNewOne) {
    ASSERT_NO_THROW(sut_.reset(new FCLCollisionDetector(robotBaseConfiguration_,
                                                        createDefaultSE2SpaceTypes())));
    octomap::OcTree tree(DEFAULT_MAP_RESOLUTION);
    tree.updateNode(octomap::point3d(5, 0, 0), true);
    ASSERT_TRUE(sut_->updateMap(tree));
    std::vector<float> state1 = {0.5, 0.5, M_PI/2};
    auto clearanceValid = sut_->clearance(state1);
    ASSERT_TRUE(clearanceValid);
    float resultDistanceMap1 = clearanceValid.get();
    ASSERT_TRUE(sut_->updateMap(tree));
    ASSERT_FLOAT_EQ(resultDistanceMap1, sut_->clearance(state1).get());
    tree.updateNode(octomap::point3d(3, 0, 0), true);
    ASSERT_TRUE(sut_->updateMap(tree));
    ASSERT_TRUE(std::abs(sut_->clearance(state1).get()-resultDistanceMap1) > 1e-1);
}
