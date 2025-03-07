/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <utility>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "CollisionDetector/GPUVoxelsCollisionDetector/GPUVoxelsCollisionDetector.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "CollisionDetector/SpaceType.hpp"

using testing::_;
using testing::Invoke;

class GPUVoxelsCollisionDetectorShould: public ::testing::Test {
 protected:
    GPUVoxelsCollisionDetectorShould(): logger_("GPUVoxelsCollisionDetectorShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("GPUVoxelsCollisionDetectorTests.cpp"));
        testDirName_ += "config/";
    }
    ~GPUVoxelsCollisionDetectorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::navigation::collisiondetector::GPUVoxelsCollisionDetector> sut_;
    std::string testDirName_;
};

TEST_F(GPUVoxelsCollisionDetectorShould, throwExceptionWhenRobotArmConfigurationIsNull) {
    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < 6; i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        nullptr,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_noEnvironment.json"))), std::runtime_error);
}

TEST_F(GPUVoxelsCollisionDetectorShould, throwExceptionWhenIncorrectConfigFileProvided) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "configFile_badFileName.json"),
        (testDirName_ + "GPUVoxelsTest.json"))), std::runtime_error);
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels.json"),
        (testDirName_ + "configFile_badFileName.json"))), std::runtime_error);

    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_badKey.json"),
        (testDirName_ + "GPUVoxelsTest.json"))), std::runtime_error);
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels.json"),
        (testDirName_ + "GPUVoxelsTest_badKey.json"))), std::runtime_error);

    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_badSyntax.json"),
        (testDirName_ + "GPUVoxelsTest.json"))), std::runtime_error);
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels.json"),
        (testDirName_ + "GPUVoxelsTest_badSyntax.json"))), std::runtime_error);
}

TEST_F(GPUVoxelsCollisionDetectorShould, throwExceptionWhenNumberOfDimensionsIsDifferent) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));


    std::vector<crf::navigation::collisiondetector::SpaceType> incorrectStateTypes;
    for (unsigned int i = 0; i <= robotArmConfig->getNumberOfJoints(); i++) {
        incorrectStateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        incorrectStateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels.json"),
        (testDirName_ + "GPUVoxelsTest.json"))), std::runtime_error);
}

TEST_F(GPUVoxelsCollisionDetectorShould, throwExceptionWhenTheModelsLocationIsIncorrect) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));
    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_badModelsLocation.json"),
        (testDirName_ + "GPUVoxelsTest.json"))), std::runtime_error);
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels.json"),
        (testDirName_ + "GPUVoxelsTest_badModelsLocation.json"))), std::runtime_error);
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfDimenionsNumberAreDifferent) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_noEnvironment.json"))));

    std::vector<float> correctState1 = {0, 0, 0};
    std::vector<float> incorrectState1 = {0, 0};
    ASSERT_TRUE(sut_->checkState(correctState1));
    ASSERT_FALSE(sut_->checkState(incorrectState1));

    std::vector<float> correctState2 = {M_PI/4, M_PI/4, M_PI/4};
    std::vector<float> incorrectState2 = {M_PI/2, M_PI/2, M_PI/2, M_PI/2};
    ASSERT_TRUE(sut_->checkMotion(correctState1, correctState2));
    ASSERT_FALSE(sut_->checkMotion(incorrectState1, correctState2));
    ASSERT_FALSE(sut_->checkMotion(correctState1, incorrectState2));
    ASSERT_FALSE(sut_->checkMotion(incorrectState1, incorrectState2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfSelfCollision) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_forcedSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_noEnvironment.json"))));

    std::vector<float> state1 = {0, 0, 0};
    ASSERT_FALSE(sut_->checkState(state1));

    std::vector<float> state2 = {M_PI/2, M_PI/2, M_PI/2};
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, throwExceptionWhenTheStateTypeIsNotSupported) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> incorrectStateTypes;
    incorrectStateTypes.push_back(
        crf::navigation::collisiondetector::SpaceType::UNKNOWN_STATE_SPACE);
    for (unsigned int i = 1; i < robotArmConfig->getNumberOfJoints(); i++) {
        incorrectStateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        incorrectStateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels.json"),
        (testDirName_ + "GPUVoxelsTest.json"))), std::runtime_error);
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfTheSO2StatesAreNotWithinTheBounds) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::SO2_STATE_SPACE);
    for (unsigned int i = 1; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_noEnvironment.json"))));

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {3*M_PI, M_PI/2, M_PI/2};
    ASSERT_FALSE(sut_->checkState(state2));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnTrueInsertingAPointCloud) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::SO2_STATE_SPACE);
    for (unsigned int i = 1; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }
    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_noEnvironment.json"))));

    std::string directory = __FILE__;
    directory = directory.substr(0, directory.find("GPUVoxelsCollisionDetectorTests.cpp"));
    directory += "models/";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZ> (directory + "LHCSection.pcd", *cloud), -1);
    ASSERT_TRUE(sut_->updateMap(cloud));

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/2, M_PI/4, M_PI/4};
    ASSERT_TRUE(sut_->checkState(state2));
    ASSERT_TRUE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, throwExceptionWhenStateTypesAndJointTypesDoesNotMatch) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reducedLinearType.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    for (unsigned int i = 1; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::SO2_STATE_SPACE);
    }
    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_noEnvironment.json"))), std::runtime_error);
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnTrueIfTheJointsTypeIsSetCorrectly) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reducedLinearType.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::SO2_STATE_SPACE);
    stateTypes.push_back(crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_Walls.json"))));

    std::vector<float> zero = {0, 0, 0};
    ASSERT_TRUE(sut_->checkState(zero));
    std::vector<float> changeRotationalJoint = {0, M_PI/4, 0};
    ASSERT_FALSE(sut_->checkState(changeRotationalJoint));
    std::vector<float> changeLinearJoint = {0, 0, M_PI/4};
    ASSERT_FALSE(sut_->checkState(changeLinearJoint));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentDetVoxelMapDetVoxelMap) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_DetVoxelMapDetVoxelMap.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentDetVoxelMapDetOctree) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_DetVoxelMapDetOctree.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentDetVoxelMapProbVoxelMap) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_DetVoxelMapProbVoxelMap.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentDetOctreeDetVoxelMap) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_DetOctreeDetVoxelMap.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentDetOctreeDetOctree) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_DetOctreeDetOctree.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentDetOctreeProbVoxelMap) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_DetOctreeProbVoxelMap.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentProbVoxelMapDetVoxelMap) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_ProbVoxelMapDetVoxelMap.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentProbVoxelMapDetOctree) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_ProbVoxelMapDetOctree.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, returnFalseIfCollWithEnviromentProbVoxelMapProbVoxelMap) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    std::vector<float> state1 = {0, 0, 0};
    std::vector<float> state2 = {M_PI/8, M_PI/8, M_PI/8};

    ASSERT_NO_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_forcedCollision_ProbVoxelMapProbVoxelMap.json"))));
    ASSERT_FALSE(sut_->checkState(state1));
    ASSERT_FALSE(sut_->checkMotion(state1, state2));
}

TEST_F(GPUVoxelsCollisionDetectorShould, throwExceptionIfMapRepresentationNotSuported) {
    std::shared_ptr<crf::robots::robotarm::RobotArmConfiguration> robotArmConfig =
        std::make_shared<crf::robots::robotarm::RobotArmConfiguration>();
    std::ifstream robotData(testDirName_ + "KinovaJaco2SW_reduced.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(robotArmConfig->parse(robotJSON));

    std::vector<crf::navigation::collisiondetector::SpaceType> stateTypes;
    for (unsigned int i = 0; i < robotArmConfig->getNumberOfJoints(); i++) {
        stateTypes.push_back(
            crf::navigation::collisiondetector::SpaceType::REALVECTOR_STATE_SPACE);
    }

    ASSERT_THROW(sut_.reset(new crf::navigation::collisiondetector::GPUVoxelsCollisionDetector(
        robotArmConfig,
        stateTypes,
        (testDirName_ + "KinovaJaco2SW3DModels_noSelfCollision.json"),
        (testDirName_ + "GPUVoxelsTest_NotImplementedMapType.json"))), std::runtime_error);
}
