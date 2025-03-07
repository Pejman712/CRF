/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <utility>
#include <vector>
#include <memory>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "PathPlanner/OMPLGeometricPlanner/OMPLGeometricPlanner.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;

class OMPLGeometricPlannerShould: public ::testing::Test {
 protected:
    OMPLGeometricPlannerShould():
        logger_("OMPLGeometricPlannerShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~OMPLGeometricPlannerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() {
        {
            std::string file = __FILE__;
            file = file.substr(0, file.find("PathPlanner"));
            file += "PathPlanner/tests/config/TaskSpacePlanner.json";
            std::ifstream fstreamFile(file);
            taskConfigFile_ = nlohmann::json::parse(fstreamFile);
        }
        {
            std::string file = __FILE__;
            file = file.substr(0, file.find("PathPlanner"));
            file += "PathPlanner/tests/config/TaskSpacePlannerSE3.json";
            std::ifstream fstreamFile(file);
            taskConfigFileSE3_ = nlohmann::json::parse(fstreamFile);
        }
        {
            std::string file = __FILE__;
            file = file.substr(0, file.find("PathPlanner"));
            file += "PathPlanner/tests/config/6DoFJointSpacePlanner.json";
            std::ifstream fstreamFile(file);
            jointConfigFile_ = nlohmann::json::parse(fstreamFile);
        }
    }

    nlohmann::json taskConfigFile_;
    nlohmann::json taskConfigFileSE3_;
    nlohmann::json jointConfigFile_;

    std::unique_ptr<crf::navigation::pathplanner::IPathPlanner> sut_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(OMPLGeometricPlannerShould, testingTaskSpacePlanning) {
    sut_ = std::make_unique<crf::navigation::pathplanner::OMPLGeometricPlanner>(
        taskConfigFile_,
        crf::navigation::pathplanner::PathPlannerMethod::FMT,
        crf::navigation::pathplanner::OptimizerMethod::PathLength);

    // Real and SO3 quaternions (x, y, z, q1, q2, q3, ,q4)
    std::vector<double> start = {1, 1, 1, 0, 1, 0, 0};
    std::vector<double> goal = {1.5, 1, 0.5, 0, 0, 1, 0};

    auto path = sut_->computePath(start, goal);
    ASSERT_TRUE(path);
    ASSERT_EQ(path.value()[0], start);
    ASSERT_EQ(path.value()[path.value().size()-1], goal);
}

TEST_F(OMPLGeometricPlannerShould, testingJointSpacePlanning) {
    sut_ = std::make_unique<crf::navigation::pathplanner::OMPLGeometricPlanner>(
        jointConfigFile_,
        crf::navigation::pathplanner::PathPlannerMethod::RRTStar,
        crf::navigation::pathplanner::OptimizerMethod::PathLength);

    // joint space real and SO2: (q0, q1, q2, q3, q4, q5)
    std::vector<double> start = {0, 1.57, 4, 0, 3.14, 0};
    std::vector<double> goal = {3.14, 1, 1, 3, 3.14, 1.57};

    auto path = sut_->computePath(start, goal);
    ASSERT_TRUE(path);
    ASSERT_EQ(path.value()[0], start);
    ASSERT_EQ(path.value()[path.value().size()-1], goal);
}

TEST_F(OMPLGeometricPlannerShould, testAnotherOptimizationGoal) {
    sut_ = std::make_unique<crf::navigation::pathplanner::OMPLGeometricPlanner>(
        jointConfigFile_,
        crf::navigation::pathplanner::PathPlannerMethod::SPARS,
        crf::navigation::pathplanner::OptimizerMethod::PathLength);

    // joint space real and SO2: (q0, q1, q2, q3, q4, q5)
    std::vector<double> start = {0, 1.57, 4, 0, 3.14, 0};
    std::vector<double> goal = {3.14, 1, 1, 3, 3.14, 1.57};

    auto path = sut_->computePath(start, goal);
    ASSERT_TRUE(path);
    ASSERT_EQ(path.value()[0], start);
    ASSERT_EQ(path.value()[path.value().size()-1], goal);
}

TEST_F(OMPLGeometricPlannerShould, testAnotherMethod) {
    sut_ = std::make_unique<crf::navigation::pathplanner::OMPLGeometricPlanner>(
        taskConfigFileSE3_,
        crf::navigation::pathplanner::PathPlannerMethod::EST,
        crf::navigation::pathplanner::OptimizerMethod::PathLength);

    // Real and SO3 quaternions (x, y, z, q1, q2, q3, ,q4)
    std::vector<double> start = {1, 1, 1, 0, 1, 0, 0};
    std::vector<double> goal = {1.5, 1, 0.5, 0, 0, 1, 0};

    auto path = sut_->computePath(start, goal);
    ASSERT_TRUE(path);
    ASSERT_EQ(path.value()[0], start);
    ASSERT_EQ(path.value()[path.value().size()-1], goal);
}
