/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Prados CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <Eigen/Dense>
#include <vector>
#include <pcl/common/transforms.h>

#include "EventLogger/EventLogger.hpp"
#include "GraphOptimization/GraphOptimization.hpp"
#include "GraphOptimization/GraphOptimizationConfiguration.hpp"

using crf::applications::graphoptimization::GraphOptimization;
using crf::applications::graphoptimization::GraphOptimizationConfiguration;
using crf::applications::graphoptimization::NeighboursSearch;
using crf::applications::graphoptimization::Solvers;
using crf::applications::graphoptimization::Simulations;
using crf::applications::graphoptimization::SlamParameters;

using testing::_;
using testing::Return;

class GraphOptimizationShould: public ::testing::Test {
 protected:
    GraphOptimizationShould():
        logger_("GraphOptimizationShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("GraphOptimizationTest.cpp"));
        testDirName_ += "config/";
    }

    ~GraphOptimizationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::unique_ptr<GraphOptimization> sut_;
};

TEST_F(GraphOptimizationShould, throwExceptionWhenIncorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badSyntax.json"));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badKey.json"));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badFileName.json"));
}

TEST_F(GraphOptimizationShould, notThrowExceptionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadNeighborsNumberProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badNeighborsNumber.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadRadiusOfSearchProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badRadiusOfSearch.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadLambdaProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badLambda.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadTrialsProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badTrials.json"));
}

TEST_F(GraphOptimizationShould, testAllLinearSolvers) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_cholmod.json"));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_csparse.json"));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_eigen.json"));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_pcg.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenLinearSolverNotSpecified) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_noLinearSolver.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenNonLinearSolverNotSpecified) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_noNonLinearSolver.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadDoorsNumberProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badDoorsNumber.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadDoorsNameProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badDoorName.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadTransformationMatrixProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badTransformationMatrix.json"));
}

TEST_F(GraphOptimizationShould, throwExceptionWhenBadInitialPositionProvided) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_FALSE(sut_->parse(testDirName_ + "configFile_badInitialPosition.json"));
}

TEST_F(GraphOptimizationShould, testGettingDataFromFile) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    boost::optional <NeighboursSearch> neighboursData = sut_->getNeighbourgsSearchData();
    boost::optional <Solvers> solversData = sut_->getSolversData();
    boost::optional <Simulations> simulationsData = sut_->getSimulationsData();
    boost::optional <SlamParameters> slamData = sut_->getSlamData();
    boost::optional <Eigen::Matrix4f> tranformationData = sut_->getTransformationData();
    boost::optional <Eigen::Matrix4f> initialPose = sut_->getInitialPositionData();
    boost::optional <std::vector <std::vector <float>>> doorsPosition = sut_->getDoorsPosition();
    ASSERT_TRUE(neighboursData);
    ASSERT_TRUE(solversData);
    ASSERT_TRUE(simulationsData);
    ASSERT_TRUE(slamData);
    ASSERT_TRUE(tranformationData);
    ASSERT_TRUE(initialPose);
    ASSERT_TRUE(doorsPosition);
}

TEST_F(GraphOptimizationShould, wrongInputPosition) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    realMotion(2, 1) = realMotion(2, 1) + 0.2;
    Eigen::Matrix4f previousPosition = Eigen::Matrix4f::Identity();
    ASSERT_FALSE(sut_->addVertex(realMotion, previousPosition, boost::none,
        boost::none, boost::none));
}

TEST_F(GraphOptimizationShould, DISABLED_goodInputPositionTest) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousPosition = Eigen::Matrix4f::Identity();
    ASSERT_TRUE(sut_->addVertex(realMotion, previousPosition, boost::none,
        boost::none, boost::none));
}

TEST_F(GraphOptimizationShould, notParseAppliedTest) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousPosition = Eigen::Matrix4f::Identity();

    ASSERT_FALSE(sut_->addVertex(realMotion, previousPosition, boost::none,
        boost::none, boost::none));

    boost::optional <NeighboursSearch> neighboursData = sut_->getNeighbourgsSearchData();
    boost::optional <Solvers> solversData = sut_->getSolversData();
    boost::optional <Simulations> simulationsData = sut_->getSimulationsData();
    boost::optional <SlamParameters> slamData = sut_->getSlamData();
    boost::optional <Eigen::Matrix4f> tranformationData = sut_->getTransformationData();
    boost::optional <Eigen::Matrix4f> initialPose = sut_->getInitialPositionData();
    boost::optional <std::vector <std::vector <float>>> doorsPosition = sut_->getDoorsPosition();
    ASSERT_FALSE(neighboursData);
    ASSERT_FALSE(solversData);
    ASSERT_FALSE(simulationsData);
    ASSERT_FALSE(slamData);
    ASSERT_FALSE(tranformationData);
    ASSERT_FALSE(initialPose);
    ASSERT_FALSE(doorsPosition);
}

TEST_F(GraphOptimizationShould, DISABLED_saveGraphTest) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousPosition = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; i++) {
        ASSERT_TRUE(sut_->addVertex(realMotion, previousPosition, boost::none,
            boost::none, boost::none));
    }
    ASSERT_TRUE(sut_->saveGraph("testFile"));
}

TEST_F(GraphOptimizationShould, DISABLED_savePictureGraphTest) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousPosition = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; i++) {
        ASSERT_TRUE(sut_->addVertex(realMotion, previousPosition, boost::none,
            boost::none, boost::none));
    }
    ASSERT_TRUE(sut_->saveResults("testFile"));
}

TEST_F(GraphOptimizationShould, DISABLED_radiusSearchTest) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_radiusSearch.json"));
    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousPosition = Eigen::Matrix4f::Identity();
    Eigen::Vector3f fixPose;
    fixPose << 0, 0, 0;
    for (int i = 0; i < 4; i++) {
        ASSERT_TRUE(sut_->addVertex(realMotion, previousPosition, fixPose,
            fixPose, fixPose));
    }
}

TEST_F(GraphOptimizationShould, DISABLED_resetGraphTest) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));
    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousPosition = Eigen::Matrix4f::Identity();
    ASSERT_TRUE(sut_->addVertex(realMotion, previousPosition, boost::none,
        boost::none, boost::none));
    ASSERT_TRUE(sut_->resetGraph());
}

TEST_F(GraphOptimizationShould, DISABLED_testCompareOrganizedPointCloudsSeveralTimes) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_good.json"));

    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f SupposedMotion = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr previousPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());

    for (int i = 1; i < 4; i++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
        std::string name = testDirName_+ "../data/testOrganizedCloud.pcd";
        pcl::io::loadPCDFile<pcl::PointXYZRGBA> (name, *inPointCloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>());
        Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
        transformationMatrix(2, 3) = (i - 1) * 0.04;
        SupposedMotion(2, 3) = (i - 1) * 0.03;

        pcl::transformPointCloud(*inPointCloud, *transformedCloud, transformationMatrix);

        if (i != 1) {
            boost::optional<Eigen::Matrix4f> boostRealMotion = sut_->comparePointClouds(
                SupposedMotion, realMotion, transformedCloud, previousPointCloud);
            ASSERT_TRUE(boostRealMotion);
            realMotion = boostRealMotion.get();
        }

        ASSERT_TRUE(sut_->addVertex(realMotion, previousMotion, boost::none,
            boost::none, boost::none));

        previousPointCloud = inPointCloud;
        previousMotion = realMotion;
    }

    ASSERT_TRUE(sut_->getNodePositions());
    ASSERT_TRUE(sut_->getNodePointclouds());
}

TEST_F(GraphOptimizationShould, DISABLED_testCompareUnorganizedPointCloudsSeveralTimes) {
    ASSERT_NO_THROW(sut_.reset(new GraphOptimization(
        testDirName_ + "configFileMapper_good.json")));
    ASSERT_TRUE(sut_->parse(testDirName_ + "configFile_unorganized.json"));

    Eigen::Matrix4f realMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f previousMotion = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f SupposedMotion = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr previousPointCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr auxPointCloud
        (new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 1; i < 4; i++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
        std::string name = testDirName_+ "../data/testUnOrganizedCloud.pcd";
        pcl::io::loadPCDFile<pcl::PointXYZRGBA> (name, *inPointCloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>());
        Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
        transformationMatrix(2, 3) = -(i - 1) * 0.04;
        SupposedMotion(2, 3) = (i - 1) * 0.03;

        pcl::transformPointCloud(*inPointCloud, *transformedCloud, transformationMatrix);

        if (i != 1) {
            boost::optional<Eigen::Matrix4f> boostRealMotion = sut_->comparePointClouds(
                SupposedMotion, realMotion, inPointCloud, previousPointCloud);
            ASSERT_TRUE(boostRealMotion);
            realMotion = boostRealMotion.get();
        }

        ASSERT_TRUE(sut_->addVertex(realMotion, previousMotion, boost::none,
            boost::none, boost::none));

        previousPointCloud = transformedCloud;
        previousMotion = realMotion;
    }

    ASSERT_TRUE(sut_->getNodePositions());
    ASSERT_TRUE(sut_->getNodePointclouds());
}
