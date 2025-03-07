/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author:Sergio CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <boost/optional.hpp>
#include <pcl/common/transforms.h>

#include "EventLogger/EventLogger.hpp"
#include "Mapper3d/IMapper3d.hpp"
#include "Mapper3d/HandHeldMapper3d.hpp"

using crf::applications::mapper3d::HandHeldMapper3d;

using testing::_;
using testing::Return;

class HandHeldMapper3dShould: public ::testing::Test {
 protected:
    HandHeldMapper3dShould():
        logger_("HandHeldMapper3dShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("HandHeldMapper3dTest.cpp"));
        testDirName_ += "config/";
    }

    ~HandHeldMapper3dShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::unique_ptr<crf::applications::mapper3d::IMapper3d> sut_;
};

TEST_F(HandHeldMapper3dShould, throwExceptionWhenIncorrectConfigFileProvided) {
    ASSERT_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ + "configFile_badSyntax.json")),
        nlohmann::detail::parse_error);
    ASSERT_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ + "configFile_badKey.json")),
        nlohmann::detail::out_of_range);
    ASSERT_THROW(sut_.reset(new HandHeldMapper3d("configFile_badFileName.json")),
        nlohmann::detail::parse_error);
}

TEST_F(HandHeldMapper3dShould, notThrowExceptionWhenCorrectConfigFileProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_
        + "configFile_HandHeldGood.json")));
}

TEST_F(HandHeldMapper3dShould, throwExceptionWhenBadOctreeResolutionProvided) {
    ASSERT_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badOctreeResolution.json")),
        std::invalid_argument);
}

TEST_F(HandHeldMapper3dShould, throwExceptionWhenpassThroughMinLimitNotProvided) {
    ASSERT_THROW(sut_.reset(new HandHeldMapper3d(testDirName_
        + "configFile_badpassThroughMinLimit.json")),
        std::invalid_argument);
}

TEST_F(HandHeldMapper3dShould, notFilterInputCloudIfItIsNotOrganized) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_HandHeldGood.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr unOrganizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testUnOrganizedCloud.pcd", *unOrganizedCloud), -1);
    ASSERT_FALSE(sut_->updateMap(unOrganizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, FilterInputCloudIfItIsOrganized) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_HandHeldGood.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, DISABLED_notEstimateICPTransformIfBadNormalEstimationParametersProvided) {  // NOLINT
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_
        + "configFile_badNormalEstimationParameters.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould,
        DISABLED_notEstimateICPTransformIfBadNormalSamplingTotalPointsDivisionProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badNormalSamplingTotalPointsDivision.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, DISABLED_notEstimateICPTransformIfBadNormalSamplingBinSizeProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badNormalSamplingBinSize.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould,
        DISABLED_notEstimateICPTransformIfBadRandomlSamplingTotalPointsDivisionProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badRandomlSamplingTotalPointsDivision.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould,
        DISABLED_notEstimateICPTransformIfBadCorrespondencesRejectionParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badCorrespondencesRejectionParameters.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    transformation << 1, 0, 0, 1,
                 0, 1, 0, 1,
                 0, 0, 1, 1,
                 0, 0, 0, 1;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*organizedCloud, *transformedCloud, transformation);
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->updateMap(transformedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, DISABLED_estimateICPTransformIfGoofICPParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_HandHeldGood.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, DISABLED_notEstimateInitialTransformIfBadSubsamplingSideLengthProvided) {  // NOLINT
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badSubsamplingSideLength.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould,
        DISABLED_notEstimateInitialTransformIfBadNormalEstimationSearchRadiusProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badNormalEstimationSearchRadius.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, DISABLED_notEstimateInitialTransformIfBadFPFHParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badFPFHParameters.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, notReturnPointCloudIfBadVoxelGridParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badVoxelGridParameters.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, notUpdateMapdIfBadMLSFilterParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_badMLSFilterParameters.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_FALSE(sut_->updateMap(organizedCloud, cameraPose));
}

TEST_F(HandHeldMapper3dShould, DISABLED_ReturnPointCloudIfGodOutputFilterParametersProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_HandHeldGood.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_NE(sut_->getPointCloudMap(), boost::none);
}

TEST_F(HandHeldMapper3dShould, DISABLED_notSavePointCloudIfBadFilePathProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_HandHeldGood.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->savePointCloudToDisk("IncorrectPath.pcd", false));
}

TEST_F(HandHeldMapper3dShould, DISABLED_notSaveOctreeIfBadFilePathProvided) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_HandHeldGood.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
    ASSERT_FALSE(sut_->saveOctreeToDisk("IncorrectPath.ot"));
}

TEST_F(HandHeldMapper3dShould, DISABLED_updateOctreeIfPointCloudNotEmpty) {
    ASSERT_NO_THROW(sut_.reset(new HandHeldMapper3d(testDirName_ +
        "configFile_HandHeldGood.json")));
    Eigen::Matrix4f cameraPose(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr organizedCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (testDirName_+
        "../data/testOrganizedCloud.pcd", *organizedCloud), -1);
    ASSERT_TRUE(sut_->updateMap(organizedCloud, cameraPose));
}


