/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Comparison.hpp"
#include "Rotation/JsonConverters.hpp"

#include <fstream>
#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::math::rotation::areAlmostEqual;

using crf::math::rotation::Rotation;
using crf::math::rotation::RotationRepresentation;

class JsonConvertersShould : public ::testing::Test {
 protected:
    JsonConvertersShould() :
        quaternion1_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        matrix1_(
            {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
             {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472},
             {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        angleAxis1_(
            1.9755068924749106,
            Eigen::Vector3d({-0.7621249848254679, 0.4328991822668132, 0.4814185346426796})),
        cardanXYZ1_({1.4, 2.0, 3.1}),
        eulerZXZ1_({1.1471123464639490, -1.6415867259093058, 0.1139727950424842}),
        rotationQuaternion1_(quaternion1_),
        rotationMatrix1_(matrix1_),
        rotationAngleAxis1_(angleAxis1_),
        rotationCardanXYZ1_(cardanXYZ1_),
        rotationEulerZXZ1_(eulerZXZ1_),
        jsonRotationQuaternion1_(),
        jsonRotationMatrix1_(),
        jsonRotationAngleAxis1_(),
        jsonRotationCardanXYZ1_(),
        jsonRotationEulerZXZ1_(),
        jsonRotationQuaternion1FromFile_(),
        jsonRotationMatrix1FromFile_(),
        jsonRotationAngleAxis1FromFile_(),
        jsonRotationCardanXYZ1FromFile_(),
        jsonRotationEulerZXZ1FromFile_(),
        outputJson_() {
        jsonRotationQuaternion1_["Quaternion"]["W"] = 0.5505666516902112;
        jsonRotationQuaternion1_["Quaternion"]["X"] = -0.6362152372281484;
        jsonRotationQuaternion1_["Quaternion"]["Y"] = 0.3613804315900029;
        jsonRotationQuaternion1_["Quaternion"]["Z"] = 0.4018839604029799;
        jsonRotationMatrix1_["Matrix"]["Row1"] = {
            0.4157869320692789, -0.9023592869214287, -0.1134413699981963};
        jsonRotationMatrix1_["Matrix"]["Row2"] = {
            -0.0173036611331485, -0.1325610914209059, 0.9910237839490472};
        jsonRotationMatrix1_["Matrix"]["Row3"] = {
            -0.9092974268256818, -0.4100917877109334, -0.0707312888348917};
        jsonRotationAngleAxis1_["AngleAxis"]["Angle"] = 1.9755068924749106;
        jsonRotationAngleAxis1_["AngleAxis"]["Axis"] = {
            -0.7621249848254679, 0.4328991822668132, 0.4814185346426796};
        jsonRotationCardanXYZ1_["CardanXYZ"]["X"] = 1.4;
        jsonRotationCardanXYZ1_["CardanXYZ"]["Y"] = 2.0;
        jsonRotationCardanXYZ1_["CardanXYZ"]["Z"] = 3.1;
        jsonRotationEulerZXZ1_["EulerZXZ"]["Z1"] = 1.1471123464639490;
        jsonRotationEulerZXZ1_["EulerZXZ"]["X"] = -1.6415867259093058;
        jsonRotationEulerZXZ1_["EulerZXZ"]["Z2"] = 0.1139727950424842;

        std::string rotationJsonPath = __FILE__;
        rotationJsonPath = rotationJsonPath.substr(0, rotationJsonPath.find("cpproboticframework"));
        rotationJsonPath += "cpproboticframework/modules/Math/Rotation/tests/config/Rotation.json";
        std::ifstream rotationJsonFile(rotationJsonPath);
        nlohmann::json rotationJson = nlohmann::json::parse(rotationJsonFile);

        jsonRotationQuaternion1FromFile_ = rotationJson["RotationQuaternion"];
        jsonRotationMatrix1FromFile_ = rotationJson["RotationMatrix"];
        jsonRotationAngleAxis1FromFile_ = rotationJson["RotationAngleAxis"];
        jsonRotationCardanXYZ1FromFile_ = rotationJson["RotationCardanXYZ"];
        jsonRotationEulerZXZ1FromFile_ = rotationJson["RotationEulerZXZ"];
    }

    void SetUp() override {
        outputJson_ = nlohmann::json();
    }

    const Eigen::Quaterniond quaternion1_;
    const Eigen::Matrix3d matrix1_;
    const Eigen::AngleAxisd angleAxis1_;
    const CardanXYZ cardanXYZ1_;
    const EulerZXZ eulerZXZ1_;

    const Rotation rotationQuaternion1_;
    const Rotation rotationMatrix1_;
    const Rotation rotationAngleAxis1_;
    const Rotation rotationCardanXYZ1_;
    const Rotation rotationEulerZXZ1_;

    nlohmann::json jsonRotationQuaternion1_;
    nlohmann::json jsonRotationMatrix1_;
    nlohmann::json jsonRotationAngleAxis1_;
    nlohmann::json jsonRotationCardanXYZ1_;
    nlohmann::json jsonRotationEulerZXZ1_;

    nlohmann::json jsonRotationQuaternion1FromFile_;
    nlohmann::json jsonRotationMatrix1FromFile_;
    nlohmann::json jsonRotationAngleAxis1FromFile_;
    nlohmann::json jsonRotationCardanXYZ1FromFile_;
    nlohmann::json jsonRotationEulerZXZ1FromFile_;

    nlohmann::json outputJson_;
};

TEST_F(JsonConvertersShould, GiveCorrectCardanXYZ) {
    ASSERT_TRUE(areAlmostEqual(jsonRotationCardanXYZ1_["CardanXYZ"].get<CardanXYZ>(), cardanXYZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectCardanXYZJson) {
    ASSERT_NO_THROW(outputJson_ = cardanXYZ1_);
    ASSERT_EQ(outputJson_, jsonRotationCardanXYZ1_["CardanXYZ"]);
}

TEST_F(JsonConvertersShould, GiveCorrectEulerZXZ) {
    ASSERT_TRUE(areAlmostEqual(jsonRotationEulerZXZ1_["EulerZXZ"].get<EulerZXZ>(), eulerZXZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectEulerZXZJson) {
    ASSERT_NO_THROW(outputJson_ = eulerZXZ1_);
    ASSERT_EQ(outputJson_, jsonRotationEulerZXZ1_["EulerZXZ"]);
}

TEST_F(JsonConvertersShould, GiveCorrectRotation) {
    ASSERT_TRUE(areAlmostEqual(jsonRotationQuaternion1_.get<Rotation>(), rotationQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(jsonRotationMatrix1_.get<Rotation>(), rotationMatrix1_));
    ASSERT_TRUE(areAlmostEqual(jsonRotationAngleAxis1_.get<Rotation>(), rotationAngleAxis1_));
    ASSERT_TRUE(areAlmostEqual(jsonRotationCardanXYZ1_.get<Rotation>(), rotationCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(jsonRotationEulerZXZ1_.get<Rotation>(), rotationEulerZXZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectRotationJson) {
    ASSERT_NO_THROW(outputJson_ = rotationQuaternion1_);
    ASSERT_EQ(outputJson_, jsonRotationQuaternion1_);

    ASSERT_NO_THROW(outputJson_ = rotationMatrix1_);
    ASSERT_EQ(outputJson_, jsonRotationMatrix1_);

    ASSERT_NO_THROW(outputJson_ = rotationAngleAxis1_);
    ASSERT_EQ(outputJson_, jsonRotationAngleAxis1_);

    ASSERT_NO_THROW(outputJson_ = rotationCardanXYZ1_);
    ASSERT_EQ(outputJson_, jsonRotationCardanXYZ1_);

    ASSERT_NO_THROW(outputJson_ = rotationEulerZXZ1_);
    ASSERT_EQ(outputJson_, jsonRotationEulerZXZ1_);
}

TEST_F(JsonConvertersShould, GiveCorrectCardanXYZFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonRotationCardanXYZ1FromFile_["CardanXYZ"].get<CardanXYZ>(), cardanXYZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectCardanXYZJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = cardanXYZ1_);
    ASSERT_EQ(outputJson_, jsonRotationCardanXYZ1FromFile_["CardanXYZ"]);
}

TEST_F(JsonConvertersShould, GiveCorrectEulerZXZFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonRotationEulerZXZ1FromFile_["EulerZXZ"].get<EulerZXZ>(), eulerZXZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectEulerZXZJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = eulerZXZ1_);
    ASSERT_EQ(outputJson_, jsonRotationEulerZXZ1FromFile_["EulerZXZ"]);
}

TEST_F(JsonConvertersShould, GiveCorrectRotationFromFile) {
    ASSERT_TRUE(
        areAlmostEqual(jsonRotationQuaternion1FromFile_.get<Rotation>(), rotationQuaternion1_));
    ASSERT_TRUE(areAlmostEqual(jsonRotationMatrix1FromFile_.get<Rotation>(), rotationMatrix1_));
    ASSERT_TRUE(
        areAlmostEqual(jsonRotationAngleAxis1FromFile_.get<Rotation>(), rotationAngleAxis1_));
    ASSERT_TRUE(
        areAlmostEqual(jsonRotationCardanXYZ1FromFile_.get<Rotation>(), rotationCardanXYZ1_));
    ASSERT_TRUE(areAlmostEqual(jsonRotationEulerZXZ1FromFile_.get<Rotation>(), rotationEulerZXZ1_));
}

TEST_F(JsonConvertersShould, GiveCorrectRotationJsonFromFile) {
    ASSERT_NO_THROW(outputJson_ = rotationQuaternion1_);
    ASSERT_EQ(outputJson_, jsonRotationQuaternion1FromFile_);

    ASSERT_NO_THROW(outputJson_ = rotationMatrix1_);
    ASSERT_EQ(outputJson_, jsonRotationMatrix1FromFile_);

    ASSERT_NO_THROW(outputJson_ = rotationAngleAxis1_);
    ASSERT_EQ(outputJson_, jsonRotationAngleAxis1FromFile_);

    ASSERT_NO_THROW(outputJson_ = rotationCardanXYZ1_);
    ASSERT_EQ(outputJson_, jsonRotationCardanXYZ1FromFile_);

    ASSERT_NO_THROW(outputJson_ = rotationEulerZXZ1_);
    ASSERT_EQ(outputJson_, jsonRotationEulerZXZ1FromFile_);
}
