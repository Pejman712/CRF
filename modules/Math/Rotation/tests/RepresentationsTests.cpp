/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Representations.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::math::rotation::RotationRepresentation;

class RotationRepresentationShould : public ::testing::Test {
 protected:
    RotationRepresentationShould() = default;
    std::unique_ptr<RotationRepresentation> sut_;
};

TEST_F(RotationRepresentationShould, HaveCorrectValues) {
    ASSERT_NO_THROW(sut_.reset(new RotationRepresentation(RotationRepresentation::Quaternion)));
    ASSERT_EQ(static_cast<std::size_t>(*sut_), 0);
    ASSERT_NO_THROW(sut_.reset(new RotationRepresentation(RotationRepresentation::Matrix)));
    ASSERT_EQ(static_cast<std::size_t>(*sut_), 1);
    ASSERT_NO_THROW(sut_.reset(new RotationRepresentation(RotationRepresentation::AngleAxis)));
    ASSERT_EQ(static_cast<std::size_t>(*sut_), 2);
    ASSERT_NO_THROW(sut_.reset(new RotationRepresentation(RotationRepresentation::CardanXYZ)));
    ASSERT_EQ(static_cast<std::size_t>(*sut_), 3);
    ASSERT_NO_THROW(sut_.reset(new RotationRepresentation(RotationRepresentation::EulerZXZ)));
    ASSERT_EQ(static_cast<std::size_t>(*sut_), 4);
}
