/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <experimental/filesystem>

#include <vector>
#include <fstream>
#include <memory>
#include <algorithm>
#include <string>
#include <sstream>
#include <cassert>

#include "EventLogger/EventLogger.hpp"
#include "DataLogger/InfluxLogger.hpp"
#include "DataLogger/DataPoint.hpp"
#include <curlpp/Exception.hpp>

using crf::utility::logger::EventLogger;
using crf::utility::datalogger::InfluxLogger;
using crf::utility::datalogger::dataPoint;

class DataLoggerShould : public ::testing::Test {
 protected:
  DataLoggerShould() :
       logger_("DataLoggerShould") {
           logger_->info("{} BEGIN",
               testing::UnitTest::GetInstance()->current_test_info()->name());
  }

  ~DataLoggerShould() {
       logger_->info("{} END with {}",
           testing::UnitTest::GetInstance()->current_test_info()->name(),
           testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
  }
  EventLogger logger_;
  std::unique_ptr<InfluxLogger> sut_;
};

/*
 * Testing for legal types of databas names to create
 */
TEST_F(DataLoggerShould, CreatedbReturnTrueOrFalseBasedOnInput) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(2, 3.0)));
  ASSERT_FALSE(sut_->selectDB(""));
  std::cout << "test1" << std::endl;
  ASSERT_TRUE(sut_->selectDB("Test"));
  std::cout << "test2" << std::endl;
  ASSERT_TRUE(sut_->selectDB("123"));
  std::cout << "test3" << std::endl;
  ASSERT_FALSE(sut_->selectDB("!@#$%^&"));
  std::cout << "test4" << std::endl;
}

/*
 * Testing for legal types of database names to drop
 */
TEST_F(DataLoggerShould, ReturnsTrueIfDBnameExists) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(2, 3.0)));
  sut_->selectDB("Test");
  ASSERT_FALSE(sut_->dropDB(""));
  ASSERT_TRUE(sut_->dropDB("test"));
  ASSERT_TRUE(sut_->dropDB("TEST"));
  ASSERT_TRUE(sut_->dropDB("Test"));
}

/*
 * Testing if jointPositions is inserted correctly into DB
 */
TEST_F(DataLoggerShould, JointPositions) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::JointPositions joint(v);
  ASSERT_TRUE(sut_->writer("mock", joint));

  std::cout << "test9" << std::endl;
  sut_->dropDB("Test");
}

/*
 * Testing if JointVelocities is inserted correctly into DB
 */
TEST_F(DataLoggerShould, JointVelocities) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::JointVelocities joint(v);
  ASSERT_TRUE(sut_->writer("mock", joint));
  sut_->dropDB("Test");
}

/*
 * Testing if JointForceTorques is inserted correctly into DB
 */
TEST_F(DataLoggerShould, JointForceTorques) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::JointForceTorques joint(v);
  ASSERT_TRUE(sut_->writer("mock", joint));
  sut_->dropDB("Test");
}

/*
 * Testing if JointAccelerations is inserted correctly into DB
 */
TEST_F(DataLoggerShould, JointAccelerations) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::JointAccelerations joint(v);
  ASSERT_TRUE(sut_->writer("mock", joint));
  sut_->dropDB("Test");
}

/*
 * Testing if TaskPose is inserted correctly into DB
 */
TEST_F(DataLoggerShould, TaskPose) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v1{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::TaskPose joint(v1);;
  ASSERT_TRUE(sut_->writer("mock", joint));
  sut_->dropDB("Test");
}

/*
 * Testing if TaskVelocity is inserted correctly into DB
 */
TEST_F(DataLoggerShould, TaskVelocity) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::TaskVelocity joint(v);
  ASSERT_TRUE(sut_->writer("mock", joint));
  sut_->dropDB("Test");
}

/*
 * Testing if TaskForceTorque is inserted correctly into DB
 */
TEST_F(DataLoggerShould, TaskForceTorque) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::TaskForceTorque joint(v);
  ASSERT_TRUE(sut_->writer("mock", joint));
  sut_->dropDB("Test");
}

/*
 * Testing if TaskAcceleration is inserted correctly into DB
 */
TEST_F(DataLoggerShould, TaskAcceleration) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::TaskAcceleration joint(v);
  ASSERT_TRUE(sut_->writer("mock", joint));
  sut_->dropDB("Test");
}

/*
 * Testing if Value is inserted correctly into DB
 */
TEST_F(DataLoggerShould, Value) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  float stub = 25.5;
  ASSERT_TRUE(sut_->writer("Radiation", "mock", stub));
  sut_->dropDB("Test");
}

/*
 * Testing if Values is inserted correctly into DB
 */
TEST_F(DataLoggerShould, Values) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::vector<float> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  ASSERT_TRUE(sut_->writer("Radiation", "mock", v));
  sut_->dropDB("Test");
}

/*
 * Testing if reader is called correctly from DATABASE
 */
TEST_F(DataLoggerShould, Reader) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger(1, 2.0)));
  sut_->selectDB("Test");
  std::initializer_list<float> input{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::JointPositions joint(input);
  sut_->writer("mack", joint);
  sleep(1);
  auto pair1 = std::make_pair("'2020-09-01T00:00:00.000000000Z'","'2025-09-10T00:00:00.000000000Z'"); //NOLINT
  auto dummy = sut_->reader("Test", "JointPositions", "ID = 'mack'", pair1);
  ASSERT_TRUE(dummy.empty());
  sut_->dropDB("Test");
}

/*

TEST_F(DataLoggerShould, Export) {
  ASSERT_NO_THROW(sut_.reset(new InfluxLogger<float>()));
  sut_->createDB("Test");
  std::initializer_list<float> input{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  crf::utility::types::JointPositions joint(input);
  auto pair1 = std::make_pair("'2020-09-01T00:00:00.000000000Z'","'2020-09-09T22:00:00.000000000Z'"); //NOLINT
  sut_->writer("Test", "mock", joint);
  sut_->Export("Test", "JointPositions", "ID = 'mock'", pair1);
  std::ifstream ifs("data.json", std::ifstream::in);
  ASSERT_FALSE(ifs.good());
  sut_->dropDB("Test");
}
*/
