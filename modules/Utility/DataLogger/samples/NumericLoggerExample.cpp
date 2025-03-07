/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <fstream>
#include <memory>
#include <vector>
#include <iostream>
#include <string>
#include <utility>
#include <unistd.h>

#include "DataLogger/InfluxLogger.hpp"
#include "DataLogger/DataPoint.hpp"

#include <nlohmann/json.hpp>


int main() {
  /*
   * The strings represent the input values for the selectDB,
   * dropDB, writer, reader and export
   */
  std::string mes1 = "JointPositions";
  std::string mes2 = "Radiation";
  std::string mes3 = "LHC";
  std::string mes4 = "stringer";
  std::string tag1 = "Schunk";
  std::string tag2 = "Schunk2";
  std::string tag3 = "Cavill";
  std::string tag5 = "Temperature";
  std::string DB  = "Schunk";
  std::string search = "ID ='Schunk' or ID = 'Schunk2'";
  std::string search2 = "ID ='Cavill'";
  std::string input = "ladies and gentlemen. We got em.";

  /*
   * fundamentalInput is a standard vector to be insereted into the db based on given
   * measurements and tag. typesInput is a standard initializer_list to be used in the
   * creation of the different types.
   */
  std::vector<float> fundamentalInput{1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  std::initializer_list<float> typesInput{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.1};

  crf::utility::types::JointPositions JPosition(typesInput);

  /*
   * For InfluxLogger constructor, the parameters are max line count before sending to db
   * and the amount of seconds (in float) for when to send if nothing is added to query.
   */
  auto *Test = new crf::utility::datalogger::InfluxLogger(350, 2.0);

  Test->dropDB(DB);    // drops a database with the name of BD, in case it already exists

  Test->selectDB(DB);  // creates a new Database with the name of DB

  for (size_t i = 0; i < 100; i++) {
    Test->writer(tag1, JPosition);  // writes to measurement JointPositions with tag schunk
    Test->writer(tag2, JPosition);  // writes to measurement JointPositions with tag schunk2
    Test->writer(mes2, tag1, fundamentalInput);  // writes to measurement Radiation with tag schunk
    Test->writer(mes3, tag5, 26.4);  // writes to measurement LHC wih tag Temperature
    Test->writer(mes4, tag3, input);
  }

  /*
   * string format for TIME must always be in format 'YYYY-MM-DDTHH:MM:SS.000000000Z'.
   * If no string is specified the the reader function will search all data
   * instread of searching in a timeframe.
   */
  auto timeFrame =
  std::make_pair("'2020-10-02T08:30:00.000000000Z'", "'2030-10-02T08:30:00.000000000Z'");

  std::vector<crf::utility::datalogger::dataPoint> arr =
  Test->reader(DB, mes1, search, timeFrame);

  std::vector<crf::utility::datalogger::dataPoint> arr2 =
  Test->reader(DB, mes4, search2);

  /*
   * Testing DataPoint's cast options. since everything from read is stored as a string.
   */
  std::vector<float> floatVec = arr[0].cast<float>();
  std::vector<int> intVec = arr[0].cast<int>();
  std::cout << "result from reader containing floats:";
  for (size_t i = 0; i < floatVec.size(); i++) {
    std::cout << " " <<floatVec[i];
}
  std::cout << "\nresult from reader containing strings: " <<  arr2[0].value_[0] << std::endl;

  crf::utility::types::JointVelocities testJoint =
  arr[0].castJoints<crf::utility::types::JointVelocities>();
  std::cout << "result from converting to JointPositions: "
  << testJoint(0) << ", "<< testJoint(1) << ", " << testJoint(2)
  << ", "<< testJoint(3) << ", " << testJoint(4) << ", "<< testJoint(5) << std::endl;

  int dummy = arr[0].castFundamental<int>();
  std::cout << "result from casting to fundamental type int: " << dummy << std::endl;

  /*
   * export test that will create a .json file in the current directory (build)
   * Format is the same as the one in read.
   */
  Test->Export(DB, mes1, search, timeFrame);

  delete Test;

  return 0;
}
