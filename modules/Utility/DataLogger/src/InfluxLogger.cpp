/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <regex>
#include <utility>

#include <nlohmann/json.hpp>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Infos.hpp>

#include "DataLogger/DataPoint.hpp"
#include "DataLogger/InfluxLogger.hpp"

#define OK 200                  // Standard response for successful HTTP requests.
#define NOCONTENT 204           // The server successfully processed the request, and is not returning any content. //NOLINT
#define NOTFOUND 404            // Standard not found message (most likley the DB in this case)
#define CONTINUE 100            // Message recieved if an operation is large and is taking some time to execute // NOLINT
#define INTERALSERVERERROR 500  // Problems with the Influx Server Internally, check it's status

namespace crf {
namespace utility {
namespace datalogger {

InfluxLogger::~InfluxLogger() {
  stopThread_ = false;
  t1_.join();
  std::cout << "DataLogger finished running." << std::endl;
}

bool InfluxLogger::selectDB(const std::string& dbName) {
    std::string query = "q=CREATE DATABASE ";
    std::regex regx("[@_!#$%^&*()<>?/|}{~:]");
    if (!dbName.empty() && !regex_search(dbName, regx)) {
        currentdb_ = dbName;  // Sets the currentdb name to name of the new db
        newDB_ = true;
        try {
             curlpp::Cleanup cleaner;  // ensures RAII
             curlpp::Easy request;

             request.setOpt(new curlpp::options::Url(
                 "http://localhost:8086/query"));
                 request.setOpt(new curlpp::options::Verbose(true));

                 request.setOpt(new curlpp::options::PostFields(query + dbName));
                 request.setOpt(new curlpp::options::PostFieldSize(
                 query.length() + dbName.length()));

                 request.perform();  // Send request and get a result.
             }

        catch(curlpp::RuntimeError & e) {
            std::cout << e.what() << std::endl;
            return false;
        }

        catch(curlpp::LogicError & e) {
            std::cout << e.what() << std::endl;
            return false;
        }

        mtx_.lock();
        send();
        mtx_.unlock();

        newDB_ = false;
        return true;
    } else {
        std::cout << "ERROR" << std::endl;
        return false;
    }
}

bool InfluxLogger::dropDB(const std::string& dbName) {
    std::string query = "q=DROP DATABASE ";
    std::regex regx("[@_!#$%^&*()<>?/|}{~:]");
    if (!dbName.empty() && !regex_search(dbName, regx)) {
        try {
            curlpp::Cleanup cleaner;  // ensures RAII
            curlpp::Easy request;

            request.setOpt(new curlpp::options::Url(
                "http://localhost:8086/query"));
                request.setOpt(new curlpp::options::Verbose(true));

                request.setOpt(new curlpp::options::PostFields(query + dbName));
                request.setOpt(new curlpp::options::PostFieldSize(
                query.length() + dbName.length()));

                request.perform();  // Send request and get a result.
            }

        catch(curlpp::RuntimeError & e) {
            std::cout << e.what() << std::endl;
            return false;
        }

        catch(curlpp::LogicError & e) {
            std::cout << e.what() << std::endl;
            return false;
        }
        return true;
    } else {
    return false;
  }
}

bool InfluxLogger::Export(const std::string& dbName, const std::string& measurement,
    const std::string& tag, const std::pair<std::string, std::string>& timer) {
  std::string query = "q=SELECT * FROM " + measurement;
  std::regex regx("[@_!#$%^&*()<>?/|}{~:]");
  if (!dbName.empty() && !regex_search(dbName, regx)) {
    if (!tag.empty() && !timer.first.empty() && !timer.second.empty()) {
      query += " where " + tag + " and time >= " + timer.first + " and time <= "
      + timer.second + " tz('Europe/Zurich')";
    } else if (!tag.empty()) {
      query += " where " + tag;
    }

    try {
      curlpp::Cleanup cleaner;  // ensures RAII
      curlpp::Easy request;
      std::ostringstream respMsg;
      nlohmann::json j;

      request.setOpt(new curlpp::options::Url(
        "http://localhost:8086/query?db=" + dbName));
      request.setOpt(new curlpp::options::Verbose(true));

      request.setOpt(new curlpp::options::PostFields(query));
      request.setOpt(new curlpp::options::PostFieldSize(query.length()));
      request.setOpt(new curlpp::options::WriteStream(&respMsg));

      request.perform();  // Send request and get a result.

      if (curlpp::infos::ResponseCode::get(request) == OK) {
        std::cout << respMsg.str() << std::endl;
        j = nlohmann::json::parse(respMsg.str());

        std::ofstream out("./../build/data.json");
        out << std::setw(4) << j << std::endl;
        } else {
          return false;
        }
      }

    catch(curlpp::RuntimeError & e) {
      std::cout << e.what() << std::endl;
      return false;
    }

    catch(curlpp::LogicError & e) {
      std::cout << e.what() << std::endl;
      return false;
    }
    return true;
    } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& tag,
    const types::JointPositions& joints) {
  if (joints.size() > 0) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();
    mtx_.lock();
    query_ += "JointPositions,ID="+tag+" ";
    for (unsigned int i = 0; i < joints.size(); i++) {
        query_ += "Joint-" + std::to_string(i+1) + "-position" + "="
        + std::to_string(joints(i)) + ",";
    }

    query_.pop_back();  // removes the last ',' from query

    query_ += " " + timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
    } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& tag, const types::JointVelocities& joints) {
  if (joints.size() > 0) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += "JointVelocities,ID="+tag+" ";
    for (unsigned int i = 0; i < joints.size(); i++) {
        query_ += "Joint-" + std::to_string(i+1) + "-Velocity" + "=" +
        std::to_string(joints(i)) + ",";
    }

    query_.pop_back();  // removes the last ',' from query

    query_ += " " + timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
    } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& tag, const types::JointForceTorques& joints) {
  if (joints.size() > 0) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += "JointForceTorques,ID="+tag+" ";
    for (unsigned int i = 0; i < joints.size(); i++) {
        query_ += "Joint-" + std::to_string(i+1) + "-Torque" + "="
        + std::to_string(joints(i)) + ",";
    }

    query_.pop_back();  // removes the last ',' from query

    query_ += " " + timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
    } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& tag, const types::JointAccelerations& joints) {
  if (joints.size() > 0) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += "JointAccelerations,ID="+tag+" ";
    for (unsigned int i = 0; i < joints.size(); i++) {
        query_ += "Joint-" + std::to_string(i+1) + "-Acceleration" + "="
        + std::to_string(joints(i)) + ",";
    }

    query_.pop_back();  // removes the last ',' from query

    query_ += " " + timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
    } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& tag, const types::TaskPose& task) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += "TaskPose,ID="+tag+" "+"X="+ std::to_string(task(0))+
    ",Y="+ std::to_string(task(1))+",Z="+ std::to_string(task(2))+
    ",roll="+ std::to_string(task(3))+",pitch="+ std::to_string(task(4))+
    ",yaw="+ std::to_string(task(5)) + " " + timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
}

bool InfluxLogger::writer(const std::string& tag, const types::TaskVelocity& task) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += "TaskVelocity,ID="+tag+" "+"X-Velocity="+ std::to_string(task(0))+
    ",Y-Velocity="+ std::to_string(task(1))+",Z-Velocity="+ std::to_string(task(2))+
    ",roll-Velocity="+ std::to_string(task(3))+",pitch-Velocity="+
    std::to_string(task(4))+",yaw-Velocity="+ std::to_string(task(5)) + " " +
    timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
}

bool InfluxLogger::writer(const std::string& tag, const types::TaskForceTorque& task) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += "TaskForceTorque,ID="+tag+" "+"X-Torque="+ std::to_string(task(0))+
    ",Y-Torque="+ std::to_string(task(1))+",Z-Torque="+ std::to_string(task(2))+
    ",roll-Torque="+ std::to_string(task(3))+",pitch-Torque="+
    std::to_string(task(4))+ ",yaw-Torque="+ std::to_string(task(5)) + " " +
    timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
}

bool InfluxLogger::writer(const std::string& tag, const types::TaskAcceleration& task) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += "TaskAcceleration,ID="+tag+" "+"X-Acceleration="+ std::to_string(task(0))+
    ",Y-Acceleration="+ std::to_string(task(1))+",Z-Acceleration="+
    std::to_string(task(2))+ ",roll-Acceleration="+ std::to_string(task(3))+
    ",pitch-Acceleration="+ std::to_string(task(4))+
    ",yaw-Acceleration="+ std::to_string(task(5)) + " " + timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
}

bool InfluxLogger::writer(const std::string& measurement, const  std::string& tag,
  const bool& value) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += measurement+","+"ID="+tag+" "+"Value="+ std::to_string(value) + " " +timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
}

bool InfluxLogger::writer(const std::string& measurement, const std::string& tag,
  const float& value) {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanosec = now.time_since_epoch();

    std::stringstream ss;
    ss << nanosec.count();
    std::string timeStamp = ss.str();

    mtx_.lock();
    query_ += measurement+","+"ID="+tag+" "+"Value="+ std::to_string(value) + " " +timeStamp +"\n";

    lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
    mtx_.unlock();

    return true;
}

bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const double& value) {
        auto now = std::chrono::high_resolution_clock::now();
        auto nanosec = now.time_since_epoch();

        std::stringstream ss;
        ss << nanosec.count();
        std::string timeStamp = ss.str();

        mtx_.lock();
        query_ += measurement+","+"ID="+tag+" "+"Value="+ std::to_string(value) + " " +
        timeStamp +"\n";

        lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
        mtx_.unlock();

        return true;
}

bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const int& value) {
        auto now = std::chrono::high_resolution_clock::now();
        auto nanosec = now.time_since_epoch();

        std::stringstream ss;
        ss << nanosec.count();
        std::string timeStamp = ss.str();

        mtx_.lock();
        query_ += measurement+","+"ID="+tag+" "+"Value="+ std::to_string(value) + "i " +
        timeStamp +"\n";

        lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
        mtx_.unlock();

        return true;
}

bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const std::string& value) {
        auto now = std::chrono::high_resolution_clock::now();
        auto nanosec = now.time_since_epoch();

        std::stringstream ss;
        ss << nanosec.count();
        std::string timeStamp = ss.str();

        mtx_.lock();
        query_ += measurement+","+"ID="+tag+" "+"Value=\"" + value + "\" " + timeStamp +"\n";

        lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
        mtx_.unlock();

        return true;
}


bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const std::vector<bool>& value) {
        if (value.size() > 0) {
            auto now = std::chrono::high_resolution_clock::now();
            auto nanosec = now.time_since_epoch();

            std::stringstream ss;
            ss << nanosec.count();
            std::string timeStamp = ss.str();

            mtx_.lock();
            query_ += measurement+","+"ID="+tag+" ";
            for (unsigned int i = 0; i < value.size(); i++) {
                query_ += "value-" + std::to_string(i+1) + "=" + std::to_string(value[i]) + ",";
            }

            query_.pop_back();  // removes the last ',' from query

            query_ += " " + timeStamp +"\n";

            lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
            mtx_.unlock();

            return true;
        } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const std::vector<float>& value) {
        if (value.size() > 0) {
            auto now = std::chrono::high_resolution_clock::now();
            auto nanosec = now.time_since_epoch();

            std::stringstream ss;
            ss << nanosec.count();
            std::string timeStamp = ss.str();

            mtx_.lock();
            query_ += measurement+","+"ID="+tag+" ";
            for (unsigned int i = 0; i < value.size(); i++) {
                query_ += "value-" + std::to_string(i+1) + "=" + std::to_string(value[i]) + ",";
            }

            query_.pop_back();  // removes the last ',' from query

            query_ += " " + timeStamp +"\n";

            lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
            mtx_.unlock();

            return true;
        } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const std::vector<double>& value) {
        if (value.size() > 0) {
            auto now = std::chrono::high_resolution_clock::now();
            auto nanosec = now.time_since_epoch();

            std::stringstream ss;
            ss << nanosec.count();
            std::string timeStamp = ss.str();

            mtx_.lock();
            query_ += measurement+","+"ID="+tag+" ";
            for (unsigned int i = 0; i < value.size(); i++) {
                query_ += "value-" + std::to_string(i+1) + "=" + std::to_string(value[i]) + ",";
            }

            query_.pop_back();  // removes the last ',' from query

            query_ += " " + timeStamp +"\n";

            lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
            mtx_.unlock();

            return true;
        } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const std::vector<int>& value) {
        if (value.size() > 0) {
            auto now = std::chrono::high_resolution_clock::now();
            auto nanosec = now.time_since_epoch();

            std::stringstream ss;
            ss << nanosec.count();
            std::string timeStamp = ss.str();

            mtx_.lock();
            query_ += measurement+","+"ID="+tag+" ";
            for (unsigned int i = 0; i < value.size(); i++) {
                query_ += "value-" + std::to_string(i+1) + "=" + std::to_string(value[i]) + "i,";
            }

            query_.pop_back();  // removes the last ',' from query

            query_ += " " + timeStamp +"\n";

            lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
            mtx_.unlock();

            return true;
        } else {
    return false;
  }
}

bool InfluxLogger::writer(const std::string& measurement,
    const std::string& tag, const std::vector<std::string>& value) {
        if (value.size() > 0) {
            auto now = std::chrono::high_resolution_clock::now();
            auto nanosec = now.time_since_epoch();

            std::stringstream ss;
            ss << nanosec.count();
            std::string timeStamp = ss.str();

            mtx_.lock();
            query_ += measurement+","+"ID="+tag+" ";
            for (unsigned int i = 0; i < value.size(); i++) {
                query_ += "value-" + std::to_string(i+1) + "=\"" + value[i] + "\",";
            }

            query_.pop_back();  // removes the last ',' from query

            query_ += " " + timeStamp +"\n";

            lineCounter_++;  // counts the ammount of lines/datapoints currently in the query
            mtx_.unlock();

            return true;
        } else {
    return false;
  }
}

std::vector<dataPoint> InfluxLogger::reader(const std::string& dbName,
    const std::string& measurement, const std::string& tag,
    const std::pair<std::string, std::string>& timer) {
        std::vector<dataPoint> arr;
        std::string query = "q=SELECT * FROM " + measurement;
        if (!tag.empty() && !timer.first.empty() && !timer.second.empty()) {
            query += " where " + tag + " and time >= " + timer.first + " and time <= "
            + timer.second + " tz('Europe/Zurich')";
        } else if (!tag.empty()) {
            query += " where " + tag;
        }

    try {
        curlpp::Cleanup cleaner;  // ensures RAII
        curlpp::Easy request;
        std::ostringstream respMsg;  // checks the responsemessage of the curl command
        nlohmann::json j;

        request.setOpt(new curlpp::options::Url(
        "http://localhost:8086/query?db=" + dbName));
        request.setOpt(new curlpp::options::Verbose(true));

        request.setOpt(new curlpp::options::PostFields(query));
        request.setOpt(new curlpp::options::PostFieldSize(query.length()));
        request.setOpt(new curlpp::options::WriteStream(&respMsg));

        request.perform();  // Send request and get a result.

        if (curlpp::infos::ResponseCode::get(request) == OK) {
            std::cout << "response success" << std::endl;

            j = nlohmann::json::parse(respMsg.str());

            std::string id = j.at("results")[0].at("series")[0].at("name").get<std::string>();

            unsigned int countValues = j.at("results")[0].at("series")[0].at("values").size();
            unsigned int countElements = j.at("results")[0].at("series")[0].at("columns").size();

            std::cout << "ID OF SERIES IS: "
            << id << " AND NUMBER OF ELEMENTS ARE: "
            << countElements-2 << " AND NUMBER OF VALUES ARE: "
            <<  countValues <<std::endl;

            for (size_t k = 1; k < countValues+1; k++) {
                dataPoint data;
                data.id_ = std::to_string(k);
                arr.push_back(data);
            }

            for (size_t i = 0; i < countValues; i++) {
                auto params = j.at("results")[0].at("series")[0].at("values")[i];

                for (size_t j = 2; j < countElements; j++) {
                    arr[i].time_ = params[0];
                    try {
                        arr[i].value_.push_back((params[j]));
                    }
                    catch(nlohmann::json::exception& e) {
                        arr[i].value_.push_back(std::to_string(params[j].get<float>()));
                    }
                }
                for (size_t g = 0; g < countElements-2; g++) {
                    std::cout << "ID = " << arr[i].id_ << " and TIME = "
                    << arr[i].time_ << std::endl;
                    std::cout << "VALUES = " << arr[i].value_[g] << std::endl;
                }
            }

        } else {
            arr.clear();
            return arr;
        }
    }

    catch(curlpp::RuntimeError & e) {
        std::cout << e.what() << std::endl;
        arr.clear();
        return arr;
    }

    catch(curlpp::LogicError & e) {
        std::cout << e.what() << std::endl;
        arr.clear();
        return arr;
    }

    return arr;
}

bool InfluxLogger::send() {
    if (!query_.empty() && !currentdb_.empty()) {
        try {
            curlpp::Cleanup cleaner;  // ensures RAII
            curlpp::Easy request;
            std::ostringstream respMsg;  // checks the responsemessage of the curl command

            request.setOpt(new curlpp::options::Url(
            "http://localhost:8086/write?db="+currentdb_+"&precision=ns"));
            request.setOpt(new curlpp::options::Verbose(true));

            request.setOpt(new curlpp::options::PostFields(query_));
            request.setOpt(new curlpp::options::PostFieldSize(query_.length()));
            request.setOpt(new curlpp::options::WriteStream(&respMsg));

            request.perform();   // Send request and get a result.

            if (!query_.empty()) query_.clear();

            lineCounter_ = 0;

            if (curlpp::infos::ResponseCode::get(request) == 404) {
                std::cout << "Not Found" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            if (curlpp::infos::ResponseCode::get(request) == 204) {
                std::cout << "No Content" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            if (curlpp::infos::ResponseCode::get(request) == 100) {
                std::cout << "Continue" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            if (curlpp::infos::ResponseCode::get(request) == 500) {
                std::cout << "Internal server error" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            return true;
            std::cout << "sending complete" << std::endl;
        }

        catch(curlpp::RuntimeError & e) {
            std::cout << e.what() << std::endl;
            return false;
        }

        catch(curlpp::LogicError & e) {
            std::cout << e.what() << std::endl;
            return false;
        }
    } else {
        return false;
        std::cout << "sending failed" << std::endl;
    }
}

void InfluxLogger::senderThread() {
    auto start = std::chrono::high_resolution_clock::now();
    while (stopThread_) {
        auto end = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        if (lineCounter_ > lineCounterMax_ || time_span.count() > timerMax_ || newDB_ == true) {
            mtx_.lock();
            send();
            mtx_.unlock();
            start = std::chrono::high_resolution_clock::now();
        }
    }
}

}  // namespace datalogger
}  // namespace utility
}  // namespace crf
