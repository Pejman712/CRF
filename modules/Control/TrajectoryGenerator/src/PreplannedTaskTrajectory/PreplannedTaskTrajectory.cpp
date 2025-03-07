/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <numeric>

#include "TrajectoryGenerator/PreplannedTaskTrajectory/PreplannedTaskTrajectory.hpp"

namespace crf::control::trajectorygenerator {

PreplannedTaskTrajectory::PreplannedTaskTrajectory(
    const std::string& trajectoryFile,
    const double& cycleTime):
    logger_("PreplannedTaskTrajectory"),
    file_(trajectoryFile),
    cycleTime_(cycleTime),
    index_(0),
    is6DTaskSpace_(true),
    content_{false, false, false},
    orientationType_(),
    taskPose_(),
    taskVelocity_(),
    taskAcceleration_(),
    cycle_(0),
    quantityOfCycles_(0) {
    logger_->debug("CTor");
    if (cycleTime_ <= 0.0) {
        throw std::invalid_argument("Cycle time must be greater than 0.");
    }
    // Open csv file
    if (!file_.is_open()) {
        throw std::invalid_argument("Wrong path: " + trajectoryFile);
    }
    // Get the names line
    std::string line, title;
    std::getline(file_, line);
    // Store the names in a vector
    std::stringstream str(line);
    std::vector<std::string> titles;
    while (std::getline(str, title, ',')) {
        titles.push_back(title);
    }
    if (titles.size() == 0) {
        throw std::invalid_argument("Empty file: " + trajectoryFile);
    }

    // Check which data is inside the csv file
    // The variable counter is equal to -1 because the loop adds 1 to it in the first iteration.
    uint64_t counter = -1;
    for (uint64_t i = 0; i < taskSpace6D_.size(); i++) {
        counter++;
        if (!matchHeader(titles, taskSpace6D_[i])) continue;
        content_[counter] = true;
        if (!matchHeaderWIndex(titles, taskSpace6D_[i])) {
            throw std::invalid_argument("Wrong order of the headers " +
                std::accumulate(begin(taskSpace6D_[i]), end(taskSpace6D_[i]), std::string{}));
        }
        // If this is not a position we continue
        if (i != 0) continue;
        for (uint64_t j = 0; j < orientations_.size(); j++) {
            index_ = taskSpace6D_[0].size();
            if (!matchHeader(titles, orientations_[j])) continue;
            if (!matchHeaderWIndex(titles, orientations_[j])) {
                throw std::invalid_argument("Wrong order of the orientation headers " +
                    std::accumulate(begin(orientations_[j]), end(orientations_[j]), std::string{}));
            }
            orientationType_ = j;
            break;
        }
    }
    if (index_ == 0) {
        throw std::invalid_argument("Incorrect input headers.");
        is6DTaskSpace_ = false;
    }
    if (index_ != titles.size()) {
        throw std::invalid_argument("There is an error in the titles: " + line +
            ".\nContent detected: Pose->" + std::to_string(content_[0]) + ", Velocity->" +
            std::to_string(content_[1]) + ", Acceleration->" + std::to_string(content_[2]));
    }

    // Read data from csv
    std::vector<std::vector<double>> data;
    std::vector<double> row;
    std::string word;
    while (getline(file_, line)) {
        row.clear();
        std::stringstream str2(line);
        while (getline(str2, word, ',')) {
            row.push_back(std::stod(word));
        }
        if (row.size() != index_) {
            throw std::invalid_argument("There is a mismatch in the number of row elements.");
        }
        data.push_back(row);
    }

    // Storing classified data
    quantityOfCycles_ = data.size();
    crf::math::rotation::OrientationRepresentation posRepresentation;
    posRepresentation = crf::math::rotation::OrientationRepresentation::Quaternion;
    if (content_[0]) {  // if (isTherePosition)
        taskPose_.resize(quantityOfCycles_);
        if (is6DTaskSpace_) {
            if (orientationType_ == 0) {
                posRepresentation = crf::math::rotation::OrientationRepresentation::Quaternion;
                // posElements = 7;
            } else if (orientationType_ == 1) {
                posRepresentation = crf::math::rotation::OrientationRepresentation::AngleAxis;
                // posElements = 7;
            } else if (orientationType_ == 2) {
                posRepresentation = crf::math::rotation::OrientationRepresentation::CardanXYZ;
                // posElements = 6;
            } else if (orientationType_ == 3) {
                posRepresentation = crf::math::rotation::OrientationRepresentation::EulerZXZ;
                // posElements = 6;
            } else {
                throw std::invalid_argument("Incorrect number of columns.");
            }
            // posDimensions = 6;
        } else {
             throw std::invalid_argument("Incorrect number of columns.");
        }
    }
    uint64_t velElements = 0;
    if (content_[1]) {  // if (isThereVelocity)
        taskVelocity_.resize(quantityOfCycles_);
        if (is6DTaskSpace_) {
            velElements = 6;
        } else {
            throw std::invalid_argument("Incorrect number of velocity columns.");
        }
    }
    uint64_t accElements = 0;
    if (content_[2]) {  // if (isThereAcceleration)
        taskAcceleration_.resize(quantityOfCycles_);
        if (is6DTaskSpace_) {
            accElements = 6;
        } else {
            throw std::invalid_argument("Incorrect number of acceleration columns.");
        }
    }

    for (uint64_t i = 0; i < quantityOfCycles_; i++) {
        uint64_t index = 0;
        if (content_[0]) {  // if (isTherePosition)
            taskPose_[i] = TaskPose();
            Eigen::Vector3d position(
                eigenVectorFromStdVector(
                    std::vector<double>(data[i].begin(), data[i].begin() + 3)));
            Eigen::Quaterniond quaternion;
            Eigen::AngleAxisd angleAxis;
            CardanXYZ cardanXYZ;
            EulerZXZ eulerZXZ;
            switch (posRepresentation) {
                case OrientationRepresentation::Quaternion:
                    quaternion.w() = data[i][3];
                    quaternion.x() = data[i][4];
                    quaternion.y() = data[i][5];
                    quaternion.z() = data[i][6];
                    taskPose_[i] = TaskPose(position, quaternion);
                    break;
                case OrientationRepresentation::AngleAxis:
                    angleAxis.angle() = data[i][3];
                    angleAxis.axis()(0) = data[i][4];
                    angleAxis.axis()(1) = data[i][5];
                    angleAxis.axis()(2) = data[i][6];
                    taskPose_[i] = TaskPose(position, angleAxis);
                    break;
                case OrientationRepresentation::CardanXYZ:
                    cardanXYZ[0] = data[i][3];
                    cardanXYZ[1] = data[i][4];
                    cardanXYZ[2] = data[i][5];
                    taskPose_[i] = TaskPose(position, cardanXYZ);
                    break;
                case OrientationRepresentation::EulerZXZ:
                    eulerZXZ[0] = data[i][3];
                    eulerZXZ[1] = data[i][4];
                    eulerZXZ[2] = data[i][5];
                    taskPose_[i] = TaskPose(position, eulerZXZ);
                    break;
                default:
                    throw std::logic_error(
                        "RotationRepresentation invalid");
                    break;
            }
        }
        if (content_[1]) {  // if (isThereVelocity)
            for (uint64_t j = 0; j < velElements; j++) {
                taskVelocity_[i][j] = data[i][index];
                index++;
            }
        }
        if (content_[2]) {  // if (isThereAcceleration)
            for (uint64_t j = 0; j < accElements; j++) {
                taskAcceleration_[i][j] = data[i][index];
                index++;
            }
        }
    }
}

PreplannedTaskTrajectory::~PreplannedTaskTrajectory() {
    logger_->debug("DTor");
}

void PreplannedTaskTrajectory::setInitialPose(const TaskPose& initialPose) {
    logger_->debug("setInitialPose");
}

void PreplannedTaskTrajectory::append(const std::vector<TaskPose>& path) {
    logger_->debug("append");
    logger_->warn("No path is being appended. "
        "Function not implemented for PreplannedTaskTrajectory class.");
}

void PreplannedTaskTrajectory::setProfileVelocity(const TaskVelocity& vel) {
    logger_->debug("setProfileVelocity");
    // TODO(lrodrigo) Define the desired behavior
    throw std::runtime_error("Function not available for PreplannedTaskTrajectory class");
}

void PreplannedTaskTrajectory::setProfileAcceleration(const TaskAcceleration& acc) {
    logger_->debug("setProfileAcceleration");
    // TODO(lrodrigo) Define the desired behavior
    throw std::runtime_error("Function not available for PreplannedTaskTrajectory class");
}

void PreplannedTaskTrajectory::reset() {
    logger_->debug("reset");
    cycle_ = 0;
}

void PreplannedTaskTrajectory::clearMemory() {
    logger_->debug("clearMemory");
    logger_->warn("No memory is being cleared. "
        "Function not implemented for PreplannedTaskTrajectory class.");
}

bool PreplannedTaskTrajectory::isTrajectoryRunning() {
    logger_->debug("isTrajectoryRunning");
    return cycle_ < quantityOfCycles_;
}

TaskSignals PreplannedTaskTrajectory::getTrajectoryPoint(double Tp) {
    logger_->debug("getTrajectoryPoint {}", Tp);

    // TODO(lrodrigo) Do we want to check Tp input respect to the sampling time?
    //                If not, delete constructor input cycleTime and the following lines.
    //                If yes, think a proper method. The following lines are not working properly.
    //                Fix the corresponding unit test.
    /*
    if (static_cast<uint64_t>(Tp/cycleTime_) != cycle_) {
        throw std::runtime_error("Wrong input time: " + std::to_string(Tp) +
            ". The last evaluation point was " + std::to_string(cycle_*cycleTime_) +
            ", and the cycle time is " + std::to_string(cycleTime_));
    }
    */
    uint64_t cycle = cycle_;
    if (cycle_ > (quantityOfCycles_ - 1)) cycle = quantityOfCycles_ - 1;
    TaskSignals result;
    // if (isTherePosition)
    if (content_[0]) result.pose = taskPose_[cycle];
    // if (isThereVelocity)
    if (content_[1]) result.velocity = taskVelocity_[cycle];
    // if (isThereAcceleration)
    if (content_[2]) result.acceleration = taskAcceleration_[cycle];
    cycle_++;
    return result;
}


bool PreplannedTaskTrajectory::matchHeader(const std::vector<std::string>& fileHeader,
    const std::vector<std::string>& referenceHeader) {
    for (uint64_t i = 0; i < referenceHeader.size(); i++) {
        auto resutl = std::find(fileHeader.begin(), fileHeader.end(), referenceHeader[i]);
        if (resutl == fileHeader.end()) return false;
    }
    return true;
}

bool PreplannedTaskTrajectory::matchHeaderWIndex(const std::vector<std::string>& fileHeader,
    const std::vector<std::string>& referenceHeader) {
    int counter = 0;
    for (uint64_t i = index_; i < index_ + referenceHeader.size(); i++) {
        if (fileHeader[i] != referenceHeader[counter]) return false;
        counter++;
    }
    index_ += counter;
    return true;
}

}  // namespace crf::control::trajectorygenerator
