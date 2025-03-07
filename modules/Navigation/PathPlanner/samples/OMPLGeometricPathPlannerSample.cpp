/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <fstream>

#include <nlohmann/json.hpp>

#include "PathPlanner/PathPlannerMethod.hpp"
#include "PathPlanner/OptimizerMethod.hpp"
#include "PathPlanner/OMPLGeometricPlanner/OMPLGeometricPlanner.hpp"

int main() {
    std::string file = __FILE__;
    file = file.substr(0, file.find("PathPlanner"));
    file += "PathPlanner/config/PlannerConfig.json";
    std::ifstream fstreamFile(file);
    nlohmann::json configFile = nlohmann::json::parse(fstreamFile);

    crf::navigation::pathplanner::OMPLGeometricPlanner planner(
        configFile,
        crf::navigation::pathplanner::PathPlannerMethod::RRTStar,
        crf::navigation::pathplanner::OptimizerMethod::PathLength);

    std::vector<double> start = {0, 0, 0};
    std::vector<double> goal = {1e3, 1e3, 2};
    auto path = planner.computePath(start, goal);

    if (!path) {
        std::puts("Path planner failed!");
        return -1;
    }
    std::puts("Path planner sucess!");

    std::cout << "Path size: " << path.value().size() << std::endl;

    for (uint64_t i = 0; i < path.value().size(); i++) {
        for (uint64_t j = 0; j < path.value()[i].size(); j++) {
            std::cout << path.value()[i][j] << ", ";
        }
        std::cout << std::endl;
    }

    return 0;
}
