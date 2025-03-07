/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <exception>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>
#include <Eigen/Dense>

#include "GraphOptimization/GraphOptimizationConfiguration.hpp"

namespace crf {
namespace applications {
namespace graphoptimization {

GraphOptimizationConfiguration::GraphOptimizationConfiguration():
    log_("GraphOptimizationConfiguration"),
    alreadyParsed_{false},
    neighboursData_{},
    solversData_{},
    simulationsData_{},
    slamData_{},
    transfomation_{},
    doorsPosition_{} {
        log_->debug("CTor");
    }

bool GraphOptimizationConfiguration::parse(const std::string& configFileName) {
    std::ifstream config(configFileName);
    if ((config.rdstate() & std::ifstream::failbit) != 0) {
        return false;
    }
    nlohmann::json jConfig;
    try {
        config >> jConfig;

        // neighboursData
        int knnMax = jConfig.at("kNN").at("value").get<int>();
        if (knnMax <= 0) {
            throw std::invalid_argument("Knn should be greater or equal to 1");
        }
        neighboursData_.value = knnMax;
        bool closestSearch = jConfig.at("kNN").at("closestPoints").get<bool>();
        if (closestSearch) {
            neighboursData_.type = NeighboursSearch::SearchType::Closest;
        } else {
            neighboursData_.type = NeighboursSearch::SearchType::Radius;
        }
        float radius = jConfig.at("kNN").at("radius").get<float>();
        if (radius < 0) {
            throw std::invalid_argument("The search radius should be positive");
        }
        neighboursData_.radius = radius;

        // solversData_
        bool dense = jConfig.at("solver").at("dense").get<bool>();
        bool cholmod = jConfig.at("solver").at("cholmod").get<bool>();
        bool csparse = jConfig.at("solver").at("csparse").get<bool>();
        bool eigen = jConfig.at("solver").at("eigen").get<bool>();
        bool pcg = jConfig.at("solver").at("pcg").get<bool>();
        if (dense) {
            solversData_.linearType = Solvers::LinearSolverType::Dense;
        } else if (cholmod) {
            solversData_.linearType = Solvers::LinearSolverType::Choldmod;
        } else if (csparse) {
            solversData_.linearType = Solvers::LinearSolverType::Csparse;
        } else if (eigen) {
            solversData_.linearType = Solvers::LinearSolverType::Eigen;
        } else if (pcg) {
            solversData_.linearType = Solvers::LinearSolverType::Pcg;
        } else {
            throw std::invalid_argument("A linear solver must be selected");
        }

        // optimizer configuration
        bool levenberg = jConfig.at("optimizer").at("type").at("levenberg").get<bool>();
        bool gaussNewton = jConfig.at("optimizer").at("type").at("gaussNewton").get<bool>();
        if (levenberg) {
            solversData_.nonLinearType = Solvers::NonLinearSolverType::Levenberg;
        } else if (gaussNewton) {
            solversData_.nonLinearType = Solvers::NonLinearSolverType::GaussNewton;
        } else {
            throw std::invalid_argument("A non-linear solver must be selected");
        }
        solversData_.lambdaSelection =
            jConfig.at("optimizer").at("levenberg").at("lambdaSelection").get<bool>();
        double lambda = jConfig.at("optimizer").at("levenberg").at("lambda").get<double>();
        if (lambda < 0) {
            throw std::invalid_argument("Lambda should be positive");
        }

        solversData_.trialsSelection =
            jConfig.at("optimizer").at("levenberg").at("trialsSelection").get<bool>();
        int trialAfterFailure =
            jConfig.at("optimizer").at("levenberg").at("trialAfterFailure").get<int>();
        if (trialAfterFailure < 0) {
            throw std::invalid_argument("trialAfterFailure should be positive");
        }
        solversData_.trials = trialAfterFailure;
        solversData_.verbose = jConfig.at("optimizer").at("optimizerVerbose").get<bool>();
        solversData_.iterations = jConfig.at("optimizer").at("iterations").get<unsigned int>();
        solversData_.robustKernel = jConfig.at("optimizer").at("robustKernel").get<bool>();

        // simulationsData_;
        simulationsData_.noiseX = jConfig.at("odometrySimulation").at("noise").at("x").get<float>();
        simulationsData_.noiseY = jConfig.at("odometrySimulation").at("noise").at("y").get<float>();
        simulationsData_.noiseZ = jConfig.at("odometrySimulation").at("noise").at("z").get<float>();
        simulationsData_.speedX = jConfig.at("odometrySimulation").at("speed").at("x").get<float>();
        simulationsData_.speedY = jConfig.at("odometrySimulation").at("speed").at("y").get<float>();
        simulationsData_.speedZ = jConfig.at("odometrySimulation").at("speed").at("z").get<float>();

        // slamData_
        slamData_.threshold = jConfig.at("threshold").at("value").get<float>();
        slamData_.visualization = jConfig.at("visualization").get<bool>();
        slamData_.organizedPointCloud = jConfig.at("organizedPointCloud").get<bool>();

        transfomation_.M00 = jConfig.at("frameTransformation").at("R0")[0].get<float>();
        transfomation_.M01 = jConfig.at("frameTransformation").at("R0")[1].get<float>();
        transfomation_.M02 = jConfig.at("frameTransformation").at("R0")[2].get<float>();
        transfomation_.M10 = jConfig.at("frameTransformation").at("R1")[0].get<float>();
        transfomation_.M11 = jConfig.at("frameTransformation").at("R1")[1].get<float>();
        transfomation_.M12 = jConfig.at("frameTransformation").at("R1")[2].get<float>();
        transfomation_.M20 = jConfig.at("frameTransformation").at("R2")[0].get<float>();
        transfomation_.M21 = jConfig.at("frameTransformation").at("R2")[1].get<float>();
        transfomation_.M22 = jConfig.at("frameTransformation").at("R2")[2].get<float>();
        transfomation_.t0 = jConfig.at("frameTransformation").at("t0").get<float>();
        transfomation_.t1 = jConfig.at("frameTransformation").at("t1").get<float>();
        transfomation_.t2 = jConfig.at("frameTransformation").at("t2").get<float>();

        // Test if the transformation matrix is good
        float sum = pow(transfomation_.M00, 2) + pow(transfomation_.M10, 2) +
            pow(transfomation_.M20, 2);
        if ((sum < 0.99) || (sum > 1.01)) {
            throw std::invalid_argument("Bad transformation matrix provide");
        }
        sum = pow(transfomation_.M01, 2) + pow(transfomation_.M11, 2) + pow(transfomation_.M21, 2);
        if ((sum < 0.99) || (sum > 1.01)) {
            throw std::invalid_argument("Bad transformation matrix provide");
        }
        sum = pow(transfomation_.M02, 2) + pow(transfomation_.M12, 2) + pow(transfomation_.M22, 2);
        if ((sum < 0.99) || (sum > 1.01)) {
            throw std::invalid_argument("Bad transformation matrix provide");
        }

        initialPosition_.M00 = jConfig.at("initialPosition").at("R0")[0].get<float>();
        initialPosition_.M01 = jConfig.at("initialPosition").at("R0")[1].get<float>();
        initialPosition_.M02 = jConfig.at("initialPosition").at("R0")[2].get<float>();
        initialPosition_.M10 = jConfig.at("initialPosition").at("R1")[0].get<float>();
        initialPosition_.M11 = jConfig.at("initialPosition").at("R1")[1].get<float>();
        initialPosition_.M12 = jConfig.at("initialPosition").at("R1")[2].get<float>();
        initialPosition_.M20 = jConfig.at("initialPosition").at("R2")[0].get<float>();
        initialPosition_.M21 = jConfig.at("initialPosition").at("R2")[1].get<float>();
        initialPosition_.M22 = jConfig.at("initialPosition").at("R2")[2].get<float>();
        initialPosition_.t0 = jConfig.at("initialPosition").at("t0").get<float>();
        initialPosition_.t1 = jConfig.at("initialPosition").at("t1").get<float>();
        initialPosition_.t2 = jConfig.at("initialPosition").at("t2").get<float>();

        // Test if the transformation matrix is good
        sum = pow(initialPosition_.M00, 2) + pow(initialPosition_.M10, 2) +
            pow(initialPosition_.M20, 2);
        if ((sum < 0.99) || (sum > 1.01)) {
            throw std::invalid_argument("Bad initial position matrix provide");
        }
        sum = pow(initialPosition_.M01, 2) + pow(initialPosition_.M11, 2) +
            pow(initialPosition_.M21, 2);
        if ((sum < 0.99) || (sum > 1.01)) {
            throw std::invalid_argument("Bad initial position matrix provide");
        }
        sum = pow(initialPosition_.M02, 2) + pow(initialPosition_.M12, 2) +
            pow(initialPosition_.M22, 2);
        if ((sum < 0.99) || (sum > 1.01)) {
            throw std::invalid_argument("Bad initial position matrix provide");
        }

        int doorNumber = jConfig.at("doors").at("number").get<int>();
        if (doorNumber <= 0) {
            throw std::invalid_argument("Number of doors should be more than 0");
        }
        for (int i = 0; i < doorNumber; i++) {
            std::string id = "door";
            id += std::to_string(i);
            std::vector <float> singlePosition;
            for (int j = 0; j < 3; j++) {
                singlePosition.push_back(
                    jConfig.at("doors").at("doorsLocation").at(id)[j].get<float>());
            }
            doorsPosition_.push_back(singlePosition);
        }

        alreadyParsed_ = true;
        log_->debug("Configuration parameters... Done");
    } catch (const std::exception& e) {
        log_->error("Failed to parse because: {}", e.what());
        return false;
    }
    return true;
}

boost::optional <NeighboursSearch> GraphOptimizationConfiguration::getNeighbourgsSearchData() {
    if (!alreadyParsed_) {
        log_->error("Call parse function before ask for data");
        return boost::none;
    }
    return neighboursData_;
}

boost::optional <Solvers> GraphOptimizationConfiguration::getSolversData() {
    if (!alreadyParsed_) {
        log_->error("Call parse function before ask for data");
        return boost::none;
    }
    return solversData_;
}

boost::optional <Simulations> GraphOptimizationConfiguration::getSimulationsData() {
    if (!alreadyParsed_) {
        log_->error("Call parse function before ask for data");
        return boost::none;
    }
    return simulationsData_;
}

boost::optional <SlamParameters> GraphOptimizationConfiguration::getSlamData() {
    if (!alreadyParsed_) {
        log_->error("Call parse function before ask for data");
        return boost::none;
    }
    return slamData_;
}

boost::optional <Eigen::Matrix4f> GraphOptimizationConfiguration::getTransformationData() {
    if (!alreadyParsed_) {
        log_->error("Call parse function before ask for data");
        return boost::none;
    }
    Eigen::Matrix4f transformationMatrix;
    transformationMatrix <<
        transfomation_.M00, transfomation_.M01, transfomation_.M02, transfomation_.t0,
        transfomation_.M10, transfomation_.M11, transfomation_.M12, transfomation_.t1,
        transfomation_.M20, transfomation_.M21, transfomation_.M22, transfomation_.t2,
        0, 0, 0, 1;
    return transformationMatrix;
}

boost::optional <Eigen::Matrix4f> GraphOptimizationConfiguration::getInitialPositionData() {
    if (!alreadyParsed_) {
        log_->error("Call parse function before ask for data");
        return boost::none;
    }
    Eigen::Matrix4f initialPositionMatrix;
    initialPositionMatrix <<
        initialPosition_.M00, initialPosition_.M01, initialPosition_.M02, initialPosition_.t0,
        initialPosition_.M10, initialPosition_.M11, initialPosition_.M12, initialPosition_.t1,
        initialPosition_.M20, initialPosition_.M21, initialPosition_.M22, initialPosition_.t2,
        0, 0, 0, 1;
    return initialPositionMatrix;
}

boost::optional <std::vector <std::vector <float>>>
    GraphOptimizationConfiguration::getDoorsPosition() {
    if (!alreadyParsed_) {
        log_->error("Call parse function before ask for data");
        return boost::none;
    }
    return doorsPosition_;
}

}  // namespace graphoptimization
}  // namespace applications
}  // namespace crf
