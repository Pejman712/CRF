#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>
#include <boost/optional.hpp>

#include <Eigen/Dense>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace applications {
namespace graphoptimization {

struct NeighboursSearch {
    enum SearchType { Closest, Radius };

    unsigned int value;
    float radius;
    SearchType type;
};

struct Solvers {
    enum LinearSolverType {Dense, Choldmod, Csparse, Eigen, Pcg };
    enum NonLinearSolverType { Levenberg, GaussNewton };

    LinearSolverType linearType;
    NonLinearSolverType nonLinearType;
    bool lambdaSelection;
    double lambda;
    bool trialsSelection;
    unsigned int trials;
    bool verbose;
    unsigned int iterations;
    bool robustKernel;
};

struct Simulations {
    float noiseX;
    float noiseY;
    float noiseZ;
    float speedX;
    float speedY;
    float speedZ;
};

struct SlamParameters {
    float threshold;
    bool visualization;
    bool organizedPointCloud;
};

class GraphOptimizationConfiguration {
 public:
    GraphOptimizationConfiguration();
    virtual ~GraphOptimizationConfiguration() = default;
    virtual bool parse(const std::string& configFileName);
    boost::optional <NeighboursSearch> getNeighbourgsSearchData();
    boost::optional <Solvers> getSolversData();
    boost::optional <Simulations> getSimulationsData();
    boost::optional <SlamParameters> getSlamData();
    boost::optional <Eigen::Matrix4f> getTransformationData();
    boost::optional <Eigen::Matrix4f> getInitialPositionData();
    boost::optional <std::vector <std::vector <float>>> getDoorsPosition();

 private:
    utility::logger::EventLogger log_;
    bool alreadyParsed_;

    struct MatrixStructure {
        float M00;
        float M01;
        float M02;
        float M10;
        float M11;
        float M12;
        float M20;
        float M21;
        float M22;
        float t0;
        float t1;
        float t2;
    };

    NeighboursSearch neighboursData_;
    Solvers solversData_;
    Simulations simulationsData_;
    SlamParameters slamData_;
    MatrixStructure transfomation_;
    MatrixStructure initialPosition_;
    std::vector <std::vector <float>> doorsPosition_;
};

}  // namespace graphoptimization
}  // namespace applications
}  // namespace crf
