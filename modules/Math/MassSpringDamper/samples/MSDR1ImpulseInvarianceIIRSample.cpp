/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <vector>
#include <memory>

#include "MassSpringDamper/MSDR1/MSDR1BilinearTransformIIR.hpp"
#include "MassSpringDamper/MSDR1/MSDR1ImpulseInvarianceIIR.hpp"
#include "MassSpringDamper/MSDR1/MSDR1StepInvarianceIIR.hpp"

using crf::math::massspringdamper::MSDR1BilinearTransformIIR;
using crf::math::massspringdamper::MSDR1ImpulseInvarianceIIR;
using crf::math::massspringdamper::MSDR1StepInvarianceIIR;
using crf::math::massspringdamper::IMassSpringDamperR1;
using crf::math::massspringdamper::SignalR1;

const double mass = 1;
const double damping = 2;
const double stiffness = 1;
const double timeStep = 0.01;

void performCalculations(const std::vector<double>& force, const std::string& zTransform,
    const std::string& filename, bool print = true,
    const std::vector<double>& inputOriginal = {}) {

    std::shared_ptr<IMassSpringDamperR1> msdR1;
    if (zTransform == "BilinearTransformIIR") {
        msdR1 = std::make_shared<MSDR1BilinearTransformIIR>(timeStep);
    } else if (zTransform == "ImpulseInvarianceIIR") {
        msdR1 = std::make_shared<MSDR1ImpulseInvarianceIIR>(timeStep);
    } else if (zTransform == "StepInvarianceIIR") {
        msdR1 = std::make_shared<MSDR1StepInvarianceIIR>(timeStep);
    } else {
        std::cout << "Unknown Filter" << std::endl;
    }

    std::ofstream csvFile(filename);
    csvFile << "t,force,m,d,k,position,velocity,acceleration";
    if (!inputOriginal.empty()) {
        csvFile << ",inputOriginal";
    }
    csvFile << "\n0,0," << mass << "," << damping << "," << stiffness << ",0,0,0";
    if (!inputOriginal.empty()) {
        csvFile << ",0";
    }

    for (size_t i = 0; i < force.size(); ++i) {
        auto begin = std::chrono::steady_clock::now();
        SignalR1 val;
        try {
            val = msdR1->calculate(force[i], mass, damping, stiffness);
        } catch (std::invalid_argument &ex) {
            std::cout << "An error occurred! Details: " << ex.what() << std::endl;
            break;
        }

        csvFile << std::endl << (i + 1) * timeStep << "," << force[i] << "," <<
            mass << "," << damping << "," << stiffness << "," <<
            val.position << "," << val.velocity << "," << val.acceleration;
        if (!inputOriginal.empty())
            csvFile << "," << inputOriginal[i];

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        int timeStepDuration = timeStep * 1000 - duration;

        if (print) {
            std::cout << "Displacement: " << val.position << ", Velocity: " << val.velocity <<
                ", Acceleration: " << val.acceleration << std::endl;
        }
    }
    csvFile.close();
}

int main(int argc, char* argv[]) {
    std::string zTransform = argv[1];

    const size_t iterations = 1000;
    std::string path = __FILE__;
    path = path.substr(0, path.find("cpproboticframework")) + "cpproboticframework/build/";
    std::cout << zTransform << "files saved in: " << path << std::endl;

    std::vector<double> force(iterations, 0.0);

    // Function types and filenames
    std::vector<std::string> functionTypes = {"Impulse", "Step", "Ramp", "Sin"};

    for (size_t i = 0; i < functionTypes.size(); ++i) {
        std::string funcType = functionTypes[i];
        std::string filename = path + "data" + funcType + zTransform + ".csv";

        if (funcType == "Impulse") {
            force[5] = 1.0;
        } else if (funcType == "Step") {
            std::fill(force.begin(), force.end(), 1.0);
            force[0] = 0;
            force[1] = 0;
        } else if (funcType == "Ramp") {
            std::fill(force.begin(), force.begin() + 3, 0.0);
            for (size_t j = 3; j < iterations; ++j) {
                force[j] = 1 * (j -2) * timeStep;
            }
        } else if (funcType == "Sin") {
            double w = 10;
            std::fill(force.begin(), force.begin() + 3, 0.0);
            for (size_t j = 3; j < iterations; ++j) {
                force[j] = 1 * sin(w * timeStep * (j - 2));
            }
        }

        std::cout << "-------- Calculating for " << funcType << " function --------" << std::endl;
        performCalculations(force, zTransform, filename, false);
    }

    // Prewarping for ImpulseInvarianceIIR
    if (zTransform == "ImpulseInvarianceIIR") {
        force.assign(iterations, 0.0);
        double w = 20;
        for (size_t i = 3; i < iterations; ++i) {
            force[i] = 1 * sin(w * timeStep * (i + 1));
        }

        std::string filename = path + "withoutPrewarping" + zTransform + ".csv";
        performCalculations(force, zTransform, filename, false);

        // With Prewarping
        std::vector<double> newForce(iterations, 0.0);
        w = 2 / timeStep * atan(w * timeStep / 2);
        for (size_t i = 3; i < iterations; ++i) {
            newForce[i] = 1 * sin(w * timeStep * (i + 1));
        }

        filename = path + "withPrewarping" + zTransform + ".csv";
        performCalculations(newForce, zTransform, filename, false, force);
    }

    return 0;
}
