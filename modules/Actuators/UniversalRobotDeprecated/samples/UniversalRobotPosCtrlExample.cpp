/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "UniversalRobot/UniversalRobotRTDE.hpp"
#include "UniversalRobot/UniversalRobotRTDEInterface.hpp"
#include "GeometricMethods/DeBoor/DeBoor.hpp"

using namespace std::chrono; // NOLINT

/*
 * @brief Function to facilitate easy printing of vectors
 */
void printVec(std::vector<double> vec, std::string name) {
    std::cout << name << " = ( ";
    for (size_t i = 0; i < vec.size()-1; i++) std::cout << vec[i] << ", ";
    std::cout << vec[vec.size()-1] << " )" << std::endl;
}

/*
 * @brief Function to read a csv file
 */
std::vector<double> readCSV(std::string file_loc) {
    std::ifstream fin;
    std::string line;
    std::vector<double> input;
    fin.open(file_loc);
    if (!fin.is_open()) {
        std::cout << "Couldn't open file!" << std::endl;
    }
    while (std::getline(fin, line, ',')) {  // \r because data is coming from windows system
        if (line[0] != '\n') {
            input.push_back(std::stod(line));
        }
    }
    return input;
}

/*
 * @brief Function to performa cross-correlation between two vectors
 */
std::vector<double> xcorr(const std::vector<double>& v1, const std::vector<double>& v2) {
    if (v1.size() != v2.size()) {
        std::cout << "Error in XCORR: vectors are not the same size" << std::endl;
        std::vector<double> empty_vec;
        return empty_vec;
    }

    size_t n {v1.size()};

    // Zero padding
    std::vector<double> v_pad(3*n);
    for (int i = 0; i < n; i++) {
        v_pad[n+i] = v2[i];
    }

    // Calculate Correlation
    std::vector<double> res(2*n+1);
    double sum {0};
    for (int i = 0; i <= 2*n; i++) {
        sum = 0;
        for (int j = 0; j < n; j++) {
            sum += v1[j] * v_pad[i+j];
        }
        res[i] = sum;
    }

    return res;
}

/*
 * @brief Function to find the offset of the highest peak from the middle of the vector. In
 *        combination with xcorr() this can be used to determine the delay/shift in time between
 *        two vectors.
 */
int getDelay(const std::vector<double>& v) {
    size_t n = v.size();
    auto it_max = std::max_element(v.begin(), v.end());
    int d = std::distance(v.begin(), it_max);
    std::cout << "\tn/2 = " << n/2 << " / max = " << *it_max << " / d = " << d << std::endl;
    return d - (n/2-1);
}

int main(int argc, char** argv) {
    // Some constants
    const int SAMPLING_FQ = 500, MOV_JOINT = 3;
    const double MIN_ACC = 0, MAX_ACC = 30;  // rad/s^2
    const double SPL_DUR = 10, SIM_TIME = SPL_DUR + 1;  // duration in seconds
    double v = 0, a = 0, SAMPLING_TIME = 1.0/SAMPLING_FQ, kp = 4, kd = 0, ki = 0;
    const double STEPS = SIM_TIME*SAMPLING_FQ;

    // Sanity Checks
    if (argc > 2) {
        std::cout << "Too many arguments" << std::endl;
        std::cout << "[1] Configuration File" << std::endl;
        return -1;
    } else if (argc <= 1) {
        std::cout << "Not Enough input arguments" << std::endl;
        std::cout << "[1] Configuration File" << std::endl;
        return -1;
    }

    // Parse the configuration for the UniversalRobot
    std::ifstream robotData(argv[1]);
    crf::actuators::universalrobot::UniversalRobotRTDE ur{
        std::make_shared<crf::actuators::universalrobot::UniversalRobotRTDEInterface>(),
        nlohmann::json::parse(robotData)};

    // Initialize the NURBS trajectory
    std::vector<double> knots = readCSV("/home/lrodrigo/Documents/deBoor/knots.csv");
    std::vector<double> controlPoints = readCSV("/home/lrodrigo/Documents/deBoor/ctrlPts.csv");
    std::vector<double> degreeVector = readCSV("/home/lrodrigo/Documents/deBoor/degree.csv");
    int degree = static_cast<int>(degreeVector[0]);

    std::cout << std::endl;
    std::cout << "Configure Spline..." << std::endl;
    printVec(knots, "\tknots");
    printVec(controlPoints, "\tctrlPts");
    printVec(degreeVector, "\tdegree");
    std::cout << std::endl;

    // Evaluate Trajectory
    for (size_t i = 0; i < knots.size(); i++) knots[i] *= SPL_DUR;
    auto spline = std::make_unique<crf::math::geometricmethods::DeBoor>(degree, knots,
        controlPoints);
    std::vector<double> q_des(STEPS, 0), qd_des(STEPS, 0), qdd_des(STEPS, 0);
    for (int i = 0; i < SIM_TIME*static_cast<int>(SAMPLING_FQ); i++) {
        auto q = spline->evaluate(i*SAMPLING_TIME, 0);
        q_des[i] = q.value();
        q_des[i] *= 0.5;
    }
    for (int i = 0; i < SIM_TIME*static_cast<int>(SAMPLING_FQ)-1; i++) {
        qd_des[i] = (q_des[i+1] - q_des[i]) / SAMPLING_TIME;
    }
    for (int i = 0; i < SIM_TIME*static_cast<int>(SAMPLING_FQ)-1; i++) {
        qdd_des[i] = (qd_des[i+1] - qd_des[i]) / SAMPLING_TIME;
    }

    std::cout <<"Analyse Spline..." << std::endl;
    std::cout <<"\tmax Velocity: " << *std::max_element(qd_des.begin(), qd_des.end())
        << " rad/s" << std::endl;
    std::cout <<"\tmax Acceleration: " << *std::max_element(qdd_des.begin(), qdd_des.end())
        << " rad/s^2" << std::endl;
    std::cout << std::endl;
    std::cout <<"Initialize UR..." << std::endl;
    ur.initialize();

    // Read the actual joint positions
    crf::utility::types::JointPositions qStart = ur.getJointPositions().get();
    std::vector<double> qStartVec{qStart[0], qStart[1], qStart[2], qStart[3], qStart[4], qStart[5]};
    printVec(qStartVec, "\tqStart");

    // Add offset to the trajectory (because we cant 0 the robot arm with the current interface)
    if (std::abs(qStart[MOV_JOINT-1]) > 0.01) {
        std::cout << "ERROR: Joint to be moved not within [-0.01, 0.01] rad. " << std::endl;
    }

    // Set up some variables
    std::vector<double> q_act_print(STEPS, 0), q_des_print(STEPS, 0), t_rest_print(STEPS, 0),
        error_pos(STEPS, 0), qd_ctrl(STEPS, 0), qdd_max(STEPS, 0), q_temp;
    crf::utility::types::JointVelocities qd_des_vec6{0, 0, 0, 0, 0, 0};
    duration<double> t_simDur, t_cycleDur, t_readDur, t_writeDur;
    high_resolution_clock::time_point t_simStart, t_cycleStart, t_cycleStop, t_readStart,
        t_readStop, t_writeStart, t_writeStop;
    std::vector<double> time(STEPS, 0), timeSim_pcClock(STEPS, 0), t_readDur_data(STEPS, 0),
        t_writeDur_data(STEPS, 0);
    bool RTviolation = false, controlException = false;
    int count = 0;
    double q_act = 0, euler = 0, P = 0, I = 0, D = 0, acc_max = 0;

    // Start the 500Hz control loop
    t_simStart = high_resolution_clock::now();
    for (unsigned int i = 0; i < SIM_TIME*SAMPLING_FQ; i++) {
        // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
        t_cycleStart = high_resolution_clock::now();
        t_simDur = duration_cast<duration<double>>(t_cycleStart - t_simStart);
        timeSim_pcClock[i] = t_simDur.count();
        t_readStart = high_resolution_clock::now();
        crf::utility::types::JointPositions qTemp = ur.getJointPositions().get();
        t_readStop = high_resolution_clock::now();
        t_readDur = duration_cast<duration<double>>(t_readStop - t_readStart);
        t_readDur_data[i] = t_readDur.count();

        q_act = qTemp[MOV_JOINT-1];
        q_act_print[i] = q_act;
        q_des_print[i] = q_des[i];

        t_writeStart = high_resolution_clock::now();

        // calculate control input
        error_pos[i] = q_des[i] - q_act;
        P = kp * error_pos[i];
        euler += error_pos[i] * SAMPLING_TIME;
        I = ki * euler;
        if (i > 0) {
            D = kd * (error_pos[i] - error_pos[i-1]) / SAMPLING_TIME;
        }

        qd_ctrl[i] = qd_des[i] + P + I + D;

        qd_des_vec6[MOV_JOINT-1] = qd_ctrl[i];

        if (i > 0) {
            qdd_max[i] = (qd_ctrl[i] - qd_ctrl[i-1]) / SAMPLING_TIME;
        }
        // std::cout<<"qdd_max "<<qdd_max<<std::endl;
        acc_max = std::abs(qdd_des[i])*1.5;
        if (acc_max > MAX_ACC) {
            acc_max = MAX_ACC;
        } else if (acc_max < MIN_ACC) {
            acc_max = MIN_ACC;
        }



        try {
            ur.setJointVelocities(qd_des_vec6, acc_max);
            // absolute value because only value within [0, 40] are allowed
        } catch (std::exception& e) {
            controlException = true;
            std::cout << e.what() << std::endl;
            ur.deinitialize();
            break;
        }
        t_writeStop = high_resolution_clock::now();
        t_writeDur = duration_cast<duration<double>>(t_writeStop - t_writeStart);
        t_writeDur_data[i] = t_writeDur.count();

        count++;
        t_cycleStop = high_resolution_clock::now();
        t_cycleDur = duration_cast<duration<double>>(t_cycleStop - t_cycleStart);
        t_rest_print[i] = SAMPLING_TIME - t_cycleDur.count();
        if (!(t_cycleDur.count() < SAMPLING_TIME)) {
            std::cout << "FATAL ERROR: Real-Time Cycle violation. Computation time > "
                << SAMPLING_TIME << "s" << std::endl;
            ur.deinitialize();
            break;
        } else {
            time[i+1] = time[i] + SAMPLING_TIME;
            std::this_thread::sleep_until(t_simStart + std::chrono::duration<double>(time[i+1]));
        }
    }

    if (!RTviolation && !controlException) {
        std::cout << std::endl;
        std::cout <<"Control loop ended..." << std::endl;

        crf::utility::types::JointPositions qEnd = ur.getJointPositions().get();
        std::vector<double> qEndVec{qEnd[0], qEnd[1], qEnd[2], qEnd[3], qEnd[4], qEnd[5]};
        printVec(qEndVec, "\tqEnd");
        std::cout << std::endl;

        ur.deinitialize();
    }

    std::string filename;
    std::vector<double> xcorr_vec(xcorr(q_des_print, q_act_print));

    // calculate histogram of how much we are overshooting desired cycletime of 2ms
    std::vector<double> hist(8, 0);
    for (int i = 0; i < count; i++) {
        if (t_rest_print[i] <= 0) {
            hist[0] = hist[0] + 1;
        } else if (t_rest_print[i] <= 0 + 0.0005) {
            hist[1] = hist[1] + 1;
        } else if (t_rest_print[i] <= 0 + 0.001) {
            hist[2] = hist[2] + 1;
        } else if (t_rest_print[i] <= 0 + 0.0015) {
            hist[3] = hist[3] + 1;
        } else if (t_rest_print[i] <= 0 + 0.0018) {
            hist[4] = hist[4] + 1;
        } else if (t_rest_print[i] <= 0 + 0.00185) {
            hist[5] = hist[5] + 1;
        } else if (t_rest_print[i] <= 0 + 0.0019) {
            hist[6] = hist[6] + 1;
        } else if (t_rest_print[i] <= 0 + 0.00195) {
            hist[7] = hist[7] + 1;
        }
    }

    std::cout << std::endl << "Test Results..." << std::endl;
    std::cout << "\tAnticipated Cycles: " << SIM_TIME*SAMPLING_FQ
                                                        << " (SIM_TIME*SAMPLING_FQ)" << std::endl;
    std::cout << "\tActual Cycles: " << count << std::endl;
    int del = getDelay(xcorr_vec);
    std::cout << "\tDelay in timesteps: " << del << std::endl;
    std::cout << "\tDelay in ms: " << SAMPLING_TIME*del*1000 << std::endl;
    std::cout << "\tMax time to read from robot: "
        << *max_element(std::begin(t_readDur_data), std::end(t_readDur_data)) << std::endl;
    std::cout << "\tMax time to write from robot: "
        << *max_element(std::begin(t_writeDur_data), std::end(t_writeDur_data)) << std::endl;
    std::cout << "\tHist of Rest Times: " << "|" << std::setw(6)<< hist[0] << "|" << std::setw(6)
        << hist[1] << "|" << std::setw(6)<< hist[2] << "|" << std::setw(6)<< hist[3] << "|"
        << std::setw(6)<< hist[4] << "|" << std::setw(6)<< hist[5] << "|" << std::setw(6)
        << hist[6] << "|" << std::setw(6)<< hist[7] << "|" << std::setw(6)<< hist[8] << "|"
        << std::endl;
    std::cout << "\t                          0ms   0.5ms   1ms   1.5ms   1.8ms  1.85ms  1.9ms  1.95ms" // NOLINT
                                                                                    << std::endl;

    std::cout << "Saving Data..." << std::endl;

    filename += "/home/lrodrigo/Documents/deBoor/rec_data.csv";
    std::ofstream fout4(filename);
    fout4 << "TestFile\n";
    fout4 << "q_des" << ',' << "q_act" << ',' << "q_diff" << ',' << "qd_des" << ',' << "qd_act"
        << ',' << "t_rest" << ',' << "time_ideal" << ',' << "time_pcClock"  <<','
        << "qdd_ctrl_input" << ',' << "qdd_des" << '\n';
    for (size_t i = 0; i < q_des_print.size()-1; i++) {
        fout4 << q_des_print[i] << ',' << q_act_print[i] << ',' << (q_act_print[i] - q_des_print[i])
        << ','<< (q_des_print[i+1] - q_des_print[i])/SAMPLING_TIME << ','
        << (q_act_print[i+1] - q_act_print[i])/SAMPLING_TIME << ',' << t_rest_print[i] << ','
        << time[i] << ',' << timeSim_pcClock[i] << ',' << qd_ctrl[i] << ',' << qdd_des[i] << '\n';
    }
    fout4.close();

    filename = "/home/lrodrigo/Documents/deBoor/xcorr_data.csv";
    std::ofstream fout5(filename);
    fout5  << "xcorr(q_des,q_act)" << '\n';
    for (size_t i = 0; i < xcorr_vec.size(); i++) fout5 << xcorr_vec[i] << '\n';
    fout5.close();

    return 0;
}
