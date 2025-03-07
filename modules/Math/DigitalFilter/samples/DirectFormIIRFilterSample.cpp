/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <iostream>
#include "DigitalFilter/IIRFilter.hpp"

using crf::math::digitalfilter::DF2IIRFilter;
using crf::math::digitalfilter::direct2FormFilter;

int main() {
    printf("********************************************************************************\n");
    printf("************* Example of an IIR Filter implementation with order 2 *************\n");
    printf("********************************************************************************\n");
    std::vector<double> a = {1, 0.5, -1};
    std::vector<double> b = {0.5, -1, 0.5};
    std::vector<double> u;
    std::vector<double> x = {10, 6, 2, 0.5, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < x.size(); i++) {
        if (u.size() != 0) {
            printf("Inputs - x: %f, u: [%f %f], a: [%f %f], b: [%f %f %f]\n", x[i],
                u[0], u[1], a[0], a[1], b[0], b[1], b[2]);
        } else {
            printf("Inputs - x: %f, a: [%f %f], b: [%f %f %f]\n", x[i],
                a[0], a[1], b[0], b[1], b[2]);
        }

        try {
            DF2IIRFilter val = direct2FormFilter(x[i], a, b, u);
            u = val.u;
            printf("Ouputs - y: %f, u: [%f, %f]\n", val.y, u[0], u[1]);
        } catch (std::invalid_argument const& ex) {
            std::cout << "A problem has ocurred! Error: " << ex.what() << std::endl;
        }
        printf("---------------------------------------------------------\n");
    }

    printf("\n\n");
    printf("********************************************************************************\n");
    printf("************* Example of an IIR Filter implementation with order 3 *************\n");
    printf("********************************************************************************\n");
    a = {1, 1, 1, 1};
    b = {2, -1, 1, 1};
    u = {};

    for (int i = 0; i < x.size(); i++) {
        if (u.size() != 0) {
            printf("Inputs - x: %f, u: [%f %f %f], a: [%f %f %f], b: [%f %f %f %f]\n", x[i],
                u[0], u[1], u[2], a[0], a[1], a[2], b[0], b[1], b[2], b[3]);
        } else {
            printf("Inputs - x: %f, a: [%f %f %f], b: [%f %f %f %f]\n", x[i],
                a[0], a[1], a[2], b[0], b[1], b[2], b[3]);
        }
        try {
            DF2IIRFilter val = direct2FormFilter(x[i], a, b, u);
            u = val.u;
            printf("Ouputs - y: %f, u: [%f, %f %f]\n", val.y, u[0], u[1], u[2]);
        } catch (std::invalid_argument const& ex) {
            std::cout << "A problem has ocurred! Error: " << ex.what() << std::endl;
        }
        printf("---------------------------------------------------------\n");
    }
    return 0;
}
