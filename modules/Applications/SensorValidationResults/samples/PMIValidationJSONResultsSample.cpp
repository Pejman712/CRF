/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>

#include "SensorValidationResults/PMIValidationJSONResults/PMIValidationJSONResults.hpp"

int main(int argc, char* argv[]) {
    crf::applications::sensorvalidationresults::PMIValidationJSONResults results(argv[1],
        argv[2], std::chrono::seconds(2));

    results.expertName("PMIAM.D4R8", 2, "Expert PMIAM.D4R8");
    auto expertMonitorNameResult = results.expertName("PMIAM.D4R8");
    std::cout << "expertName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    results.expertName("MIAM.B5L7", 1, "Expert MIAM.B5L7");
    expertMonitorNameResult = results.expertName("MIAM.B5L7");
    std::cout << "expertName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    results.expertName("MIAM.B5L7", 10, "Expert MIAM.B5L7");
    expertMonitorNameResult = results.expertName("MIAM.B5L7");
    std::cout << "expertName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    results.expertName("MIAM.B5L7", 1, "Expert MIAM.B5L7");
    expertMonitorNameResult = results.expertName("MIAM.B5L7");
    std::cout << "expertName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    results.sector("PMIAM.D4R8", 1, "PMIAM.D4R8 IP");
    auto sectorResult = results.sector("PMIAM.D4R8");
    std::cout << "sector: " << std::endl;
    std::cout << "  - WriterPriority: " << sectorResult.first << std::endl;
    std::cout << "  - Value: " << sectorResult.second << std::endl;

    results.expectedDCUM("PMIAM.D4R8", 1, 150);
    auto expectedDCUMResult = results.expectedDCUM("PMIAM.D4R8");
    std::cout << "expectedDCUM: " << std::endl;
    std::cout << "  - WriterPriority: " << expectedDCUMResult.first << std::endl;
    std::cout << "  - Value: " << expectedDCUMResult.second << std::endl;

    results.measuredDCUM("PMIAM.D4R8", 1, 5000);
    auto measuredDCUMResult = results.measuredDCUM("PMIAM.D4R8");
    std::cout << "measuredDCUM: " << std::endl;
    std::cout << "  - WriterPriority: " << measuredDCUMResult.first << std::endl;
    std::cout << "  - Value: " << measuredDCUMResult.second << std::endl;

    results.labelID("PMIAM.D4R8", 2, "PMIAM.D4R8 ID");
    auto labelIDResult = results.labelID("PMIAM.D4R8");
    std::cout << "labelID: " << std::endl;
    std::cout << "  - WriterPriority: " << labelIDResult.first << std::endl;
    std::cout << "  - Value: " << labelIDResult.second << std::endl;

    results.labelQRCode("PMIAM.D4R8", 1, "PMIAM.D4R8 QRCode");
    auto labelQRCodeResult = results.labelQRCode("PMIAM.D4R8");
    std::cout << "labelQRCode: " << std::endl;
    std::cout << "  - WriterPriority: " << labelQRCodeResult.first << std::endl;
    std::cout << "  - Value: " << labelQRCodeResult.second << std::endl;

    results.isLabelCorrect("PMIAM.D4R8", 1, true);
    auto isLabelCorrectResult = results.isLabelCorrect("PMIAM.D4R8");
    std::cout << "isLabelCorrect: " << std::endl;
    std::cout << "  - WriterPriority: " << isLabelCorrectResult.first << std::endl;
    std::cout << "  - Value: " << isLabelCorrectResult.second << std::endl;

    results.irradiationTime("PMIAM.D4R8", 7, 60);
    auto irradiationTimeResult = results.irradiationTime("PMIAM.D4R8");
    std::cout << "irradiationTime: " << std::endl;
    std::cout << "  - WriterPriority: " << irradiationTimeResult.first << std::endl;
    std::cout << "  - Value: " << irradiationTimeResult.second << std::endl;

    results.timeTestStart("PMIAM.D4R8", 1, std::time(nullptr));
    auto timeTestStartResult = results.timeTestStart("PMIAM.D4R8");
    std::cout << "timeTestStart: " << std::endl;
    std::cout << "  - WriterPriority: " << timeTestStartResult.first << std::endl;
    std::cout << "  - Value: " << timeTestStartResult.second << std::endl;

    results.timeTestEnd("PMIAM.D4R8", 1, std::time(nullptr));
    auto timeTestEndResult = results.timeTestEnd("PMIAM.D4R8");
    std::cout << "timeTestEnd: " << std::endl;
    std::cout << "  - WriterPriority: " << timeTestEndResult.first << std::endl;
    std::cout << "  - Value: " << timeTestEndResult.second << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    return 0;
}
