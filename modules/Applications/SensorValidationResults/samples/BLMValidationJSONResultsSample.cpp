/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>

#include "SensorValidationResults/BLMValidationJSONResults/BLMValidationJSONResults.hpp"
#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"

using crf::applications::blmtriggeringanalysis::BLMTriggeringResult;

int main(int argc, char* argv[]) {
    crf::applications::sensorvalidationresults::BLMValidationJSONResults results(argv[1],
        argv[2], std::chrono::seconds(2));

    std::this_thread::sleep_for(std::chrono::seconds(10));

    results.irradiationTime("BLMTS.A6L7", std::numeric_limits<std::uint32_t>::max(), 1000);
    auto irradiationTimeResult = results.irradiationTime("BLMTS.A6L7");
    std::cout << "irradiationTime: " << std::endl;
    std::cout << "  - WriterPriority: " << irradiationTimeResult.first << std::endl;
    std::cout << "  - Value: " << irradiationTimeResult.second << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(10));

    results.expertMonitorName("BLMTS.A6L7", 2, "Expert BLMTS.A6L7");
    auto expertMonitorNameResult = results.expertMonitorName("BLMTS.A6L7");
    std::cout << "expertMonitorName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    results.expertMonitorName("BLMQI.F25L4", 1, "Expert BLMQI.F25L4");
    expertMonitorNameResult = results.expertMonitorName("BLMQI.F25L4");
    std::cout << "expertMonitorName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    results.expertMonitorName("BLMQI.F25L4", 10, "Expert BLMQI.F25L4");
    expertMonitorNameResult = results.expertMonitorName("BLMQI.F25L4");
    std::cout << "expertMonitorName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    results.expertMonitorName("BLMQI.F25L4", 1, "Expert BLMQI.F25L4");
    expertMonitorNameResult = results.expertMonitorName("BLMQI.F25L4");
    std::cout << "expertMonitorName: " << std::endl;
    std::cout << "  - WriterPriority: " << expertMonitorNameResult.first << std::endl;
    std::cout << "  - Value: " << expertMonitorNameResult.second << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    results.interactionPoint("BLMTS.A6L7", 1, "BLMTS.A6L7 IP");
    auto interactionPointResult = results.interactionPoint("BLMTS.A6L7");
    std::cout << "interactionPoint: " << std::endl;
    std::cout << "  - WriterPriority: " << interactionPointResult.first << std::endl;
    std::cout << "  - Value: " << interactionPointResult.second << std::endl;

    results.expectedDCUM("BLMTS.A6L7", 1, 150);
    auto expectedDCUMResult = results.expectedDCUM("BLMTS.A6L7");
    std::cout << "expectedDCUM: " << std::endl;
    std::cout << "  - WriterPriority: " << expectedDCUMResult.first << std::endl;
    std::cout << "  - Value: " << expectedDCUMResult.second << std::endl;

    results.measuredDCUM("BLMTS.A6L7", 1, 5000);
    auto measuredDCUMResult = results.measuredDCUM("BLMTS.A6L7");
    std::cout << "measuredDCUM: " << std::endl;
    std::cout << "  - WriterPriority: " << measuredDCUMResult.first << std::endl;
    std::cout << "  - Value: " << measuredDCUMResult.second << std::endl;

    results.reachabilityLevel("BLMTS.A6L7", 1, "BLMTS.A6L7 Reach");
    auto reachibilityLevelResult = results.reachabilityLevel("BLMTS.A6L7");
    std::cout << "reachabilityLevel: " << std::endl;
    std::cout << "  - WriterPriority: " << reachibilityLevelResult.first << std::endl;
    std::cout << "  - Value: " << reachibilityLevelResult.second << std::endl;

    results.location("BLMTS.A6L7", 1, "En un lugar de la Mancha");
    auto locationResult = results.location("BLMTS.A6L7");
    std::cout << "location: " << std::endl;
    std::cout << "  - WriterPriority: " << locationResult.first << std::endl;
    std::cout << "  - Value: " << locationResult.second << std::endl;

    results.position("BLMTS.A6L7", 1, "Front");
    auto positionResult = results.position("BLMTS.A6L7");
    std::cout << "position: " << std::endl;
    std::cout << "  - WriterPriority: " << positionResult.first << std::endl;
    std::cout << "  - Value: " << positionResult.second << std::endl;

    results.labelID("BLMTS.A6L7", 2, "BLMTS.A6L7 ID");
    auto labelIDResult = results.labelID("BLMTS.A6L7");
    std::cout << "labelID: " << std::endl;
    std::cout << "  - WriterPriority: " << labelIDResult.first << std::endl;
    std::cout << "  - Value: " << labelIDResult.second << std::endl;

    results.labelQRCode("BLMTS.A6L7", 1, "BLMTS.A6L7 QRCode");
    auto labelQRCodeResult = results.labelQRCode("BLMTS.A6L7");
    std::cout << "labelQRCode: " << std::endl;
    std::cout << "  - WriterPriority: " << labelQRCodeResult.first << std::endl;
    std::cout << "  - Value: " << labelQRCodeResult.second << std::endl;

    results.isLabelCorrect("BLMTS.A6L7", 1, true);
    auto isLabelCorrectResult = results.isLabelCorrect("BLMTS.A6L7");
    std::cout << "isLabelCorrect: " << std::endl;
    std::cout << "  - WriterPriority: " << isLabelCorrectResult.first << std::endl;
    std::cout << "  - Value: " << isLabelCorrectResult.second << std::endl;

    results.selectedRunningSum("BLMTS.A6L7", 1, "BLMTS.A6L7 R1");
    auto selectedRunningSumResult = results.selectedRunningSum("BLMTS.A6L7");
    std::cout << "selectedRunningSum: " << std::endl;
    std::cout << "  - WriterPriority: " << selectedRunningSumResult.first << std::endl;
    std::cout << "  - Value: " << selectedRunningSumResult.second << std::endl;

    results.irradiationTime("BLMTS.A6L7", 7, 60);
    irradiationTimeResult = results.irradiationTime("BLMTS.A6L7");
    std::cout << "irradiationTime: " << std::endl;
    std::cout << "  - WriterPriority: " << irradiationTimeResult.first << std::endl;
    std::cout << "  - Value: " << irradiationTimeResult.second << std::endl;

    results.analysisThreshold("BLMTS.A6L7", 1, 0.000008);
    auto analysisThresholdResult = results.analysisThreshold("BLMTS.A6L7");
    std::cout << "analysisThreshold: " << std::endl;
    std::cout << "  - WriterPriority: " << analysisThresholdResult.first << std::endl;
    std::cout << "  - Value: " << analysisThresholdResult.second << std::endl;

    results.distanceToSource("BLMTS.A6L7", 1, 0.001);
    auto distanceToSourceResult = results.distanceToSource("BLMTS.A6L7");
    std::cout << "distanceToSource: " << std::endl;
    std::cout << "  - WriterPriority: " << distanceToSourceResult.first << std::endl;
    std::cout << "  - Value: " << distanceToSourceResult.second << std::endl;

    crf::applications::blmtriggeringanalysis::BLMTriggeringResult analysisInput;
    analysisInput.monitorName("BLMTS.A6L7");
    analysisInput.interactionPoint("BLMTS.A6L7IP");
    analysisInput.timeBackgroundStart(10000);
    analysisInput.timeBackgroundEnd(10030);
    analysisInput.backgroundLevel(0.0000056);
    analysisInput.timeTestStart(50030);
    analysisInput.timeTestEnd(50060);
    analysisInput.signalMedian(0.0000078);
    analysisInput.signalMean(0.0);
    analysisInput.signalStandardDeviation(0.0);
    analysisInput.isSignalAboveThreshold(true);
    analysisInput.blmsAboveThreshold({"X", "Y", "Z"});
    std::cout << "triggeringAnalysis: " << std::endl;

    results.triggeringAnalysis("BLMTS.A6L7", 1, analysisInput);

    std::cout << "  - WriterPriority: " << results.triggeringAnalysis("BLMTS.A6L7").first << std::endl;  // NOLINT

    auto analysisOutput = results.triggeringAnalysis("BLMTS.A6L7").second;
    std::cout << "  - TimeBackgroundStart: " << analysisOutput.timeBackgroundStart() << std::endl;
    std::cout << "  - TimeBackgroundEnd: " << analysisOutput.timeBackgroundEnd() << std::endl;
    std::cout << "  - BackgroundLevel: " << analysisOutput.backgroundLevel() << std::endl;
    std::cout << "  - TimeTestStart: " << analysisOutput.timeTestStart() << std::endl;
    std::cout << "  - TimeTestEnd: " << analysisOutput.timeTestEnd() << std::endl;
    std::cout << "  - SignalMedian: " << analysisOutput.signalMedian() << std::endl;
    std::cout << "  - SignalMean: " << analysisOutput.signalMean() << std::endl;
    std::cout << "  - SignalStandardDeviation: " << analysisOutput.signalStandardDeviation() << std::endl;  // NOLINT
    std::cout << "  - IsSignalAboveThreshold: " << analysisOutput.isSignalAboveThreshold() << std::endl;  // NOLINT
    for (auto iter : analysisOutput.blmsAboveThreshold()) {
        std::cout << "  - BLMsAboveThreshold: " << iter << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    return 0;
}
