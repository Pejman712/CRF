/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "YoubotArm/YoubotArm.hpp"
#include "SerialCommunication/SerialCommunication.hpp"

using json = nlohmann::json;

float switchWhitePositionAbove[] = {0.10, 0.0831649, 1.57358, 1.26672, 0.216772};
float switchWhitePositionEngaged[] = {0.10, 0.0831649, 1.57358, 1.46672, 0.216772};
float switchWhitePositionEngagedOn[] = {0.10, 0.0831649, 1.57358, 1.46672, -0.686772};

float switchBluePositionAbove[] = {0.13, 0.0, 1.44358, 1.26672, 0.216772};
float switchBluePositionEngaged[] = {0.13, 0.0, 1.44358, 1.6148, 0.216772};
float switchBluePositionEngagedOn[] = {0.13, 0.0, 1.4458, 1.6148, -0.706772};

void goToPositionAndWait(crf::robots::YoubotArm* arm, Matrix<float> position) {
    bool positionReached = false;
    arm->setJointPositions(position);

    while (!positionReached) {
        Matrix<float> currentPosition = arm->getJointPositions();
        positionReached = true;
        for (int i = 0; i < 5; i++) {
            if (fabs(currentPosition(i, 0)- position(i, 0)) > 0.0001) {
                positionReached = false;
            }
        }
    }
}

std::vector<int> getDIValues() {
    SerialCommunication serial("/dev/ttyACM0", 9600, true);
    bool append = false;
    std::string line;
    char buf;

    do {
        serial.read(&buf, 1);

        if (buf == '{') append = true;
        if (append) line.append(&buf, 1);

        if ((buf == '\n') && (append)) break;
    } while (true);

    std::vector<int> values;
    json j = json::parse(line);

    values.push_back(j["p21"].get<int>());
    values.push_back(j["p22"].get<int>());
    values.push_back(j["p23"].get<int>());
    values.push_back(j["p24"].get<int>());
    values.push_back(j["p25"].get<int>());
    values.push_back(j["p26"].get<int>());
    values.push_back(j["p29"].get<int>());
    values.push_back(j["p30"].get<int>());

    return values;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Wrong number of arguments\n");
        printf("[1] Type of switch (1: blue switch, 2: white switch)\n");
        printf("[2] Number of iterations\n");
        return -1;
    }

    crf::robots::YoubotArm arm("/tmp/fifo_youbot_arm", "/tmp/mmap_youbot_arm");

    Matrix<float> zeroPosition(5, 1, .0);
    zeroPosition(0, 0) = 0.10;
    Matrix<float> switchPositionAboveMatrix(1, 1);
    Matrix<float> switchPositionEngagedMatrix(1, 1);
    Matrix<float> switchPositionEngagedOnMatrix(1, 1);

    int typeOfSwitch = atoi(argv[1]);
    int numberOfIterations = atoi(argv[2]);
    int numberOfOutputs;
    switch (typeOfSwitch) {
        case 1:
            switchPositionAboveMatrix = Matrix<float> (5, 1, switchBluePositionAbove);
            switchPositionEngagedMatrix = Matrix<float>(5, 1, switchBluePositionEngaged);
            switchPositionEngagedOnMatrix = Matrix<float>(5, 1, switchBluePositionEngagedOn);
            numberOfOutputs = 8;
            break;
        case 2:
            switchPositionAboveMatrix = Matrix<float> (5, 1, switchWhitePositionAbove);
            switchPositionEngagedMatrix = Matrix<float>(5, 1, switchWhitePositionEngaged);
            switchPositionEngagedOnMatrix = Matrix<float>(5, 1, switchWhitePositionEngagedOn);
            numberOfOutputs = 4;
            break;
    }

    printf("Going to zero position\n");
    goToPositionAndWait(&arm, zeroPosition);
    arm.openGripper();
    sleep(3);

    printf("Going to above the switch\n");
    goToPositionAndWait(&arm, switchPositionAboveMatrix);
    printf("Engaging the switch\n");
    goToPositionAndWait(&arm, switchPositionEngagedMatrix);
    printf("Closing the gripper\n");
    arm.closeGripper();
    sleep(3);

    printf("Switch iteration\n");
    std::vector<int> values;
    std::vector<int> iterationsError;
    for (int i = 0; i < numberOfIterations; i++) {
        printf("Completed %d/%d\n", i, numberOfIterations);

        goToPositionAndWait(&arm, switchPositionEngagedOnMatrix);

        values = getDIValues();
        values = getDIValues();
        values = getDIValues();
        values = getDIValues();
        bool allActive = true;
        for (int i = 0; i < numberOfOutputs; i++) {
            if (values[i] != 1) allActive = false;
        }

        if (!allActive) {
            printf("Error, not all the outputs are active\n");
            iterationsError.push_back(i);
        }

        goToPositionAndWait(&arm, switchPositionEngagedMatrix);

        values = getDIValues();
        values = getDIValues();
        values = getDIValues();
        values = getDIValues();
        allActive = true;
        for (int i = 0; i < numberOfOutputs; i++) {
            if (values[i] != 0) allActive = false;
        }

        if (!allActive) {
            printf("Error, not all the outputs are off\n");
            iterationsError.push_back(i);
        }
    }
    printf("Iterations completed\n");
    printf("Opening gripper\n");
    arm.openGripper();
    sleep(3);
    goToPositionAndWait(&arm, switchPositionAboveMatrix);

    printf("Going to zero position\n");
    goToPositionAndWait(&arm, zeroPosition);

    if (iterationsError.size() == 0) return 0;
    FILE* fd;

    fd = fopen(argv[3], "w+");
    printf("\n\n\n");
    printf("There has been a problem with the switch at the following iterations\n");
    fprintf(fd, "There has been a problem with the switch at the following iterations\n");

    for (int i = 0; i < iterationsError.size(); i++) {
        printf("%d,", iterationsError[i]);
        fprintf(fd, "%d,", iterationsError[i]);
    }
    printf("\n");
    fprintf(fd, "\n");

    fclose(fd);

    return 0;
}
