#!/bin/bash

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##         Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

cd $(dirname $0)
cd ../../build/

# Instructions
Instructions() {
    echo ""
    echo "      This script launches the necessary communication points to run the"
    echo "      RP Survey with the TIM25. These are: "
    echo ""
    echo "          - TIM                  |  TCP Port 4000"
    echo "          - RP Robot Arm         |  TCP Port 3002"
    echo "          - RP Sensor            |  TCP Port 8000"
    echo "          - Oxygen Sensor        |  TCP Port 8001"
    echo "          - Temperatur Sensor    |  TCP Port 8002"
    echo ""
    echo "      After launching these communication points there is an artificial delay"
    echo "      of 25 seconds to allow everything to get initialized. Finally, the mission"
    echo "      communication point is launched: "
    echo ""
    echo "           - RP Survey Mission   |  TCP Port 5000"
    echo ""
    echo "      To display this help prompt execute this file in the following way:"
    echo "          - ./TIMStartMission.sh help"
    echo ""
    echo "      To run the mission, execute it in the following way:"
    echo "          - ./TIMStartMission.sh run"
    echo ""
    echo "      To kill any mission and points, run the following command"
    echo "          - ./TIMStartMission.sh kill"
    echo ""
    echo "      To launch any individual point, run the next command"
    echo "          - ./TIMStartMission.sh TIM"
    echo "          - ./TIMStartMission.sh TIMRPWagonArm"
    echo "          - ./TIMStartMission.sh RPSensor"
    echo "          - ./TIMStartMission.sh OxygenSensor"
    echo "          - ./TIMStartMission.sh TemperatureSensor"
    echo "          - ./TIMStartMission.sh RPSurveyMission"
    echo ""
}

TIM() {
    sudo -b -E ./../bin/TIMS300Point --protocol tcp --port 4000 --configuration ../modules/Actuators/TIM/config/TIM25.json
}

TIMRPWagonArm() {
    sudo -b -E ./../bin/TIMS300RPWagonPoint --protocol tcp --port 3002 --configuration ../modules/Actuators/TIMRPWagon/config/TIM25RPWagon.json
}

RPSensor() {
    sudo -b -E ./../bin/AtomtexBDKG24Point  --device /dev/ttyUSB0 --protocol tcp --port 8000
}

OxygenSensor() {
    sleep 1
}

TemperatureSensor() {
    sleep 1
}

RPSurveyMission() {
    sudo -b -E ./../bin/RPSurveyLHCPoint --missionPort 5000 --parametersPath ../modules/Applications/MissionManager/config/RPSurveyLHC/missionParametersRPWagon.json
}

# Kill Communication Points
KillPoints() {
    echo ""
    echo "--------------------------------------"
    echo " Killing all the Communication Points"
    echo "--------------------------------------"
    echo ""

    sudo killall -SIGTSTP TIMS300Point
    sudo killall -SIGTSTP TIMS300RPWagonPoint
    sudo killall -SIGTSTP AtomtexBDKG24Point
    sudo killall -SIGTSTP RPSurveyLHCPoint

    sleep 5

    sudo killall TIMS300Point
    sudo killall TIMS300RPWagonPoint
    sudo killall AtomtexBDKG24Point
    sudo killall RPSurveyLHCPoint
}

StartPoints() {
    echo ""
    echo "----------------------------------------------"
    echo " Executing all the Communication Points"
    echo "     - TIM25                |  TCP Port 4000"
    echo "     - TIM25 RP Wagon       |  TCP Port 3002"
    echo "     - RP Sensor            |  TCP Port 8000"
    echo "     - Oxygen Sensor        |  TCP Port 8001"
    echo "     - Temperatur Sensor    |  TCP Port 8002"
    echo "     - RP Survey Mission    |  TCP Port 5000"
    echo "----------------------------------------------"
    echo ""

    TIM
    TIMRPWagonArm
    RPSensor
    OxygenSensor
    TemperatureSensor

    sleep 25

    RPSurveyMission
}

if [ "$#" = 1 ]; then
    export LOGGER_ENABLE_LOGGING=1 && export LOGGER_PRINT_STDOUT=1
    if [ "$1" = "help" ]; then
        Instructions
    elif [ "$1" = "run" ]; then
        StartPoints
    elif [ "$1" = "kill" ]; then
        KillPoints
    elif [ "$1" = "TIM" ]; then
        echo "Executing TIM - TCP Port 4000"
        TIM
    elif [ "$1" = "TIMRPWagonArm" ]; then
        echo "Executing TIM RP Wagon Arm - TCP Port 3002"
        TIMRPWagonArm
    elif [ "$1" = "RPSensor" ]; then
        echo "Executing RP Sensor - TCP Port 8000"
        RPSensor
    elif [ "$1" = "OxygenSensor" ]; then
        echo "Executing Oxygen Sensor - TCP Port 8001"
        OxygenSensor
    elif [ "$1" = "TemperatureSensor" ]; then
        echo "Executing Temperature Sensor - TCP Port 8002"
        TemperatureSensor
    elif [ "$1" = "RPSurveyMission" ]; then
        echo "Executing RP Survey Mission W/ RPArm - TCP Port 5000"
        RPSurveyMission
    else
        Instructions
    fi
else
    Instructions
fi
