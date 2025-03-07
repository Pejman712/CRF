#!/bin/sh

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

cd $(dirname $0)
cd ../build/

# Instructions
Instructions() {
    echo ""
    echo "      This script launches the necessary communication points to run the"
    echo "      CHARMBot. These are: "
    echo ""
    echo "          - CHARMBot Base     |  TCP Port 4000"
    echo "          - Front UVC Camera  |  TCP Port 5000"
    echo "          - Back UVC Camera   |  TCP Port 5001"
    echo ""
    echo "      To display this help prompt execute this file in the following way:"
    echo "          - ./startCHARMBot.sh help"
    echo ""
    echo "      To run the mission, execute it in the following way:"
    echo "          - ./startCHARMBot.sh run"
    echo ""
    echo "      To kill the mission and points, run the following command"
    echo "          - ./startCHARMBot.sh kill"
    echo ""
    echo "      To launch any individual point, run the next command"
    echo "          - ./startCHARMBot.sh CHARMBotBase"
    echo "          - ./startCHARMBot.sh FrontUVCCamera"
    echo "          - ./startCHARMBot.sh BackUVCCamera"
    echo ""
}

RobotBase() {
    sudo ./../scripts/setCAN.sh 0 1000000
    sleep 1
    sudo -E ../bin/CHARMBotControllerPoint --can_port can0 --configuration ../modules/Robots/CHARMBot/config/CHARMBot.json --port 4000 &
}

FrontCamera() {
    sudo -E ../bin/UVCCameraPoint --device /dev/video2 --protocol tcp --port 5000 &
}

BackCamera() { 
    sudo -E ../bin/UVCCameraPoint --device /dev/video0 --protocol tcp --port 5001 &
}

# Kill Communication Points
KillPoints() {
    echo ""
    echo "--------------------------------------"
    echo " Killing all the Communication Points"
    echo "--------------------------------------"
    echo ""
    sudo killall -SIGTSTP CHARMBotControllerPoint
    sudo killall -SIGTSTP UVCCameraPoint

    sleep 5

    sudo killall CHARMBotCommunicationPoint
    sudo killall UVCCameraPoint
}

StartPoints() {
    echo ""
    echo "----------------------------------------"
    echo " Executing all the Communication Points"
    echo "     - CHARMBot Base     |  TCP Port 4000"
    echo "     - Front UVC Camera  |  TCP Port 5000"
    echo "     - Back UVC Camera   |  TCP Port 5001"
    echo "----------------------------------------"
    echo ""

    RobotBase
    FrontCamera
    BackCamera
}
    

if [ "$#" = 1 ]; then
    export LOGGER_ENABLE_LOGGING=1 && export LOGGER_PRINT_STDOUT=1
    if [ "$1" = "help" ]; then
        Instructions
    elif [ "$1" = "run" ]; then
        StartPoints
    elif [ "$1" = "kill" ]; then
        KillPoints
    elif [ "$1" = "CHARMBotBase" ]; then
        echo "Executing CHARMBot Base - TCP Port 4000"
        RobotBase
    elif [ "$1" = "FrontUVCCamera" ]; then
        echo "Executing Front UVC Camera - TCP Port 5000"
        FrontCamera
    elif [ "$1" = "BackUVCCamera" ]; then
        echo "Executing Back UVC Camera - TCP Port 5001"
        BackCamera
    else 
        Instructions
    fi
else
    Instructions
fi
