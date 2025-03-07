#!/bin/sh

########################################################################################################################
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.   ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch  ##
##                                                                                                                    ##
## Authors: David Forkel CERN BE/CEM/MRO                                                                              ##
##                                                                                                                    ##
##                                                                                                                    ##
########################################################################################################################


######################
## Global variables ##
######################

DIRECTORY=~/Libraries
CORES=$(($(nproc)/2))



##########
## Help ##
##########

Instructions() {
    echo ""
    echo "        The script installs tools for remote desktop functionalities  "
    echo ""
    echo "        To install all the listed libraries execute:"
    echo "            ./installRemoteConnectionTools.sh All"
    echo ""
    echo "        To install specific libraries execute:"
    echo "            ./installRemoteConnectionTools.sh [Library Name 1] [Library Name 2]"
    echo ""
    echo "        The available libraries are":
    echo ""
    echo "          - Remmina"
    echo "          - USBIP"
    echo ""
}



#############
## Remmina ##
#############

Remmina() {
    sudo apt-get install --no-install-recommends --assume-yes remmina
}



###########
## USBIP ##
###########

USBIP() {
    sudo apt-get install --no-install-recommends --assume-yes linux-tools-generic
}



##########################
## Installation Process ##
##########################

Install() {
    echo ""
    echo "----------------------------"
    echo " Installing $1"
    echo "----------------------------"
    echo ""

    if [ ! -d $DIRECTORY ]; then
        mkdir $DIRECTORY
    fi

    sudo apt-get update

    case "$1" in
        "Remmina")           Remmina                                         ;;
        "USBIP")             USBIP                                           ;;
        *)                   printf "The library $1 is not available \n"     ;;
    esac
}

InstallAll() {
    Install Remmina
    Install USBIP
}



##########
## Main ##
##########

if [ "$#" = 1 ]; then
    if [ "$1" = "help" ]; then
        Instructions
    elif [ "$1" = "All" ]; then
        InstallAll
    else
        Install "$1"
    fi
elif [ "$#" -gt 1 ]; then
    for i in "$@"; do
        Install "$i"
    done
else
    Instructions
fi    
