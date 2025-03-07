#!/bin/sh

########################################################################################################################
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch  ##
##                                                                                                                    ##
## Authors: Jorge Playan Garai CERN EN/SMM/MRO                                                                        ##
##          Alejandro Diaz Rosales CERN EN/SMM/MRO                                                                    ##
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
    echo "        The script installs most of the dependencies of the CERN Robotic Framework in   "
    echo "    C++ The user can select which libraries to install. Each of them is downloaded and  "
    echo "    compiled inside the folder ~/Libraries."
    echo ""
    echo "        To install all the libraries execute:"
    echo "            ./installDependencies.sh All"
    echo ""
    echo "        To install specific libraries execute:"
    echo "            ./installDependencies.sh [Library Name 1] [Library Name 2]"
    echo ""
    echo "        The available libraries are":
    echo ""
    echo "          - BasicTools"
    echo ""
    echo "          - Boost                 - JSON                  - OpenCV"
    echo "          - SML                   - Libconfig             - ViSP"
    echo "          - SpdLog                - TinyXML               - x264"
    echo "          - PreCommit             - XML2                  - x265"
    echo ""
    echo "          - Eigen                 - URDF                  - ZBar"
    echo "          - FLANN"
    echo "          - GSL                   - cURL                  - FCL"
    echo "          - G2O                   - FreeTDS               - Libccd"
    echo "          - KDL                   - MySQLClient           - Octomap"
    echo "          - NLopt                 - LZ4                   - Qhull"
    echo "          - OMPL                  - Modbus                - PCL"
    echo "          - SuitSparse            - ODBC"
    echo "                                  - Restbed               - Matplotlib"
    echo "          - DynamixelSDK          - Lely                  - QGLViewer"
    echo "          - KinovaAPI             - Vicon"
    echo "          - Kortex                                        - Qt5"
    echo "          - RaptorAPI             - Hokuyo                - GTK"
    echo "          - SOEM                  - IntelRealSense        - VTK"
    echo "          - UniversalRobot        - XeThruRadarAPI"
    echo ""
    echo "        If the user selects a specific library, this script does not take care of       "
    echo "    installing its dependencies. The user must select the order of the installation     "
    echo "    wisely. In the case of choosing all the libraries, they will be installed in the    "
    echo "    right order to guarantee no cross dependecy errors."
    echo ""
    echo "        Along the installation process you might have to write the sudo password and the"
    echo "    CERN credentials to download files located in gitlab.cern.ch"
    echo ""
    echo "        Unsupported libraries:"
    echo "          - CUDA"
    echo "          - OptrisAPI"
    echo "          - XSensAPI"
    echo "          - Tensorflow"
    echo "          - GPUVoxels"
    echo ""
    echo "        To install them go to the section 'Libraries & Dependencies' of the installation"
    echo "    process in the documentation."
    echo ""
    echo "        https://readthedocs.web.cern.ch/pages/viewpage.action?pageId=153518392"
    echo ""
}



##################
## Basic Tools  ##
##################

BasicTools() {
    sudo apt-get install --no-install-recommends --assume-yes \
        apt-utils \
        sudo \
        ca-certificates \
        wget \
        g++ \
        gcc \
        nasm \
        git \
        gdb \
        valgrind \
        htop \
        make \
        cmake \
        gcovr \
        autogen \
        autoconf \
        libtool \
        vim \
        ssh \
        unzip \
        pkg-config \
        can-utils \
        libusb-1.0-0-dev \
        doxygen \
        graphviz
}



######################
## Generic Packages ##
######################

Boost() {
    sudo apt-get install --no-install-recommends --assume-yes libboost-all-dev
}

SML() {
    cd $DIRECTORY || return
    git clone https://github.com/boost-ext/sml.git
    cd sml/ || return
    git checkout v1.1.9
    mkdir build
    cd build || return
    cmake -DSML_BUILD_BENCHMARKS=OFF -DSML_BUILD_EXAMPLES=OFF -DSML_BUILD_TESTS=OFF ../
    make -j$CORES
    sudo make install
}

SpdLog() {
    cd $DIRECTORY || return
    git clone https://github.com/gabime/spdlog.git
    cd spdlog || return
    git checkout v1.9.2
    mkdir build
    cd build || return
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ../
    make -j$CORES
    sudo make install
}


PreCommit() {
    sudo apt-get install --no-install-recommends --assume-yes pre-commit=2.17.0-1
}

###########################
## Mathematical Packages ##
###########################

Eigen() {
    cd $DIRECTORY || return
    git clone https://gitlab.com/libeigen/eigen.git
    cd eigen || return
    git checkout 3.4.0
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

FLANN() {
    sudo apt-get install --no-install-recommends --assume-yes libflann-dev
}

GSL() {
    sudo apt-get install --no-install-recommends --assume-yes libgsl-dev
}

G2O() {
    cd $DIRECTORY || return
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o || return
    git checkout 20230223_git
    mkdir build
    cd build || return
    cmake -DG2O_BUILD_EXAMPLES=OFF ../
    make -j$CORES
    sudo make install
}

KDL() {
    cd $DIRECTORY || return
    git clone https://github.com/orocos/orocos_kinematics_dynamics.git
    cd orocos_kinematics_dynamics || return
    git checkout v1.5.1
    cd orocos_kdl || return
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

NLopt() {
    cd $DIRECTORY || return
    git clone https://github.com/stevengj/nlopt.git
    cd nlopt || return
    git checkout v2.7.1
    mkdir build
    cd build || return
    cmake -DNLOPT_TESTS=OFF ../
    make -j$CORES
    sudo make install
}

OMPL() {
    cd $DIRECTORY || return
    git clone https://github.com/ompl/ompl.git
    cd ompl || return
    git checkout 1.6.0
    mkdir build
    cd build || return
    cmake -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_TESTS=OFF ../
    make -j$CORES
    sudo make install
}

SuitSparse() {
    sudo apt-get install --no-install-recommends --assume-yes libsuitesparse-dev
}



###############
## Acutators ##
###############

DynamixelSDK() {
    cd $DIRECTORY || return
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    cd DynamixelSDK || return
    git checkout 3.7.51
    cd c++/build/linux64 || return
    make -j$CORES
    sudo make install
}

KinovaAPI() {
    cd $DIRECTORY || return
    git clone https://gitlab.cern.ch/en-smm-mro/Robotronics/Libraries/kinovajaco2api.git
    cd kinovajaco2api || return
    git checkout 1.5.1
    cd Architectures/x86/64bits/ || return
    sudo sh InstallAPI64x86.sh
}

Kortex() {
    sudo mkdir /opt/Kortex/
    cd /opt/Kortex/ || return
    sudo wget https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/linux_x86_64_gcc_5.4.zip
    sudo unzip linux_x86_64_gcc_5.4.zip
    sudo rm linux_x86_64_gcc_5.4.zip
}

RaptorAPI() {
    cd $DIRECTORY || return
    wget -nc https://downloads.haption.com/software/release/RaptorAPI/RaptorAPI-1.2_6e2c15d_20230928.zip
    unzip RaptorAPI-1.2_6e2c15d_20230928
    rm RaptorAPI-1.2_6e2c15d_20230928.zip
    cd RaptorAPI/ || return
    sudo mkdir /usr/local/lib/raptorapi-1.2
    sudo cp bin/Linux/x86_64-glibc2.35/libRaptorAPI.so /usr/local/lib/raptorapi-1.2/libRaptorAPI.so
    sudo cp bin/Linux/x86_64-glibc2.35/libSimpleChannelCIFX.so /usr/local/lib/raptorapi-1.2/libSimpleChannelCIFX.so
    sudo cp bin/Linux/x86_64-glibc2.35/libSimpleChannelUDP.so /usr/local/lib/raptorapi-1.2/libSimpleChannelUDP.so
    sudo cp bin/Linux/x86_64-glibc2.35/libVirtuosePICV4.so /usr/local/lib/raptorapi-1.2/libVirtuosePICV4.so
    sudo cp bin/Linux/x86_64-glibc2.35/libAchillePICV4.so /usr/local/lib/raptorapi-1.2/libAchillePICV4.so
    sudo cp bin/Linux/x86_64-glibc2.35/libMAT6D.so /usr/local/lib/raptorapi-1.2/libMAT6D.so
    sudo cp bin/Linux/x86_64-glibc2.35/libAchilleARM.so /usr/local/lib/raptorapi-1.2/libAchilleARM.so
    sudo mkdir /usr/local/include/raptorapi-1.2
    sudo cp include/RaptorAPI.hpp /usr/local/include/raptorapi-1.2/RaptorAPI.hpp
    sudo cp include/RobotTypes.hpp /usr/local/include/raptorapi-1.2/RobotTypes.hpp
    sudo cp include/SimpleChannelInterface.hpp /usr/local/include/raptorapi-1.2/SimpleChannelInterface.hpp
    sudo cp include/Types.hpp /usr/local/include/raptorapi-1.2/Types.hpp
    sudo cp include/Linux/x86_64-glibc2.35/RaptorAPI_Export.h /usr/local/include/raptorapi-1.2/RaptorAPI_Export.h
    echo "/usr/local/lib/raptorapi-1.2" | sudo tee "/etc/ld.so.conf.d/raptorapi.conf"
    sudo ldconfig
}

SOEM() {
    cd $DIRECTORY || return
    git clone https://github.com/OpenEtherCATsociety/SOEM.git
    cd SOEM || return
    git checkout a901500618405760a564e64a6816705e29f50f9f
    mkdir build
    cd build || return
    cmake -DCMAKE_C_FLAGS=-fPIC -DCMAKE_INSTALL_PREFIX=/usr/local ../
    make -j$CORES
    sudo make install
}

UniversalRobot() {
    cd $DIRECTORY || return
    git clone https://gitlab.com/sdurobotics/ur_rtde.git
    cd ur_rtde || return
    git checkout v1.5.7
    git submodule update --init --recursive
    mkdir build
    cd build || return
    cmake ..
    make -j$CORES
    sudo make install
}



#############
## Sensors ##
#############

Hokuyo() {
    cd $DIRECTORY || return
    wget -nc https://sourceforge.net/projects/urgnetwork/files/urg_library/urg_library-1.2.7.zip/download
    unzip download
    rm download
    cd urg_library-1.2.7/ || return
    make -j$CORES
    sudo make install
}

IntelRealSense() {
    cd $DIRECTORY || return
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense || return
    git checkout v2.54.2
    sh ./scripts/patch-realsense-ubuntu-lts-hwe.sh
    mkdir build
    cd build || return
    cmake -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DCHECK_FOR_UPDATES=OFF ../
    make -j$CORES
    sudo make install
}

XeThruRadarAPI() {
    cd $DIRECTORY || return
    git clone https://gitlab.cern.ch/en-smm-mro/Robotronics/Libraries/xethruradarapi.git
    cd xethruradarapi || return
    git checkout 1.5.3
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

RPLiDARSDK() {
    cd $DIRECTORY || return
    git clone https://github.com/Slamtec/rplidar_sdk.git
    cd rplidar_sdk || return
    git checkout 9d85aae434d39fc4014018dcbf8711f8918b0254
    make -j$CORES
    sudo mkdir /usr/local/include/rplidar_sdk
    sudo cp -R sdk/include/. /usr/local/include/rplidar_sdk
    sudo cp -R sdk/src/. /usr/local/include/rplidar_sdk
    sudo cp output/Linux/Release/libsl_lidar_sdk.a /usr/local/lib/libsl_lidar_sdk.a
}

UnitreeL1SDK() {
    cd $DIRECTORY || return
    git clone https://github.com/unitreerobotics/unilidar_sdk.git
    cd  unilidar_sdk || return
    git checkout 1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6
    cd  unitree_lidar_sdk || return
    mkdir -p build
    cd  build || return
    cmake -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DCHECK_FOR_UPDATES=OFF ../
    make -j$CORES
    cd  .. || return
    sudo mkdir -p /usr/local/include/unilidar_sdk
    sudo cp -R include/. /usr/local/include/unilidar_sdk
    sudo cp lib/x86_64/libunitree_lidar_sdk.a /usr/local/lib/libunitree_lidar_sdk.a
}



###################################################
## Serialization and Configuration File Packages ##
###################################################

JSON() {
    cd $DIRECTORY || return
    git clone https://github.com/nlohmann/json.git
    cd json || return
    git checkout v3.11.3
    mkdir build
    cd build || return
    cmake -DJSON_BuildTests=OFF ../
    make -j$CORES
    sudo make install
}

Libconfig() {
    sudo apt-get install --no-install-recommends --assume-yes libconfig++-dev
}

TinyXML() {
    sudo apt-get install --no-install-recommends --assume-yes libtinyxml-dev
}

XML2() {
    sudo apt-get install --no-install-recommends --assume-yes  libxml2-dev
}

URDF() {
    sudo apt-get install --no-install-recommends --assume-yes  liburdfdom-dev
}



############################
## Communication Packages ##
############################

cURL() {
    sudo apt-get install --no-install-recommends --assume-yes libcurlpp-dev
}

FreeTDS() {
    sudo apt-get install --no-install-recommends --assume-yes freetds-dev
}

MySQLClient() {
    sudo apt-get install --no-install-recommends --assume-yes libmysqlclient-dev
}

LZ4() {
    sudo apt-get install --no-install-recommends --assume-yes liblz4-dev
}

Modbus() {
    sudo apt-get install --no-install-recommends --assume-yes libmodbus-dev
}

ODBC() {
    sudo apt-get install --no-install-recommends --assume-yes unixodbc-dev
}

Restbed() {
    cd $DIRECTORY || return
    git clone --recursive https://github.com/Corvusoft/restbed.git
    cd restbed || return
    git checkout 4.8
    mkdir build
    cd build || return
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SSL=OFF -DBUILD_TESTS=OFF ../
    make -j$CORES
    sudo make install
}

Lely() {
    sudo apt-get update
    sudo apt-get install --no-install-recommends --assume-yes software-properties-common
    sudo add-apt-repository ppa:lely/ppa
    sudo apt-get update
    sudo apt-get install --no-install-recommends --assume-yes liblely-coapp-dev liblely-co-tools python3-dcf-tools
}

Vicon() {
    cd $DIRECTORY || return
    mkdir Vicon/
    cd Vicon/
    wget -nc https://cernbox.cern.ch/remote.php/dav/public-files/nv8FjTvMbdMWMuE/ViconDataStreamSDK_1.12_145507h.zip?access_token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJyZXZhIiwiZXhwIjoxNzAwNDM0Nzk5LCJpYXQiOjE2OTk5NTcwNDAsImlzcyI6Imh0dHBzOi8vYXV0aC5jZXJuLmNoL2F1dGgvcmVhbG1zL2Nlcm4iLCJ1c2VyIjp7ImlkIjp7ImlkcCI6Imh0dHBzOi8vYXV0aC5jZXJuLmNoL2F1dGgvcmVhbG1zL2Nlcm4iLCJvcGFxdWVfaWQiOiJzY29sbG9tYiIsInR5cGUiOjF9LCJ1c2VybmFtZSI6InNjb2xsb21iIiwibWFpbCI6InNlYmFzdGllbi5jb2xsb21iQGNlcm4uY2giLCJkaXNwbGF5X25hbWUiOiJTZWJhc3RpZW4gQ29sbG9tYiIsInVpZF9udW1iZXIiOjE1ODQ3NCwiZ2lkX251bWJlciI6Mjc2Nn0sInNjb3BlIjp7InVzZXIiOnsicmVzb3VyY2UiOnsiZGVjb2RlciI6Impzb24iLCJ2YWx1ZSI6ImV5SndZWFJvSWpvaUx5SjkifSwicm9sZSI6MX19fQ.8lEEEC0aMko7QpsgIvOLfqnLSD1oEQJLj8JJsHGF3Iw -O ViconDataStreamSDK_1.12_145507h.zip
    unzip ViconDataStreamSDK_1.12_145507h.zip
    rm ViconDataStreamSDK_1.12_145507h.zip
    cd 20230413_145507h/Release/Linux64/
    sudo mkdir -p /usr/local/lib/vicondatastreamsdk-1.12/
    sudo cp *.so* /usr/local/lib/vicondatastreamsdk-1.12/
    sudo mkdir -p /usr/local/include/vicondatastreamsdk-1.12/vicon/
    sudo cp *.h /usr/local/include/vicondatastreamsdk-1.12/vicon/
}

#########################################
## Image and Video Processing Packages ##
#########################################

OpenCV() {
    sudo apt-get install libopenmpi-dev
    cd $DIRECTORY || return
    git clone https://github.com/opencv/opencv.git
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv_contrib || return
    git checkout 4.9.0
    MODULES=$(pwd)/modules
    cd ../opencv || return
    git checkout 4.9.0
    mkdir build
    cd build || return
    cmake -DOPENCV_EXTRA_MODULES_PATH="$MODULES" -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF ../
    make -j$CORES
    sudo make install
}


ViSP() {
    cd $DIRECTORY || return
    git clone https://github.com/lagadic/visp.git
    cd visp/ || return
    git checkout v3.6.0
    mkdir build
    cd build/ || return
    cmake -DBUILD_DEMOS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TUTORIALS=OFF -DUSE_CXX_STANDARD=17 ../
    make -j$CORES
    sudo make install
}

x264() {
    sudo apt-get install --no-install-recommends --assume-yes libx264-dev
}

x265() {
    sudo apt-get install --no-install-recommends --assume-yes libx265-dev
}

ZBar() {
    sudo apt-get install --no-install-recommends --assume-yes libzbar-dev
}



#################################
## 3D Data Processing Packages ##
#################################

FCL() {
    cd $DIRECTORY || return
    git clone https://github.com/flexible-collision-library/fcl.git
    cd fcl || return
    git checkout 0.7.0
    sed -i '241d' ./CMakeLists.txt
    mkdir build
    cd build/ || return
    cmake -DBUILD_TESTING=OFF ../
    make -j$CORES
    sudo make install
}

Libccd() {
    sudo apt-get install --no-install-recommends --assume-yes libccd-dev
}

Octomap() {
    cd $DIRECTORY || return
    git clone https://github.com/OctoMap/octomap.git
    cd octomap/ || return
    git checkout v1.9.6
    mkdir build
    cd build/ || return
    cmake ../
    make -j$CORES
    sudo make install
}

Qhull() {
    sudo apt-get install --no-install-recommends --assume-yes qhull-bin=2020.2-4
}

PCL() {
    cd $DIRECTORY || return
    git clone https://github.com/PointCloudLibrary/pcl.git
    cd pcl/ || return
    git checkout pcl-1.14.0
    mkdir build
    cd build || return
    make clean
    cmake -DBUILD_examples=OFF ../
    make -j6
    sudo make install
}

Gicp () {
    cd $DIRECTORY || return
    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp/ || return
    mkdir build
    cd build || return
    cmake -DBUILD_examples=OFF ../
    make -j$CORES
    sudo make install
}



######################################
## Visualization and User Interface ##
######################################

Matplotlib() {
    sudo apt-get install --no-install-recommends --assume-yes python3-matplotlib
}

Qt5() {
    sudo apt-get install --no-install-recommends --assume-yes qtdeclarative5-dev
}

QGLViewer() {
    sudo apt-get install --no-install-recommends --assume-yes libqglviewer-dev-qt5
}

VTK() {
    sudo apt-get install --no-install-recommends --assume-yes libvtk9-dev
}

GTK() {
    sudo apt-get install --no-install-recommends --assume-yes libgtk-3-dev
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
        "BasicTools")        BasicTools                                      ;;
        "Boost")             Boost                                           ;;
        "SML")               SML                                             ;;
        "SpdLog")            SpdLog                                          ;;
        "PreCommit")         PreCommit                                       ;;   
        "Eigen")             Eigen                                           ;;
        "FLANN")             FLANN                                           ;;
        "GSL")               GSL                                             ;;
        "G2O")               G2O                                             ;;
        "KDL")               KDL                                             ;;
        "NLopt")             NLopt                                           ;;
        "OMPL")              OMPL                                            ;;
        "SuitSparse")        SuitSparse                                      ;;
        "DynamixelSDK")      DynamixelSDK                                    ;;
        "KinovaAPI")         KinovaAPI                                       ;;
        "Kortex")            Kortex                                          ;;
        "RaptorAPI")         RaptorAPI                                       ;;
        "SOEM")              SOEM                                            ;;
        "UniversalRobot")    UniversalRobot                                  ;;
        "Hokuyo")            Hokuyo                                          ;;
        "IntelRealSense")    IntelRealSense                                  ;;
        "XeThruRadarAPI")    XeThruRadarAPI                                  ;;
        "JSON")              JSON                                            ;;
        "Libconfig")         Libconfig                                       ;;
        "TinyXML")           TinyXML                                         ;;
        "XML2")              XML2                                            ;;
        "URDF")              URDF                                            ;;
        "cURL")              cURL                                            ;;
        "FreeTDS")           FreeTDS                                         ;;
        "MySQLClient")       MySQLClient                                     ;;
        "LZ4")               LZ4                                             ;;
        "Modbus")            Modbus                                          ;;
        "ODBC")              ODBC                                            ;;
        "Restbed")           Restbed                                         ;;
        "Lely")              Lely                                            ;;
        "Vicon")             Vicon                                           ;;
        "OpenCV")            OpenCV                                          ;;
        "ViSP")              ViSP                                            ;;
        "x264")              x264                                            ;;
        "x265")              x265                                            ;;
        "ZBar")              ZBar                                            ;;
        "FCL")               FCL                                             ;;
        "Libccd")            Libccd                                          ;;
        "Octomap")           Octomap                                         ;;
        "Qhull")             Qhull                                           ;;
        "PCL")               PCL                                             ;;
        "Matplotlib")        Matplotlib                                      ;;
        "QGLViewer")         QGLViewer                                       ;;
        "Qt5")               Qt5                                             ;;
        "VTK")               VTK                                             ;;
        "GTK")               GTK                                             ;;
        "RPLiDARSDK")        RPLiDARSDK                                      ;;
        "UnitreeL1SDK")      UnitreeL1SDK                                    ;;
        "Gicp")              Gicp                                            ;;
        *)                   printf "The library $1 is not available \n"     ;;
    esac
}

InstallAll() {
    Install BasicTools

    Install Boost
    Install PreCommit
    Install Eigen
    Install TinyXML
    Install XML2
    Install x264
    Install x265
    Install LZ4
    Install ZBar
    Install FLANN
    Install VTK
    Install Qhull
    Install Libccd
    Install GSL
    Install SuitSparse
    Install Qt5
    Install QGLViewer
    Install Libconfig
    Install FreeTDS
    Install MySQLClient
    Install Modbus
    Install ODBC
    Install GTK
    Install cURL
    Install Matplotlib
    Install URDF
    Install NLopt
    Install KDL
    Install G2O
    Install OMPL
    Install IntelRealSense
    Install OpenCV
    Install PCL
    Install Octomap
    Install FCL
    Install SpdLog
    Install JSON
    Install SML
    Install Restbed
    Install Lely
    Install Vicon
    Install DynamixelSDK
    Install SOEM
    Install UniversalRobot
    Install KinovaAPI
    Install Kortex
    Install RaptorAPI
    Install XeThruRadarAPI
    Install Hokuyo
    Install ViSP
    Install Gicp
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
