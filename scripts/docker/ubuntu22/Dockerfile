########################################################################################################################
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch  ##
##                                                                                                                    ##
## Authors: Alejandro Diaz Rosales CERN EN/SMM/MRO 2020                                                               ##
##          Yulia Venichenko CERN EN/SMM/MRO 2020                                                                     ##
##                                                                                                                    ##
########################################################################################################################

FROM ubuntu:22.04

WORKDIR /home/

# Basic Tools
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes \
        apt-utils=2.4.12 \
        sudo=1.9.9-1ubuntu2.4 \
        ca-certificates=20230311ubuntu0.22.04.1 \
        wget=1.21.2-2ubuntu1 \
        g++=4:11.2.0-1ubuntu1 \
        gcc=4:11.2.0-1ubuntu1 \
        nasm=2.15.05-1 \
        git=1:2.34.1-1ubuntu1.11 \
        gdb=12.1-0ubuntu1~22.04.2 \
        valgrind=1:3.18.1-1ubuntu2 \
        htop=3.0.5-7build2 \
        make=4.3-4.1build1 \
        cmake=3.22.1-1ubuntu1.22.04.2 \
        gcovr=5.0-2 \
        autogen=1:5.18.16-4 \
        autoconf=2.71-2 \
        libtool=2.4.6-15build2 \
        vim=2:8.2.3995-1ubuntu2.17 \
        ssh=1:8.9p1-3ubuntu0.10 \
        unzip=6.0-26ubuntu3.1 \
        pkg-config \
        libusb-1.0-0-dev=2:1.0.25-1ubuntu2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Libraries & Dependencies

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes pre-commit=2.17.0-1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libboost-all-dev=1.74.0.3ubuntu7 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libeigen3-dev=3.4.0-2ubuntu2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libtinyxml-dev=2.6.2-6ubuntu0.22.04.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libxml2-dev=2.9.13+dfsg-1ubuntu0.4 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libx264-dev=2:0.163.3060+git5db6aa6-2build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libx265-dev=3.5-2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes liblz4-dev=1.9.3-2build2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libzbar-dev=0.23.92-4build2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libflann-dev=1.9.1+dfsg-11 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libvtk9-dev=9.1.0+really9.1.0+dfsg2-3build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes qhull-bin=2020.2-4 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libccd-dev=2.1-2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libgsl-dev=2.7.1+dfsg-3 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libsuitesparse-dev=1:5.10.1+dfsg-4build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes qtdeclarative5-dev=5.15.3+dfsg-1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libqglviewer-dev-qt5=2.6.3+dfsg2-9 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libconfig++-dev=1.5-0.4build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes freetds-dev=1.3.6-1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libmysqlclient-dev=8.0.37-0ubuntu0.22.04.3 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libmodbus-dev=3.1.6-2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes unixodbc-dev=2.3.9-5ubuntu0.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libgtk-3-dev=3.24.33-1ubuntu2.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libcurlpp-dev=0.8.1-5 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes python3-matplotlib=3.5.1-2build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes liburdfdom-dev=3.0.1-1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes doxygen=1.9.1-2ubuntu2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes graphviz=2.42.2-6 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# apt-get update && apt-get install software properties must be done together, on the same line before the
# add-apt-repository ppa:lely/ppa. If they are done separately, it will not work. Online forums state this is a bug with Docker.
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository ppa:lely/ppa && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes liblely-coapp-dev liblely-co-tools python3-dcf-tools && \
    pkg-config --cflags liblely-coapp && \
    pkg-config --libs liblely-coapp && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    git checkout v2.7.1 && \
    mkdir build && \
    cd build && \
    cmake -DNLOPT_TESTS=OFF ../ && \
    make install && \
    cd ../../ && \
    rm -rf nlopt/

RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git && \
    cd orocos_kinematics_dynamics && \
    git checkout v1.5.1 && \
    cd orocos_kdl && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make install && \
    cd ../../../ && \
    rm -rf orocos_kinematics_dynamics/

RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && \
    git checkout 20230223_git && \
    mkdir build && \
    cd build && \
    cmake -DG2O_BUILD_EXAMPLES=OFF ../ && \
    make install && \
    cd ../../ && \
    rm -rf g2o/

RUN git clone https://github.com/ompl/ompl.git && \
    cd ompl && \
    git checkout 1.6.0 && \
    mkdir build && \
    cd build && \
    cmake -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_TESTS=OFF ../ && \
    make install && \
    cd ../../ && \
    rm -rf ompl/

RUN git clone https://github.com/IntelRealSense/librealsense.git  && \
    cd librealsense && \
    git checkout v2.54.2 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_EXAMPLES=OFF -DCHECK_FOR_UPDATES=OFF ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf librealsense/

RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 4.9.0 && \
    cd ../opencv && \
    git checkout 4.9.0 && \
    mkdir build && \
    cd build && \
    cmake ../ -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF && \
    make install && \
    cd ../../ && \
    rm -rf opencv_contrib/ opencv/

RUN git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl/ && \
    git checkout pcl-1.14.0 && \
    mkdir build && \
    cd build && \
    cmake ../ -DBUILD_examples=OFF && \
    make install && \
    cd ../../ && \
    rm -rf pcl/

RUN git clone https://github.com/OctoMap/octomap.git && \
    cd octomap/ && \
    git checkout v1.9.6 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make install && \
    cd ../../ && \
    rm -rf octomap/

RUN git clone https://github.com/flexible-collision-library/fcl.git && \
    cd fcl/ && \
    git checkout 0.7.0 && \
    sed -i '241d' ./CMakeLists.txt && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_TESTING=OFF ../ && \
    make install && \
    cd ../../ && \
    rm -rf fcl/

RUN git clone https://github.com/gabime/spdlog.git && \
    cd spdlog && \
    git checkout v1.9.2 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ../ && \
    make install && \
    cd ../../ && \
    rm -rf spdlog/

RUN git clone https://github.com/nlohmann/json.git && \
    cd json && \
    git checkout v3.11.3 && \
    mkdir build && \
    cd build && \
    cmake -DJSON_BuildTests=OFF ../ && \
    make install && \
    cd ../../ && \
    rm -rf json/

RUN git clone https://github.com/boost-ext/sml.git && \
    cd sml/ && \
    git checkout v1.1.9 && \
    mkdir build && \
    cd build && \
    cmake -DSML_BUILD_BENCHMARKS=OFF -DSML_BUILD_EXAMPLES=OFF -DSML_BUILD_TESTS=OFF ../ && \
    make install && \
    cd ../../ && \
    rm -rf sml/

RUN git clone --recursive https://github.com/Corvusoft/restbed.git && \
    cd restbed && \
    git checkout 4.8 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SSL=OFF -DBUILD_TESTS=OFF ../ && \
    make install && \
    cd ../../ && \
    rm -rf restbed/

RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    cd DynamixelSDK && \
    git checkout 3.7.51 && \
    cd c++/build/linux64 && \
    make install && \
    cd ../../../../ && \
    rm -rf DynamixelSDK/

RUN git clone https://github.com/OpenEtherCATsociety/SOEM.git && \
    cd SOEM && \
    git checkout a901500618405760a564e64a6816705e29f50f9f && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_C_FLAGS=-fPIC -DCMAKE_INSTALL_PREFIX=/usr/local ../ && \
    make install && \
    cd ../../ && \
    rm -rf SOEM/

RUN git clone https://gitlab.com/sdurobotics/ur_rtde.git && \
    cd ur_rtde && \
    git checkout v1.5.7 && \
    git submodule update --init --recursive && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make install && \
    cd ../../ && \
    rm -rf ur_rtde/

COPY kinovajaco2api kinovajaco2api
RUN cd kinovajaco2api && \
    git checkout 1.5.1 && \
    cd Architectures/x86/64bits/ && \
    sh InstallAPI64x86.sh && \
    cd ../../../../ && \
    rm -rf kinovajaco2api/

RUN mkdir /opt/Kortex/ && \
    cd /opt/Kortex/ && \
    wget https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/linux_x86_64_gcc_5.4.zip && \
    unzip linux_x86_64_gcc_5.4.zip && \
    rm linux_x86_64_gcc_5.4.zip

RUN wget -nc https://downloads.haption.com/software/release/RaptorAPI/RaptorAPI-1.2_6e2c15d_20230928.zip && \
    unzip RaptorAPI-1.2_6e2c15d_20230928 && \
    rm RaptorAPI-1.2_6e2c15d_20230928.zip && \
    cd RaptorAPI/ || return && \
    sudo mkdir /usr/local/lib/raptorapi-1.2 && \
    sudo cp bin/Linux/x86_64-glibc2.35/libRaptorAPI.so /usr/local/lib/raptorapi-1.2/libRaptorAPI.so && \
    sudo cp bin/Linux/x86_64-glibc2.35/libSimpleChannelCIFX.so /usr/local/lib/raptorapi-1.2/libSimpleChannelCIFX.so && \
    sudo cp bin/Linux/x86_64-glibc2.35/libSimpleChannelUDP.so /usr/local/lib/raptorapi-1.2/libSimpleChannelUDP.so && \
    sudo cp bin/Linux/x86_64-glibc2.35/libVirtuosePICV4.so /usr/local/lib/raptorapi-1.2/libVirtuosePICV4.so && \
    sudo cp bin/Linux/x86_64-glibc2.35/libAchillePICV4.so /usr/local/lib/raptorapi-1.2/libAchillePICV4.so && \
    sudo cp bin/Linux/x86_64-glibc2.35/libMAT6D.so /usr/local/lib/raptorapi-1.2/libMAT6D.so && \
    sudo cp bin/Linux/x86_64-glibc2.35/libAchilleARM.so /usr/local/lib/raptorapi-1.2/libAchilleARM.so && \
    sudo mkdir /usr/local/include/raptorapi-1.2 && \
    sudo cp include/RaptorAPI.hpp /usr/local/include/raptorapi-1.2/RaptorAPI.hpp && \
    sudo cp include/RobotTypes.hpp /usr/local/include/raptorapi-1.2/RobotTypes.hpp && \
    sudo cp include/SimpleChannelInterface.hpp /usr/local/include/raptorapi-1.2/SimpleChannelInterface.hpp && \
    sudo cp include/Types.hpp /usr/local/include/raptorapi-1.2/Types.hpp && \
    sudo cp include/Linux/x86_64-glibc2.35/RaptorAPI_Export.h /usr/local/include/raptorapi-1.2/RaptorAPI_Export.h && \
    echo "/usr/local/lib/raptorapi-1.2" | sudo tee "/etc/ld.so.conf.d/raptorapi.conf" && \
    sudo ldconfig && \
    cd ../ && \
    rm -rf RaptorAPI/

COPY xethruradarapi xethruradarapi
RUN cd xethruradarapi && \
    git checkout 1.5.3 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make install && \
    cd ../../ && \
    rm -rf xethruradarapi/

RUN wget https://sourceforge.net/projects/urgnetwork/files/urg_library/urg_library-1.2.7.zip/download && \
    unzip download && \
    rm download && \
    cd urg_library-1.2.7/ && \
    make install && \
    cd ../ && \
    rm -rf urg_library-1.2.7/

RUN git clone https://github.com/lagadic/visp.git && \
    cd visp/ && \
    git checkout v3.6.0 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_DEMOS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TUTORIALS=OFF -DUSE_CXX_STANDARD=17 ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf visp/

RUN wget -nc https://cernbox.cern.ch/remote.php/dav/public-files/nv8FjTvMbdMWMuE/ViconDataStreamSDK_1.12_145507h.zip?access_token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJyZXZhIiwiZXhwIjoxNzAwNDM0Nzk5LCJpYXQiOjE2OTk5NTcwNDAsImlzcyI6Imh0dHBzOi8vYXV0aC5jZXJuLmNoL2F1dGgvcmVhbG1zL2Nlcm4iLCJ1c2VyIjp7ImlkIjp7ImlkcCI6Imh0dHBzOi8vYXV0aC5jZXJuLmNoL2F1dGgvcmVhbG1zL2Nlcm4iLCJvcGFxdWVfaWQiOiJzY29sbG9tYiIsInR5cGUiOjF9LCJ1c2VybmFtZSI6InNjb2xsb21iIiwibWFpbCI6InNlYmFzdGllbi5jb2xsb21iQGNlcm4uY2giLCJkaXNwbGF5X25hbWUiOiJTZWJhc3RpZW4gQ29sbG9tYiIsInVpZF9udW1iZXIiOjE1ODQ3NCwiZ2lkX251bWJlciI6Mjc2Nn0sInNjb3BlIjp7InVzZXIiOnsicmVzb3VyY2UiOnsiZGVjb2RlciI6Impzb24iLCJ2YWx1ZSI6ImV5SndZWFJvSWpvaUx5SjkifSwicm9sZSI6MX19fQ.8lEEEC0aMko7QpsgIvOLfqnLSD1oEQJLj8JJsHGF3Iw -O ViconDataStreamSDK_1.12_145507h.zip && \
    unzip ViconDataStreamSDK_1.12_145507h.zip && \
    rm ViconDataStreamSDK_1.12_145507h.zip && \
    cd 20230413_145507h/Release/Linux64/ && \
    mkdir -p /usr/local/lib/vicondatastreamsdk-1.12/ && \
    cp *.so* /usr/local/lib/vicondatastreamsdk-1.12/ && \
    mkdir -p /usr/local/include/vicondatastreamsdk-1.12/vicon/ && \
    cp *.h /usr/local/include/vicondatastreamsdk-1.12/vicon/ && \
    cd ../../../ && \
    rm -rf 20230413_145507h/

# Create non root user in the sudo group.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN useradd --create-home --shell /bin/bash --gid sudo robotronics
RUN echo 'robotronics:robotronics' | chpasswd
USER robotronics
WORKDIR /home/robotronics

# General Information
LABEL ch.cern.maintainer = "Alejandro Diaz Rosales <alejandro.diaz.rosales@cern.ch>"
LABEL ch.cern.section = "Mechatronics Robotics and Operations - BE/CEM/MRO"
LABEL ch.cern.project = "CERN Robotic Framework - C++"
LABEL ch.cern.version = "0.4.1"
LABEL ch.cern.released = "July 5, 2024"
