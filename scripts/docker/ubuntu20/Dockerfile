########################################################################################################################
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch  ##
##                                                                                                                    ##
## Authors: Alejandro Diaz Rosales CERN EN/SMM/MRO 2020                                                               ##
##          Yulia Venichenko CERN EN/SMM/MRO 2020                                                                     ##
##                                                                                                                    ##
########################################################################################################################

FROM ubuntu:20.04

WORKDIR /home/

# Basic Tools
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes \
        apt-utils=2.0.10 \
        sudo=1.8.31-1ubuntu1.5 \
        ca-certificates=20230311ubuntu0.20.04.1 \
        wget=1.20.3-1ubuntu2 \
        g++=4:9.3.0-1ubuntu2 \
        gcc=4:9.3.0-1ubuntu2 \
        nasm=2.14.02-1 \
        git=1:2.25.1-1ubuntu3.11 \
        gdb=9.1-0ubuntu1 \
        valgrind=1:3.15.0-1ubuntu9.1 \
        htop=2.2.0-2build1 \
        make=4.2.1-1.2 \
        cmake=3.16.3-1ubuntu1.20.04.1 \
        gcovr=4.2-1 \
        autogen=1:5.18.16-3 \
        autoconf=2.69-11.1 \
        libtool=2.4.6-14 \
        vim=2:8.1.2269-1ubuntu5.20 \
        ssh=1:8.2p1-4ubuntu0.9 \
        unzip=6.0-25ubuntu1 \
        libusb-1.0-0-dev=2:1.0.23-2build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Libraries & Dependencies

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libboost-all-dev=1.71.0.0ubuntu2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libflann-dev=1.9.1+dfsg-9build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libgsl-dev=2.5+dfsg-6+deb10u1build0.20.04.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libsuitesparse-dev=1:5.7.1+dfsg-2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes qtdeclarative5-dev=5.12.8-0ubuntu1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libqglviewer-dev-qt5=2.6.3+dfsg2-6build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libconfig++-dev=1.5-0.4build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libtinyxml-dev=2.6.2-4+deb10u1build0.20.04.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libxml2-dev=2.9.10+dfsg-5ubuntu0.20.04.6 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes freetds-dev=1.1.6-1.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libmysqlclient-dev=8.0.35-0ubuntu0.20.04.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libmodbus-dev=3.1.6-2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes unixodbc-dev=2.3.6-0.1build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libgtk2.0-dev=2.24.32-4ubuntu4 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libcurlpp-dev=0.8.1-2build2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libx265-dev=3.2.1-1build1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libzbar-dev=0.23-1.3 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes qhull-bin=2015.2-4 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes python3-matplotlib=3.1.2-1ubuntu4 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes liburdfdom-dev=1.0.4+ds-2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libx264-dev=2:0.155.2917+git0a84d98-2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libccd-dev=2.1-1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes liblz4-dev=1.9.2-2ubuntu0.20.04.1 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes doxygen=1.8.17-0ubuntu2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes graphviz=2.42.2-3build2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV TZ=Europe
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends --assume-yes libvtk7-dev=7.1.1+dfsg2-2ubuntu1 && \
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

RUN git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && \
    git checkout 3.4.0 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf eigen/

RUN git clone https://github.com/gabime/spdlog.git && \
    cd spdlog && \
    git checkout v1.9.2 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf spdlog/

RUN git clone https://github.com/nlohmann/json.git && \
    cd json && \
    git checkout v3.11.3 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf json/

RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && \
    git checkout 20230223_git && \
    mkdir build && \
    cd build && \
    cmake -DG2O_BUILD_EXAMPLES=OFF ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf g2o/

RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git && \
    cd orocos_kinematics_dynamics && \
    git checkout v1.5.1 && \
    cd orocos_kdl && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    make install && \
    cd ../../../ && \
    rm -rf orocos_kinematics_dynamics/

RUN git clone https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    git checkout v2.7.1 && \
    mkdir build && \
    cd build && \
    cmake -DNLOPT_TESTS=OFF ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf nlopt/

RUN git clone https://github.com/ompl/ompl.git && \
    cd ompl && \
    git checkout 1.6.0 && \
    mkdir build && \
    cd build && \
    cmake -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_TESTS=OFF ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf ompl/

RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    cd DynamixelSDK && \
    git checkout 3.7.51 && \
    cd c++/build/linux64 && \
    make && \
    make install && \
    cd ../../../../ && \
    rm -rf DynamixelSDK/

RUN git clone https://github.com/OpenEtherCATsociety/SOEM.git && \
    cd SOEM && \
    git checkout a901500618405760a564e64a6816705e29f50f9f && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_C_FLAGS=-fPIC -DCMAKE_INSTALL_PREFIX=/usr/local ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf SOEM/

RUN wget https://sourceforge.net/projects/urgnetwork/files/urg_library/urg_library-1.2.7.zip/download && \
    unzip download && \
    rm download && \
    cd urg_library-1.2.7/ && \
    make && \
    make install && \
    cd ../ && \
    rm -rf urg_library-1.2.7/

RUN git clone https://github.com/IntelRealSense/librealsense.git  && \
    cd librealsense && \
    git checkout v2.54.2 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_EXAMPLES=OFF ../ && \
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
    cmake ../ -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DBUILD_TESTS=OFF && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf opencv_contrib/ opencv/

RUN git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl/ && \
    git checkout pcl-1.14.0 && \
    mkdir build && \
    cd build && \
    cmake ../ -DBUILD_examples=OFF && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf pcl/

RUN git clone --recursive https://github.com/Corvusoft/restbed.git && \
    cd restbed && \
    git checkout 4.8 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SSL=OFF -DBUILD_TESTS=OFF ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf restbed/

RUN git clone https://github.com/OctoMap/octomap.git && \
    cd octomap/ && \
    git checkout v1.9.6 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
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
    make && \
    make install && \
    cd ../../ && \
    rm -rf fcl/

RUN git clone https://github.com/lagadic/visp.git && \
    cd visp/ && \
    git checkout v3.4.0 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_DEMOS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TUTORIALS=OFF -DUSE_CXX_STANDARD=17 ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf visp/

RUN git clone https://github.com/boost-ext/sml.git && \
    cd sml/ && \
    git checkout v1.1.9 && \
    mkdir build && \
    cd build && \
    cmake -DSML_BUILD_BENCHMARKS=OFF -DSML_BUILD_DOCS=OFF -DSML_BUILD_EXAMPLES=OFF -DSML_BUILD_TESTS=OFF ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf sml/

RUN git clone https://gitlab.com/sdurobotics/ur_rtde.git && \
    cd ur_rtde && \
    git checkout v1.5.7 && \
    git submodule update --init --recursive && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
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
    wget https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.6.0/linux_x86-64_x86_gcc.zip && \
    unzip linux_x86-64_x86_gcc.zip && \
    rm linux_x86-64_x86_gcc.zip

COPY xethruradarapi xethruradarapi
RUN cd xethruradarapi && \
    git checkout 1.5.3 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    make install && \
    cd ../../ && \
    rm -rf xethruradarapi/

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
LABEL ch.cern.version = "0.4.0"
LABEL ch.cern.released = "Jan 12, 2024"
