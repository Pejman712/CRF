# Libraries & Dependencies {#libraries_and_dependencies}

Different libraries are required for the CppRoboticFramework. After installing the \ref basic_tools continue with the installation of the libraries you need depending on the modules you want to use.

An installation script is available in the repository to automatically install all the packages listed below. The script is launched as follows:

````bash
./installDependencies All
````

This command requires sudo priviliges, so it will ask for the PC credentials. The argument All will also install the Basic Tools if you haven't done it. For more details on the utilization of the script execute:

````bash
./installDependencies help
````

Installation of the dependencies can take approximately 3h. After this is completed you can continue on the next page \ref compilation_and_execution

#### Generic Packages

| Name | Description | License |
| :-: | :-: | :-: |
| [Boost](https://packages.ubuntu.com/focal/libboost-all-dev) | Boost provides free peer-reviewed portable C++ source libraries. We emphasize libraries that work well with the C++ Standard Library. | Boost Software License 1.0 |
| [SML](https://github.com/boost-ext/sml) | Ccalable C++14 one header only State Machine Library with no dependencies | Boost Software License 1.0 |
| [SpdLog](https://github.com/gabime/spdlog) | Very fast, header only, C++ logging library. | MIT License |

#### Mathematical Packages

| Name | Description | License |
| :-: | :-: | :-: |
| [Eigen](https://gitlab.com/libeigen/eigen) | Eigen is a high-level C++ library of template headers for linear algebra, matrix and vector operations, geometrical transformations, numerical solvers and related algorithms. | Mozilla Public License 2.0 |
| FLANN | FLANN is a library for performing fast approximate nearest neighbor searches in high dimensional spaces. |  |
| [GSL](https://www.gnu.org/software/gsl/doc/html/intro.html) | The GNU Scientific Library (GSL) is a collection of routines for numerical analysis. The routines are written from scratch by the GSL team in C, and present a modern API for C programs while allowing wrappers to be written for very high-level languages. |  |
| G2O | This library contains tools to generate a graph and optimize it. |  |
| KDL | The Kinematics and Dynamics Library is an Orocos project to supply real-time usable code for rigid body kinematics calculations and representations for kinematic structures and their inverse and forward kinematic solvers. |  |
| NLopt | This is a library for nonlinear local and global optimization, for functions with and without gradient information. It is designed as a simple, unified interface and packaging of several free/open-source nonlinear optimization libraries. |  |
| OMPL | The Open Motion Planning Library consists of a set of sampling-based motion planning algorithms. There is no environment specification, no collision detection or visualization. |  |
| SuitSparse | SuiteSparse is a suite of sparse matrix algorithms |  |

#### Actuators

| Name | Description | License |
| :-: | :-: | :-: |
| DynamixelSDK | Dynamixel SDK is a software development kit that provides Dynamixel control functions using packet communication. The API is designed for Dynamixel actuators and Dynamixel-based platforms. |  |
| KinovaAPI | The Kinova Software Development Kit is a complete set of interface, documentation, examples and software tools that help the developer interact with the Kinova Gen2 Ultra-lightweight robot. |  |
| Kortex |  |  |
| RaptorAPI |  |  |
| UniversalRobot |  |  |

#### Sensors

| Name | Description | License |
| :-: | :-: | :-: |
| Hokuyo | Library to control and read data from the Hokuyo URG/UTM/UST infrared laser range scanners that provide range measurements to nearby objects using LIDAR technology. |  |
| IntelRealSense | The Intel RealSense SDK 2.0 is a cross-platform library for the D400 series and the SR300 depth cameras and the T265 tracking camera. |  |
| XeThruRadarAPI | Library that allows you to use the XeThru Radars. |  |

#### Serialization and Configuration Files Packages

| Name | Description | License |
| :-: | :-: | :-: |
| JSON | The JavaScript Object Notation is a lightweight data-interchange format. It is easy for humans to read and write. It is easy for machines to parse and generate. |  |
| Libconfig | This is a simple library for processing structured configuration files, like this one. This file format is more compact and more readable than XML. And unlike XML, it is type-aware, so it is not necessary to do string parsing in application code. |  |
| TinyXML | TinyXML is a simple, small, C++ XML parser that can be easily integrating into other programs. |  |
| XML2 | Libxml2 is the XML C parser and toolkit developed for the Gnome project. XML is a metalanguage to let you design your markup language. |  |
| URDF |  |  |

#### Communication Packages

| Name | Description | License |
| :-: | :-: | :-: |
| cURL | The multiprotocol file transfer library is a free and easy-to-use client-side URL transfer library, supporting FTP, FTPS, HTTP, HTTPS, GOPHER, TELNET, DICT, FILE, and LDAP. Then cURLpp is a C++ wrapper for libcURL. |  |
| FreeTDS | This is an implementation of the Tabular DataStream protocol, used for connecting to MS SQL and Sybase servers over TCP/IP. |  |
| MySQLClient | This library contains the C API for mysql connection. |  |
| LZ4 | LZ4 is lossless compression algorithm, providing compression speed. |  |
| Modbus | The libmodbus free software library sends and receives data according to the Modbus protocol. This library is written in C and supports RTU (serial) and TCP (Ethernet) communications. |  |
| ODBC | This package contains the development files for unixODBC, and implementation of the Open Database Connectivity interface for Unix systems. |  |
| Restbed | Comprehensive and consistent programming model for building applications that require seamless and secure communication over HTTP, with the ability to model a range of business processes, designed to target mobile, tablet, desktop, and embedded production environments. |  |
| Lely |  |  |
| SOEM | This is an EtherCAT library for the control of EtherCAT devices. |  |

#### Image and Video Processing Packages

| Name | Description | License |
| :-: | :-: | :-: |
| OpenCV | The Open Source Computer Vision Library is an open-source computer vision and machine learning software library. |  |
| ViSP | The Visual Servoing Platform library allows prototyping and developing applications using visual tracking and visual servoing technics. |  |
| x264 | x264 is a free software library and application for encoding video streams into the H.264/MPEG-4 AVC. |  |
| x265 | x265 is a library for encoding video into the High Efficiency Video Coding (HEVC/H.265) video compression format that was developed and standardized by the ISO/IEC MPEG and ITU-T VCEG. |  |
| ZBar | This is a library for scanning and decoding bar codes from various sources such as video streams, image files or raw intensity sensors. It supports EAN-13/UPC-A, UPC-E, EAN-8, Code 128, Code 39, Interleaved 2 of 5 and QR Code. |  |

#### 3D Data Processing Packages

| Name | Description | License |
| :-: | :-: | :-: |
| FCL | The Flexible Collision Library can perform collision detection and distance computation on a pair of geometric models composed of triangles. |  |
| Libccd | libccd is library for a collision detection between two convex shapes. |  |
| Octomap | The OctoMap library implements a 3D occupancy grid mapping approach, providing data structures and mapping algorithms in C++ particularly suited for robotics. |  |
| Qhull |  |  |
| PCL | The Point Cloud Library is a large scale, open project for point cloud processing. There are algorithms like filtering, feature estimation, surface reconstruction, registration, model fitting, and segmentation. |  |

#### Visualization and User Interface

| Name | Description | License |
| :-: | :-: | :-: |
| Matplotlib | Matplotlib is a comprehensive library for creating static, animated, and interactive visualizations in Python. |  |
| QGLViewer | libQGLViewer is a C++ library based on Qt that eases the creation of OpenGL 3D viewers. |  |
| Qt5 | Qt is a cross-platform C++ application framework. Qt's primary feature is its rich set of widgets that provide standard GUI functionality. |  |
| GTK |  |  |
| VTK | The Visualization Toolkit (VTK) is open source software for manipulating and displaying scientific data. It comes with state-of-the-art tools for 3D rendering, a suite of widgets for 3D interaction, and extensive 2D plotting capability. |  |
