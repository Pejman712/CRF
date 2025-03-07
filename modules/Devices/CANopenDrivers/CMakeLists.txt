#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Natalija Topalovic CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "CANopenDrivers")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS
    EventLogger
    CommonInterfaces
    ErrorHandler
)

set(INCLUDE_DIRS
    ./include
)

set(SRC_FILES)

if (NOT DISABLE_CiA402)
    set(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${nlohmann_json_INCLUDE_DIRS}
    )

    set(SRC_FILES
        ${SRC_FILES}
        ./src/CiA402/ModesOfOperation/PVM/PVM.cpp
        ./src/CiA402/ModesOfOperation/PVM/PVMConfiguration.cpp
        ./src/CiA402/ModesOfOperation/PPM/PPM.cpp
        ./src/CiA402/ModesOfOperation/PPM/PPMConfiguration.cpp
        ./src/CiA402/ModesOfOperation/CSP/CSP.cpp
        ./src/CiA402/ModesOfOperation/CSP/CSPConfiguration.cpp
        ./src/CiA402/ModesOfOperation/CSV/CSV.cpp
        ./src/CiA402/ModesOfOperation/CSV/CSVConfiguration.cpp
        ./src/CiA402/ModesOfOperation/CST/CST.cpp
        ./src/CiA402/ModesOfOperation/CST/CSTConfiguration.cpp
        ./src/CiA402/ModesOfOperation/VOM/VOM.cpp
        ./src/CiA402/ModesOfOperation/VOM/VOMConfiguration.cpp
        ./src/CiA402/ModesOfOperation/PTM/PTM.cpp
        ./src/CiA402/ModesOfOperation/PTM/PTMConfiguration.cpp
        ./src/CiA402/ModesOfOperation/IPM/IPM.cpp
        ./src/CiA402/ModesOfOperation/IPM/IPMConfiguration.cpp
        ./src/CiA402/CiA402DriverConfiguration.cpp
    )
endif()

if (NOT DISABLE_CANopen)
    set(MODULE_LIBS
        ${MODULE_LIBS}
        ${lely_LIBRARIES}
    )

    set(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${nlohmann_json_INCLUDE_DIRS}
        ${lely_INCLUDE_DIRS}
    )

    set(SRC_FILES
        ${SRC_FILES}
        ./src/CiA402/CANDrivers/CiA402CANDriver/CiA402CANDriver.cpp
        ./src/CiA402/CANDrivers/ERB415CANDriver/ERB415CANDriver.cpp
        ./src/CANopenMaster/CANopenMaster.cpp
    )
endif()

if (NOT DISABLE_CoE)
    set(MODULE_LIBS
        ${MODULE_LIBS}
        EtherCATDrivers
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CoEMaster/CoEMaster.cpp
                  ./src/CiA402/CoEDrivers/CiA402CoEDriver/CiA402CoEDriver.cpp
                  ./src/CiA402/CoEDrivers/ELMOGoldCoEDriver/ELMOGoldCoEDriver.cpp
                  ./src/CiA402/CoEDrivers/ELMOPlatinumCoEDriver/ELMOPlatinumCoEDriver.cpp
                  ./src/CiA402/CoEDrivers/EverestXCRCoEDriver/EverestXCRCoEDriver.cpp
                  ./src/CiA402/CoEDrivers/ZeroErrERobCoEDriver/ZeroErrERobCoEDriver.cpp
    )
endif()

if (NOT DISABLE_CiA402CommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CommunicationUtility
                    Sockets
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CiA402/CiA402CommunicationPoint/CiA402Manager.cpp
                  ./src/CiA402/CiA402CommunicationPoint/CiA402CommunicationPoint.cpp
                  ./src/CiA402/CiA402CommunicationPoint/CiA402CommunicationPointFactory.cpp
    )
endif()

if (DISABLE_CiA402 AND DISABLE_CoE AND DISABLE_CANopen AND DISABLE_CiA402CommunicationPoint)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

find_package(Boost COMPONENTS program_options)

if (NOT DISABLE_CiA402Drivers)
    if (NOT DISABLE_CANopen)
        add_executable(CANopenDriverSample ./samples/CANopenDriver/CANopenDriverSample.cpp)
        target_link_libraries(CANopenDriverSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if (NOT DISABLE_CoE)
        add_executable(ELMOPlatinumCoEDriverSample ./samples/EtherCATDriver/ELMOPlatinumCoEDriverSample.cpp)
        target_link_libraries(ELMOPlatinumCoEDriverSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

        add_executable(ELMOGoldCoEDriverSample ./samples/EtherCATDriver/ELMOGoldCoEDriverSample.cpp)
        target_link_libraries(ELMOGoldCoEDriverSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

        add_executable(EverestXCRCoEDriverSample ./samples/EtherCATDriver/EverestXCRCoEDriverSample.cpp)
        target_link_libraries(EverestXCRCoEDriverSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

        add_executable(ZeroErrERobCoEDriverSample ./samples/EtherCATDriver/ZeroErrERobCoEDriverSample.cpp)
        target_link_libraries(ZeroErrERobCoEDriverSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if(NOT DISABLE_CiA402CommunicationPoint AND ENABLE_UnitTests)
        add_executable(MockCiA402Point ./samples/CiA402CommunicationPoint/MockCiA402Point.cpp)
        target_link_libraries(MockCiA402Point gmock ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})
    endif()

    if (NOT DISABLE_CoE AND NOT DISABLE_CiA402CommunicationPoint)
        add_executable(ELMOGoldCoEDriverPoint ./samples/CiA402CommunicationPoint/ELMOGoldCoEDriverPoint.cpp)
        target_link_libraries(ELMOGoldCoEDriverPoint gmock ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})

        add_executable(EverestXCRCoEDriverPoint ./samples/CiA402CommunicationPoint/EverestXCRCoEDriverPoint.cpp)
        target_link_libraries(EverestXCRCoEDriverPoint gmock ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})

        add_executable(ZeroErrERobCoEDriverPoint ./samples/CiA402CommunicationPoint/ZeroErrERobCoEDriverPoint.cpp)
        target_link_libraries(ZeroErrERobCoEDriverPoint gmock ${Boost_PROGRAM_OPTIONS_LIBRARY} ${MODULE_NAME})
    endif()
endif()
