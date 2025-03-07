#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi and CERN EN/SMM/MRO 2019                                                                   ##
##         Krzysztof Szczurek CERN EN/SMM/MRO 2019                                                                   ##
## Contributor: Alejadro Diaz Rosales CERN EN/SMM/MRO 2020                                                           ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "SiemensPLC")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
                Snap7
                ${Boost_SYSTEM_LIBRARY}
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/SiemensPLCS7.cpp
              ./src/SiemensPLCTypeConverter.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(SiemensS300Sample ./samples/SiemensS300Sample.cpp)
target_link_libraries(SiemensS300Sample ${MODULE_NAME})