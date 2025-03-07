#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi and CERN EN/SMM/MRO 2019                                                                   ##
##         Krzysztof Szczurek CERN EN/SMM/MRO 2019                                                                   ##
## Contributor: Alejadro Diaz Rosales CERN EN/SMM/MRO 2020                                                           ##
##                                                                                                                   ##
#######################################################################################################################

set(LIBRARY_NAME "Snap7")

if(DISABLE_${LIBRARY_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
                 ./src/core
                 ./src/lib
                 ./src/sys
)
set(SRC_FILES ./src/core/s7_client.cpp
              ./src/core/s7_isotcp.cpp
              ./src/core/s7_micro_client.cpp
              ./src/core/s7_partner.cpp
              ./src/core/s7_peer.cpp
              ./src/core/s7_server.cpp
              ./src/core/s7_text.cpp
              ./src/libmain/snap7_libmain.cpp
              ./src/sys/snap_msgsock.cpp
              ./src/sys/snap_sysutils.cpp
              ./src/sys/snap_tcpsrvr.cpp
              ./src/sys/snap_threads.cpp
              ./src/s7.cpp
              ./src/snap7.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRC_FILES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIRS})

# TODO(adiazros): There is an error in line related to this flag that started appearing with GCC9,
#                 since is not our code I disabled it.
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11)
        target_compile_options(${LIBRARY_NAME} PRIVATE -Wno-error=stringop-overflow)
    else()
	target_compile_options(${LIBRARY_NAME} PRIVATE -Wno-error=stringop-overread)
    endif()
endif()
