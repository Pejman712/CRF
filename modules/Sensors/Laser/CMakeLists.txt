
set(MODULE_NAME "Laser")

if(DISABLE_${MODULE_NAME})
    return()
endif()

find_package(small_gicp)

set(MODULE_LIBS EventLogger
                CommonInterfaces
                VisionUtility
                ${PCL_LIBRARIES}
                ${Boost_LIBRARIES}
                Eigen3::Eigen

)
set(INCLUDE_DIRS ./include
                 ${nlohmann_json_INCLUDE_DIRS}
                 ${PCL_INCLUDE_DIRS}
                 ${Boost_INCLUDE_DIRS}
                 ${EIGEN3_INCLUDE_DIRS}
)
set(SRC_FILES ./src/LaserConfiguration.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error")


if(NOT DISABLE_HokuyoLaser)
    set(MODULE_LIBS ${MODULE_LIBS}
                    VisionUtility
                    ${URG_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${URG_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/HokuyoLaser/HokuyoLaser.cpp
    )
endif()

if(NOT DISABLE_RPLiDAR)
    find_package (LibUSB)
    set(MODULE_LIBS ${MODULE_LIBS}
                    VisionUtility
                    ${RPLiDARSDK_LIBRARIES}
                    ${LibUSB_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${RPLiDARSDK_INCLUDE_DIRS}
                     ${LibUSB_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RPLiDAR/RPLiDAR.cpp
    )
endif()



if(NOT DISABLE_UnitreeL1Viewer)
    set(MODULE_LIBS ${MODULE_LIBS}
                    VisionUtility
                    ${UnitreeL1SDK_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${UnitreeL1SDK_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/UnitreeL1/UnitreeL1.cpp    
    )
endif()


if(NOT DISABLE_Unitreecloudimu)
    set(MODULE_LIBS ${MODULE_LIBS}
                    VisionUtility
                    ${UnitreeL1SDK_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${UnitreeL1SDK_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/UnitreeL1/UnitreeL1.cpp    
    )
endif()


if(NOT DISABLE_Gicp_mapping)
    set(MODULE_LIBS ${MODULE_LIBS}
                    VisionUtility
                    ${UnitreeL1SDK_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${UnitreeL1SDK_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/UnitreeL1/UnitreeL1.cpp    
    )
endif()


add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()





# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_HokuyoLaser)
    add_executable(HokuyoLaserViewer ./samples/HokuyoLaserViewer.cpp)
    target_link_libraries(HokuyoLaserViewer LiDAR)
endif()

if(NOT DISABLE_RPLiDAR)
    add_executable(RPLiDARViewer ./samples/RPLiDARViewer.cpp)
    target_link_libraries(RPLiDARViewer ${MODULE_NAME} ${Boost_LIBRARIES})
endif()   

# Executable for UnitreeL1Viewer
if(NOT DISABLE_UnitreeL1Viewer) # Correct the condition here
    add_executable(UnitreeL1Viewer ./samples/UnitreeL1Viewer.cpp)
    target_link_libraries(UnitreeL1Viewer ${MODULE_NAME} ${Boost_LIBRARIES} IMU)
endif()

if(NOT DISABLE_UnitreeL1Viewer) # Correct the condition here
    add_executable(UnitreeViewerIntensity ./samples/UnitreeViewerIntensity.cpp)
    target_link_libraries(UnitreeViewerIntensity ${MODULE_NAME} 
    ${Boost_LIBRARIES}
    Laser 
    IMU)
endif()

if(NOT DISABLE_Unitreecloudimu) # Correct the condition here
    add_executable(Unitreecloudimu ./samples/Unitreecloudimu.cpp)
    target_link_libraries(Unitreecloudimu ${MODULE_NAME} ${Boost_LIBRARIES} IMU)
endif()


if(NOT DISABLE_Gicp_mapping) # Correct the condition here
    add_executable(Gicp_mapping ./samples/Gicp_mapping.cpp)
    target_link_libraries(Gicp_mapping ${MODULE_NAME} ${Boost_LIBRARIES})
endif()

#    if(NOT DISABLE_TIM)
#        add_executable(RPLiDARandTIMCapturer ./samples/RPLiDARandTIMCapturer.cpp)
#        target_link_libraries(RPLiDARandTIMCapturer LiDAR TIM ${Boost_PROGRAM_OPTIONS_LIBRARY})
#    endif()


