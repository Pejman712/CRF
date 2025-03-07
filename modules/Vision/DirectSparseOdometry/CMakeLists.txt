set(MODULE_NAME "DirectSparseOdometry")

find_package(Eigen3 REQUIRED) # header only library
find_package(Boost COMPONENTS system)
find_package(Pangolin 0.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED) # header only library
find_package(nlohmann_json REQUIRED)

set(MODULE_LIBS boost_system ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

set(DirectSparseOdometry_SRC
    src/FullSystem/CoarseInitializer.cpp
    src/FullSystem/CoarseTracker.cpp
    src/FullSystem/FullSystem.cpp
    src/FullSystem/FullSystemMarginalize.cpp
    src/FullSystem/FullSystemOptimize.cpp
    src/FullSystem/FullSystemOptPoint.cpp
    src/FullSystem/HessianBlocks.cpp
    src/FullSystem/ImmaturePoint.cpp
    src/FullSystem/PixelSelector2.cpp
    src/FullSystem/Residuals.cpp
    src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
    src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
    src/IOWrapper/OutputWrapper/CERNOutputWrapper.cpp
    src/IOWrapper/OutputWrapper/KeyFrameDisplay.cpp
    src/IOWrapper/OutputWrapper/PangolinDSOViewer.cpp
    src/OptimizationBackend/AccumulatedSCHessian.cpp
    src/OptimizationBackend/AccumulatedTopHessian.cpp
    src/OptimizationBackend/EnergyFunctional.cpp
    src/OptimizationBackend/EnergyFunctionalStructs.cpp
    src/util/globalCalib.cpp
    src/util/settings.cpp
    src/util/Undistort.cpp
)

add_library(${MODULE_NAME} SHARED ${DirectSparseOdometry_SRC})

target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${CHOLMOD_INCLUDE_DIR})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${Pangolin_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${Sophus_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${nlohmann_json_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} PUBLIC include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
