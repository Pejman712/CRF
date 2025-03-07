set (MODULE_NAME "RGBDVisionUtility")

find_package(Protobuf)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set (MODULE_LIBS EventLogger
                 ${PCL_LIBRARIES}
                 ${OpenCV_LIBRARIES}
                 ${EIGEN3_LIBRARIES}
)
set(VISUTIL_SRCS ./src/PCLUtils.cpp
)

add_library(${MODULE_NAME} SHARED ${VISUTIL_SRCS})
target_include_directories(${MODULE_NAME} PUBLIC  ./include)
target_include_directories(${MODULE_NAME} PUBLIC SYSTEM ${INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} PUBLIC SYSTEM ${PCL_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} PUBLIC SYSTEM ${OpenCV_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} PUBLIC SYSTEM ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
