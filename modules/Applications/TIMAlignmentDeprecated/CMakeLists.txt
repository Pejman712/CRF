set(MODULE_NAME "TIMAlignmentDepreacted")

set(TIMAlignment_SRCS ./src/TIMAlignment.cpp)

set(MODULE_LIBS EventLogger
                ObjectDetection
                VisionTypes
                CommonInterfaces
                NetworkClient
                boost_system
)

add_library(${MODULE_NAME} SHARED ${TIMAlignment_SRCS})

target_include_directories(${MODULE_NAME} PUBLIC ./include)
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(tim_alignment ./samples/tim_alignment.cpp)
target_link_libraries(tim_alignment TIMAlignment)
