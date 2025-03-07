set(MODULE_NAME "InsulateFrameWorks")
find_package(OpenCV REQUIRED )

set(INCLUDE_DIRS
    ./include
    ${OpenCV_INCLUDE_DIRS}
)
set(MODULE_LIBS ${OpenCV_LIBS})
set(Insulate_SRC ./src/InsulateFrameWorks.cpp)

add_library(${MODULE_NAME} SHARED ${Insulate_SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
