set(MODULE_NAME "PeakDetection")

set(INCLUDE_DIRS ./include
)

set(MODULE_LIBS EventLogger
)

set(SRC_FILES ./src/CFAR.cpp
              ./src/GradientPeakDetection.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(CFARSample ./samples/CFARSample.cpp)
target_link_libraries(CFARSample PeakDetection Radars)
