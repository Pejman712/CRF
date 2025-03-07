set(MODULE_NAME "YoubotArm")
project (${MODULE_NAME})
link_directories(/usr/local/lib)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)
find_package(Eigen3 REQUIRED)

FIND_PATH(OODL_YOUBOT_INCLUDE_DIR NAMES youbot/YouBotBase.hpp
  PATHS
  /usr/local/include/youbot/youbot/
)

FIND_LIBRARY(OODL_YOUBOT_LIBRARIES NAMES "YouBotDriver" "soem"
  /usr/local/lib
)

find_library(SOEM soem)

SET(OODL_YOUBOT_INCLUDE_DIRS ${OODL_YOUBOT_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR}/soem/src/ /usr/local/include/youbot/youbot/src)

set(INCLUDE_DIRS
    ./include
    ${Boost_INCLUDE_DIR}
    ${OODL_YOUBOT_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

set(MODULE_LIBS ${Boost_LIBRARIES} ${OODL_YOUBOT_LIBRARIES} RobotArm utility)
set(SRC
    ./src/InverseKinematic.cpp
    ./src/YoubotArm.cpp
    ./src/YoubotArmConfiguration.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
