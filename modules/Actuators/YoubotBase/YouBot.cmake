find_package(orocos_kdl REQUIRED)

FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)

FIND_PATH(OODL_YOUBOT_INCLUDE_DIR NAMES youbot/YouBotBase.hpp
  PATHS
  /usr/local/include/youbot/youbot/
)

FIND_LIBRARY(OODL_YOUBOT_LIBRARIES NAMES "YouBotDriver" "soem"
  /usr/local/lib
)

SET(OODL_YOUBOT_INCLUDE_DIR ${OODL_YOUBOT_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR}/soem/src/ /usr/local/include/youbot/youbot/src)

SET(OODL_YOUBOT_LIBRARIES YouBotDriver)
SET(OODL_YOUBOT_CONFIG_DIR ${youbot_driver_PACKAGE_PATH}/config/)

include_directories (
    ${OODL_YOUBOT_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    include/robot/youbot/
    src/robot/youbot
    include
    src   
    /usr/local/include/eigen3
)


set(YOUBOT_LIBRARIES
    ${Boost_LIBRARIES}
    ${OODL_YOUBOT_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
)
