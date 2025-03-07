set (MODULE_NAME "DsoDevel")
set (MODULE_LIBS DirectSparseOdometry EventLogger opencv_core Types)

add_library(${MODULE_NAME} SHARED src/DsoDevel.cpp src/PositionObserver.cpp)

target_include_directories(${MODULE_NAME} PUBLIC include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
