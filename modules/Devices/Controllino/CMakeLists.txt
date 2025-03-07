# Give the appropriate name to your module. Obviously, it cannot be "Template".
# It also cannot be "MyModule3".
set(MODULE_NAME "Controllino")

# Specify all components your project should be linked with
set(MODULE_LIBS
    EventLogger
    IPC
    modbus
    CommonInterfaces
)

# Define your library, write all source file names by hand
add_library(${MODULE_NAME} SHARED
    src/Controllino.cpp
)

# Specify where do you put the header files for your project
target_include_directories(${MODULE_NAME} PUBLIC include)

# Link your module
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(controllino ./samples/controllino.cpp)
target_link_libraries(controllino Controllino)
