# Compilation & Execution {#compilation_and_execution}

Clone the repository and go inside

````bash
git clone https://gitlab.cern.ch/mro/robotics/CERNRoboticFramework/cpproboticframework.git
cd cpproboticframework
````

Create some directory for build artifacts, for example:

````bash
mkdir build
cd build
````

To build all available targets, execute:

````bash
cmake ../ -DCMAKE_BUILD_TYPE=Debug
make
````

The output binaries are available under "bin" directory. For help execute:

````bash
make help
````

To run the tests execute:

````bash
make test             # Launch all the tests
make test_coverage    # Launch all the tests and checks the code coverage
````

To run the test make sure you enabled them in the root CMakeLists.txt. The Logger tests will fail if Logger is disabled, to enable it only set:

````bash
export LOGGER_ENABLE_LOGGING=1
export LOGGER_PRINT_STDOUT=1
export LOGGER_ENABLE_DEBUG=1
````
