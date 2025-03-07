#!/bin/bash

export LOGGER_ENABLE_LOGGING=1

cd build
ctest -T memcheck -j8 --timeout 200
