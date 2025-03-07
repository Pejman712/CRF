#!/bin/bash

export GTEST_OUTPUT="xml:test-results/"
export LOGGER_ENABLE_LOGGING=1

cd build
ctest -j$((`nproc`)) --timeout 200
