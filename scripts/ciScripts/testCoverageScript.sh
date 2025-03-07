#!/bin/bash

export LOGGER_ENABLE_LOGGING=1

cd build
make test_coverage
