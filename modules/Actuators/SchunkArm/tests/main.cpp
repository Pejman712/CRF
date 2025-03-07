/*
 * @Author: Zsolt Pasztori
 * @Copyright CERN 2019
 */

#include "gmock/gmock.h"

int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
