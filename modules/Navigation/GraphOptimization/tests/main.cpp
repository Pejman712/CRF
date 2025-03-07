/*
 * @Author: Carlos Prados
 * @Copyright CERN 2020
 */

#include "gmock/gmock.h"

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
