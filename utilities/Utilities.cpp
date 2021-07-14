/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <spdlog/spdlog.h>
#include <stddef.h>
#include <stdint.h>
#include <fstream>
#include "utilities/Utilities.hpp"

uint64_t Utility::packBytes(const uint8_t *const data, const size_t width) {
  assert(width < sizeof(uint64_t));
  uint64_t res = 0;
  for (unsigned int i = 0; i < width; i++) {
    res <<= 8;
    res |= data[i];
  }
  return res;
}

void Utility::unpackBytes(uint8_t *const data, uint64_t val,
                          const size_t width) {
  assert(width < sizeof(uint64_t));
  for (int i = width - 1; i >= 0; i--) {
    data[i] = static_cast<uint8_t>(val);
    val >>= 8;
  }
}

uint32_t Utility::htotl(uint32_t hostVal) {
  uint8_t targetBytes[4];
  if (TARGET_WORD_SIZE == 2) {
    return htots(hostVal);
  }
#ifdef TARGET_BIG_ENDIAN
  targetBytes[0] = hostVal;
  targetBytes[1] = hostVal / 256;
  targetBytes[2] = hostVal / 256 / 256;
  targetBytes[3] = hostVal / 256 / 256 / 256;
#elif defined TARGET_LITTLE_ENDIAN
  targetBytes[0] = hostVal / 256 / 256 / 256;
  targetBytes[1] = hostVal / 256 / 256;
  targetBytes[2] = hostVal / 256;
  targetBytes[3] = hostVal;
#else
#error Must specify TARGET_BIG_ENDIAN or TARGET_LITTLE_ENDIAN
#endif
  return *((uint32_t *)targetBytes);
}

uint32_t Utility::ttohl(uint32_t targetVal) {
  uint8_t *targetBytes = (uint8_t *)(&targetVal);
  uint32_t hostVal;
  if (TARGET_WORD_SIZE == 2) {
    return ttohs(targetVal);
  }

#ifdef TARGET_BIG_ENDIAN
  hostVal = targetBytes[3];
  hostVal = hostVal * 256 + targetBytes[2];
  hostVal = hostVal * 256 + targetBytes[1];
  hostVal = hostVal * 256 + targetBytes[0];
#elif defined TARGET_LITTLE_ENDIAN
  hostVal = targetBytes[0];
  hostVal = hostVal * 256 + targetBytes[1];
  hostVal = hostVal * 256 + targetBytes[2];
  hostVal = hostVal * 256 + targetBytes[3];
#else
#error Must specify TARGET_BIG_ENDIAN or TARGET_LITTLE_ENDIAN
#endif

  return hostVal;
}

uint32_t Utility::htots(uint32_t hostVal) {
  uint8_t targetBytes[4];

#ifdef TARGET_BIG_ENDIAN
  targetBytes[0] = hostVal;
  targetBytes[1] = hostVal / 256;
#elif defined TARGET_LITTLE_ENDIAN
  targetBytes[0] = hostVal / 256;
  targetBytes[1] = hostVal;
#else
#error Must specify TARGET_BIG_ENDIAN or TARGET_LITTLE_ENDIAN
#endif

  return *((uint32_t *)targetBytes);
}

uint32_t Utility::ttohs(uint32_t targetVal) {
  uint8_t *targetBytes = (uint8_t *)(&targetVal);
  uint32_t hostVal;

#ifdef TARGET_BIG_ENDIAN
  hostVal = targetBytes[1];
  hostVal = hostVal * 256 + targetBytes[0];
#elif defined TARGET_LITTLE_ENDIAN
  hostVal = targetBytes[0];
  hostVal = hostVal * 256 + targetBytes[1];
#else
#error Must specify TARGET_BIG_ENDIAN or TARGET_LITTLE_ENDIAN
#endif

  return hostVal;
}

uint32_t Utility::setBit(size_t bitIdx, uint32_t word, bool value) {
  assert(bitIdx < TARGET_WORD_SIZE * 8);
  word &= ~(1u << bitIdx);  // Clear
  return word | (static_cast<uint32_t>(value) << bitIdx);
}

bool Utility::assertFileExists(const std::string &filename) {
  std::ifstream ifile(filename.c_str());
  if (!(bool)ifile) {
    spdlog::error("File {} does not exist.", filename);
    exit(1);
  }
  return (bool)ifile;
}
