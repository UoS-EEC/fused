/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

namespace Utility {

/**
 * @brief packBytes pack an array of bytes into a single (right-aligned)
 * variable
 * packs assuming reading convention (MSB at data[0])
 * @param data pointer to array of source bytes
 * @param width number of bytes to pack
 */
uint64_t packBytes(uint8_t *const data, const size_t width);

/**
 * @brief unpackBytes unpack a (right-aligned) variable into an array of bytes
 * unpacks according to reading convention (MSB at data[0])
 * @param data pointer to array of source bytes
 * @param width number of bytes to pack
 */
void unpackBytes(uint8_t *const data, uint64_t val, const size_t width);

/**
 * @brief htotl Convert 32-bit value from host to target endianness
 * @param hostVal The value in host endianness
 * @return The value in target endianness
 */
uint32_t htotl(uint32_t hostVal);

/**
 * @brief ttohl Convert 32-bit value from target to host endianness
 * @param targetVal The value in target endianness
 * @return The value in host endianness
 */
uint32_t ttohl(uint32_t targetVal);

/**
 * @brief htots Convert 16-bit value from host to target endianness
 * @param hostVal The value in host endianness
 * @return The value in target endianness
 */
uint32_t htots(uint32_t hostVal);

/**
 * @brief ttohs Convert 16-bit value from target to host endianness
 * @param targetVal The value in target endianness
 * @return The value in host endianness
 */
uint32_t ttohs(uint32_t targetVal);

/**
 * @brief setBit set the value of a single bit at index bitIdx in a word.
 * @param bitIdx bit index
 * @param word target word
 * @param value value of bit to be set
 */
uint32_t setBit(size_t bitIdx, uint32_t word, bool value);

/**
 * @brief assertFileExists Assert that file exists, exit with error otherwise.
 * @param filename
 */
bool assertFileExists(const std::string &filename);

}  // namespace Utility
