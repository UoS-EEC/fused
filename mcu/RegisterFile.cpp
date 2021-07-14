/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcu/RegisterFile.hpp"
#include "utilities/Utilities.hpp"
#include <iostream>
#include <spdlog/spdlog.h>

bool RegisterFile::testBit(const size_t addr, const size_t n) const {
  assert(n < TARGET_WORD_SIZE * 8);
  return ((read(addr) & (1u << n)) > 0);
}

bool RegisterFile::testBitMask(const size_t address,
                               const uint32_t mask) const {
  return ((read(address) & mask) > 0);
}

void RegisterFile::setBit(const size_t address, const unsigned bit,
                          const bool force) {
  assert(bit < TARGET_WORD_SIZE * 8);
  auto &r = find(address);
  assert(r.access == AccessMode::WRITE || r.access == AccessMode::READ_WRITE ||
         force);
  write(address, read(address) | (1u << bit), force);
}

void RegisterFile::setBitMask(const size_t address, const uint32_t mask,
                              const bool force) {
  write(address, read(address) | mask, force);
}

void RegisterFile::clearBitMask(const size_t address, const uint32_t mask,
                                const bool force) {
  write(address, read(address) & (~mask), force);
}

void RegisterFile::write(const size_t address, const uint32_t value,
                         const bool force) {
  auto rit = std::find_if(
      m_regs.begin(), m_regs.end(),
      [address](RegisterFile::Register &r) { return r.addr == address; });
  if (rit == m_regs.end()) {
    spdlog::error("RegisterFile::write Address 0x{:08x} not found.", address);
    exit(1);
  }
  auto &r = *rit;

  assert(r.access == AccessMode::WRITE || r.access == AccessMode::READ_WRITE ||
         force);
  if (!force) {
    r.val = (r.val & (~r.writeMask)) | (value & r.writeMask);
  } else {
    r.val = value;
  }
}

void RegisterFile::write(size_t address, uint8_t *buf, size_t len,
                         const bool force) {
  if (len == 0) {
    spdlog::warn("RegisterFile::write 0-length write!");
  }
  while (len > 0) {
    if ((len >= 4) && ((address % 4) == 0) && (TARGET_WORD_SIZE == 4)) {
      // Aligned 32-bit access

      // Convert from target to host endianness
      uint32_t tmp = Utility::packBytes(buf, 4);
      tmp = Utility::ttohl(tmp);

      write(address, tmp, force);
      len -= 4;
      buf += 4;
      address += 4;
    } else if ((len >= 2) && ((address % 2) == 0)) {
      // Aligned 16-bit access
      // Convert from target to host endianness
      uint32_t tmp = Utility::packBytes(buf, 2);
      tmp = Utility::ttohs(tmp);

      write(address, tmp, force);
      len -= 2;
      buf += 2;
      address += 2;
    } else { // Unaligned
      writeByte(address, *buf);
      len -= 1;
      buf += 1;
      address += 1;
    }
  }
}

uint32_t RegisterFile::read(const size_t address) const {
  const auto &r = find(address);
  assert(r.access == AccessMode::READ || r.access == AccessMode::READ_WRITE);
  return r.val;
}

void RegisterFile::read(size_t address, uint8_t *buf, size_t len) const {
  while (len > 0) {
    if ((len >= 4) && ((address % 4) == 0) && (TARGET_WORD_SIZE == 4)) {
      // Aligned 32-bit access
      // Convert from host to target endianness
      uint32_t tmp = Utility::htotl(read(address));
      Utility::unpackBytes(buf, tmp, 4);
      len -= 4;
      buf += 4;
      address += 4;
    } else if ((len >= 2) && ((address % 2) == 0)) {
      // Aligned 16-bit access
      // Convert from host to target endianness
      uint32_t tmp = Utility::htots(read(address));
      Utility::unpackBytes(buf, tmp, 2);
      len -= 2;
      buf += 2;
      address += 2;
    } else { // Unaligned
      *buf = readByte(address);
      len -= 1;
      buf += 1;
      address += 1;
    }
  }
}

void RegisterFile::writeByte(const size_t address, const uint8_t value,
                             const bool force) {
  size_t ofs = address % TARGET_WORD_SIZE;
  size_t addrAligned = address - ofs;
  uint32_t tmp = read(addrAligned);
  reinterpret_cast<uint8_t *>(&tmp)[ofs] = value;
  write(addrAligned, tmp, force);
}

uint8_t RegisterFile::readByte(const size_t address) const {
  size_t ofs = address % TARGET_WORD_SIZE;
  auto addrAligned = address - ofs;
  const auto &r = find(addrAligned);
  assert(r.access == AccessMode::READ || r.access == AccessMode::READ_WRITE);
  return reinterpret_cast<const uint8_t *>(&r.val)[ofs];
}

void RegisterFile::increment(const size_t address, const bool force) {
  const auto &r = find(address);
  assert(r.access == AccessMode::READ_WRITE || force);
  write(address, r.val + 1, force);
}

void RegisterFile::clearBit(const size_t address, const unsigned bit,
                            const bool force) {
  assert(bit < TARGET_WORD_SIZE * 8);
  const auto &r = find(address);
  assert(r.access == AccessMode::WRITE || r.access == AccessMode::READ_WRITE ||
         force);
  write(address, r.val & (~(1u << bit)), force);
}

const RegisterFile::Register &RegisterFile::find(const size_t address) const {
  auto rit = std::find_if(
      m_regs.begin(), m_regs.end(),
      [address](const RegisterFile::Register &r) { return r.addr == address; });
  if (rit == m_regs.end()) {
    spdlog::error("RegisterFile::find Address 0x{:08x} not found.", address);
    exit(1); // to suppress warning
  }
  return *rit;
}

bool RegisterFile::contains(unsigned address) const {
  auto rit = std::find_if(
      m_regs.begin(), m_regs.end(),
      [address](const RegisterFile::Register &r) { return r.addr == address; });
  return (rit != m_regs.end());
}

std::ostream &operator<<(std::ostream &os, const RegisterFile &rhs) {
  for (const auto &r : rhs.m_regs) {
    os << "@0x" << std::hex << r.addr << ": 0x" << r.val << std::dec;
  }
  return os;
}
