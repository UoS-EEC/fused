/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <iostream>
#include <vector>

/**
 * @brief The RegisterFile class Convenience class to implement register files
 * for peripherals as signals with access type attribute and
 * byte-addresses.
 *
 * !***IMPORTANT NOTE***!:
 * Data is stored in host endianness in the registers, so bytes are potentially
 * flipped in the b_transport method. Take care to use htotl/htots if writing to
 * or reading from the bus
 *
 */
class RegisterFile {
 public:
  /* ------ Types ------ */
  //! Register type, can allow reads, writes or both reads and writes.
  enum access_mode { READ, WRITE, READ_WRITE };

  //! One register
  struct preg {
    uint32_t val;                   //! Value held in register
    const size_t addr;              //! Address of register
    const uint32_t write_mask;      //! Masks off undef bits
    const enum access_mode access;  //! Allowed access mode
  };

  /* ------ Public methods ------ */
  /**
   * @brief RegisterFile Constructor: Initialise empty register file
   */
  RegisterFile() : m_regs() {}

  /**
   * @brief addRegister Add register to register file.
   * @param address register address
   * @param value initial value
   * @param access access mode
   * @param write_mask write mask. Used to mask undefined bits when writing.
   */
  void addRegister(const size_t address, const uint32_t value = 0,
                   const access_mode access = READ_WRITE,
                   const uint32_t writeMask = 0xffffffff) {
    m_regs.push_back({value, address, writeMask, access});
  }

  /**
   * @brief write Write value to register based on address. Checks for access
   * type
   * @param address register address (/id) to write to
   * @param value value to write to register
   */
  void write(const size_t address, const uint32_t value,
             const bool force = false);

  /**
   * @brief RegisterFile::write Write buffer to register file
   * @param address
   * @param buf
   * @param len
   */
  void write(size_t address, uint8_t *buf, size_t len,
             const bool force = false);

  /**
   * @brief RegisterFile::writeByte write a single byte to a register.
   * @param address
   * @param value
   * @return
   */
  void writeByte(const size_t address, const uint8_t value,
                 const bool force = false);

  /**
   * @brief read Read value from register based on address. Checks for access
   * type.
   *const  @param address register address (/id) to read from
   * @return Value of register.
   */
  uint32_t read(const size_t address) const;

  /**
   * @brief RegisterFile::readByte read a single byte from a register.
   * @param address
   * @param value
   * @return
   */
  uint8_t readByte(const size_t address) const;

  /**
   * @brief RegisterFile::read Read arbitrary length of data
   * @param address
   * @param len number of bytes to be read
   * @return
   */
  void read(size_t address, uint8_t *buf, size_t len) const;

  /**
   * @brief setBit Set a single bit in a register
   * @param address register address
   * @param bit bit number to set
   */
  void setBit(const size_t address, const unsigned bit,
              const bool force = false);

  /**
   * @brief setBitMask Set bits according to mask
   * @param address register address
   * @param mask
   */
  void setBitMask(const size_t address, const uint32_t mask,
                  const bool force = false);

  /**
   * @brief clearBitMask Clear bits according to mask
   * @param address register address
   * @param mask
   */
  void clearBitMask(const size_t address, const uint32_t mask,
                    const bool force = false);

  /**
   * @brief clearBit Clear a single bit in a register
   * @param address register address
   * @param bit bit number to clear
   */
  void clearBit(const size_t address, const unsigned bit,
                const bool force = false);

  /**
   * @brief testBit Return true if bit n is 1 in the chosen register.
   * @param address register address
   * @param bit bit number to test
   * @return true if bit n is set in register value
   */
  bool testBit(const size_t addr, const size_t n) const;

  /**
   * @brief testBitMask test bit mask
   * @param addr register address
   * @param mask bitmask to test
   * @return  true if any same bit is set in both mask and register value
   */
  bool testBitMask(const size_t addr, const uint32_t mask) const;

  /**
   * @brief increment Increment register value. Does not guard against
   * overflow
   * @param address
   */
  void increment(size_t address, bool force = false);

  /**
   * @brief size Return number of registers
   * @return number of registers
   */
  unsigned int size() const { return m_regs.size(); }

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const RegisterFile &rhs);

 private:
  /**
   * @brief find find a register by address
   * @param address
   * @return register
   */
  const preg &find(const size_t address) const;

  /* ------ Private variables ------ */
  std::vector<preg> m_regs;  //! Registers
};
