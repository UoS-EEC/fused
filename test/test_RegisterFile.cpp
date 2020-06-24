/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <string>
#include "mcu/RegisterFile.hpp"

int main() {
  RegisterFile dut;
  unsigned crntAddr = 0;

  // TEST - Add a default register
  assert(dut.size() == 0);
  dut.addRegister(crntAddr, 0);
  assert(dut.size() == 1);
  assert(dut.contains(crntAddr));

  // TEST - Word access
  unsigned testval = 0xAAAA5555;
  dut.write(crntAddr, testval);
  assert(dut.read(crntAddr) == testval);

  // TEST - Reset
  dut.reset();
  assert(dut.read(crntAddr) == 0);

  // TEST - Byte access
  for (int ofs = 0; ofs < TARGET_WORD_SIZE; ofs++) {
    dut.write(crntAddr, 0xFFFFFFFF);
    dut.writeByte(ofs, 0xAA);
    assert(dut.readByte(crntAddr + ofs) == 0xAA);  // Byte read

    auto correct = (0xFFFFFFFF & ~(0xFF << (ofs * 8))) | (0xAA << (ofs * 8));
    assert(dut.read(crntAddr) == correct);  // Word read
  }

  // Test - Write mask
  crntAddr += TARGET_WORD_SIZE;
  dut.addRegister(/*address=*/crntAddr, /*value=*/0,
                  /*access_mode=*/RegisterFile::AccessMode::READ_WRITE,
                  /*writeMask=*/0xFFFF0000);
  assert(dut.size() == 2);
  dut.write(crntAddr, 0x55555555);
  assert(dut.read(crntAddr) == 0x55550000);

  return 0;
}
