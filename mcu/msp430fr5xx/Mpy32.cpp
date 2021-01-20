/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string>
#include <systemc>
#include "mcu/msp430fr5xx/Mpy32.hpp"
#include "utilities/Config.hpp"

extern "C" {
#include "device_includes/msp430fr5994.h"
}

using namespace sc_core;

Mpy32::Mpy32(sc_module_name name, const uint16_t startAddress,
             const uint16_t endAddress)
    : BusTarget(name, startAddress, endAddress) {
  // Register events

  // Initialise register file
  uint16_t endOffset = endAddress - startAddress + 1;
  for (uint16_t i = 0; i < endOffset; i += 2) {
    m_regs.addRegister(i, 0, RegisterFile::AccessMode::READ_WRITE);
  }

  SC_METHOD(reset);
  sensitive << pwrOn;
  dont_initialize();

  SC_METHOD(process);
  sensitive << m_mpyCompleteEvent;
}

void Mpy32::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  auto addr = trans.get_address();
  // Set the mode bits in the control register.
  // Process (multiplication) starts only after OP2 written.
  uint16_t ctrl = m_regs.read(OFS_MPY32CTL0);
  // "Not Implemented Case"
  if (ctrl & MPYFRAC) {
    SC_REPORT_FATAL(this->name(), "Fractional mode not implemented.");
    return;
  }

  uint8_t n = 0;
  switch (addr) {
    case OFS_MPY:  // "MPY16\n";
      m_regs.write(OFS_MPY32CTL0,
                   ((ctrl & ~(MPYM__MACS)) | MPYM__MPY) & ~(MPYOP1_32));
      break;
    case OFS_MPYS:  // "MPYS16\n";
      m_regs.write(OFS_MPY32CTL0,
                   ((ctrl & ~(MPYM__MACS)) | MPYM__MPYS) & ~(MPYOP1_32));
      break;
    case OFS_MAC:  // "MAC16\n";
      m_regs.write(OFS_MPY32CTL0,
                   ((ctrl & ~(MPYM__MACS)) | MPYM__MAC) & ~(MPYOP1_32));
      break;
    case OFS_MACS:  // "MACS16\n";
      m_regs.write(OFS_MPY32CTL0,
                   ((ctrl & ~(MPYM__MACS)) | MPYM__MACS) & ~(MPYOP1_32));
      break;
    case OFS_MPY32L:  // "MPY32\n";
      m_regs.write(OFS_MPY32CTL0,
                   (ctrl & ~(MPYM__MACS)) | MPYM__MPY | MPYOP1_32);
      break;
    case OFS_MPYS32L:  // "MPYS32\n";
      m_regs.write(OFS_MPY32CTL0,
                   (ctrl & ~(MPYM__MACS)) | MPYM__MPYS | MPYOP1_32);
      break;
    case OFS_MAC32L:  // "MAC32\n";
      m_regs.write(OFS_MPY32CTL0,
                   (ctrl & ~(MPYM__MACS)) | MPYM__MAC | MPYOP1_32);
      break;
    case OFS_MACS32L:  // "MACS32\n";
      m_regs.write(OFS_MPY32CTL0,
                   (ctrl & ~(MPYM__MACS)) | MPYM__MACS | MPYOP1_32);
      break;
    case OFS_OP2:  // "16-bit OP2; MPY16 Starts\n";
      m_regs.write(OFS_MPY32CTL0, ctrl & ~(MPYOP2_32));
      // n = ctrl & MPYOP1_32 ? 7: 4;
      n = 0;
      m_mpyCompleteEvent.notify(delay + n * systemClk->getPeriod());
      break;
    case OFS_OP2L:  // "32-bit OP2 Low Word; Expects High Write; MPY32
                    // Starts\n";
      m_regs.write(OFS_MPY32CTL0, ctrl | MPYOP2_32);
      break;
    case OFS_OP2H:  // "32-bit OP2 High Word; Update Multiplication Results\n";
                    // n = ctrl & MPYOP1_32 ? 11: 7;
      n = 0;
      m_mpyCompleteEvent.notify(delay + n * systemClk->getPeriod());
      break;
    default:  // the other register writes do not cause additional state change;
              // TODO: catch not-implemented e.g. fractional
      break;
  }
  BusTarget::b_transport(trans, delay);
}

void Mpy32::reset(void) { m_regs.reset(); }

void Mpy32::process(void) {
  if (pwrOn.read()) {
    // "Multiplication Starts\n";
    uint16_t ctrl = m_regs.read(OFS_MPY32CTL0);

    // hex << ctrl << "\n";
    if (((ctrl & MPYOP1_32) != 0) ||
        ((ctrl & MPYOP2_32) != 0)) {  // 16x32, 32x16, 32x32, 16x24 ...
      // "32x32\n";
      if ((ctrl & MPYM__MAC) != 0) {  // MAC or MACS
        // "Accumulate\n";
        // Record Previous Results
        uint128_t prev_resC = m_regs.read(OFS_SUMEXT);
        uint128_t prev_res3 = m_regs.read(OFS_RES3);
        uint128_t prev_res2 = m_regs.read(OFS_RES2);
        uint128_t prev_res1 = m_regs.read(OFS_RES1);
        uint128_t prev_res0 = m_regs.read(OFS_RES0);

        uint128_t prev_res = (prev_resC << 64) | (prev_res3 << 48) |
                             (prev_res2 << 32) | (prev_res1 << 16) |
                             (prev_res0 << 0);

        if ((ctrl & MPYM__MPYS) != 0) {  // Signed
          // "Signed\n";
          uint16_t op1_L = m_regs.read(OFS_MACS32L);
          uint16_t op1_H = m_regs.read(OFS_MACS32H);
          uint16_t op2_L = m_regs.read(OFS_OP2L);
          uint16_t op2_H = m_regs.read(OFS_OP2H);

          uint128_t op1 = (uint32_t)op1_H << 16 | (uint32_t)op1_L;
          uint128_t op2 = (uint32_t)op2_H << 16 | (uint32_t)op2_L;

          op1 = op1 & 0x80000000 ? op1 | 0xffffffff00000000 : op1;
          op2 = op2 & 0x80000000 ? op2 | 0xffffffff00000000 : op2;

          uint128_t res = (uint128_t)((uint64_t)(op1 * op2)) + prev_res;

          uint16_t resC = (uint16_t)((res >> 64) & 0x000000000000ffff);
          uint16_t res3 = (uint16_t)((res >> 48) & 0x000000000000ffff);
          uint16_t res2 = (uint16_t)((res >> 32) & 0x000000000000ffff);
          uint16_t res1 = (uint16_t)((res >> 16) & 0x000000000000ffff);
          uint16_t res0 = (uint16_t)((res >> 0) & 0x000000000000ffff);

          // Write results
          m_regs.write(OFS_RES3, res3);
          m_regs.write(OFS_RES2, res2);
          m_regs.write(OFS_RES1, res1);
          m_regs.write(OFS_RES0, res0);

          // sign
          if ((res3 & 0x8000) != 0) {  // MSB determines the sign
            m_regs.write(OFS_SUMEXT, 0xFFFF);
          } else {
            m_regs.write(OFS_SUMEXT, 0x0000);
          }

          // carry
          if ((uint64_t)res < (uint64_t)prev_res) {  // hack
            m_regs.write(OFS_MPY32CTL0, ctrl | MPYC);
          } else {
            m_regs.write(OFS_MPY32CTL0, ctrl & ~MPYC);
          }

        } else {  // Unsigned
          // "Unsigned\n";
          uint16_t op1_L = m_regs.read(OFS_MAC32L);
          uint16_t op1_H = m_regs.read(OFS_MAC32H);
          uint16_t op2_L = m_regs.read(OFS_OP2L);
          uint16_t op2_H = m_regs.read(OFS_OP2H);

          uint128_t op1 = (uint32_t)op1_H << 16 | (uint32_t)op1_L;
          uint128_t op2 = (uint32_t)op2_H << 16 | (uint32_t)op2_L;

          uint128_t res = (uint64_t)(op1 * op2) + prev_res;
          uint16_t resC = (uint16_t)((res >> 64) & 0x000000000000ffff);
          uint16_t res3 = (uint16_t)((res >> 48) & 0x000000000000ffff);
          uint16_t res2 = (uint16_t)((res >> 32) & 0x000000000000ffff);
          uint16_t res1 = (uint16_t)((res >> 16) & 0x000000000000ffff);
          uint16_t res0 = (uint16_t)((res >> 0) & 0x000000000000ffff);

          // Write results
          m_regs.write(OFS_RES3, res3);
          m_regs.write(OFS_RES2, res2);
          m_regs.write(OFS_RES1, res1);
          m_regs.write(OFS_RES0, res0);

          m_regs.write(OFS_SUMEXT, resC);
          if (resC != 0) {
            m_regs.write(OFS_MPY32CTL0, ctrl | MPYC);
          } else {
            m_regs.write(OFS_MPY32CTL0, ctrl & ~MPYC);
          }
        }
      } else {  // MPY or MPYS
        // "No Accumulate\n";

        // Clear Result Registers // Review: Not really done/required?
        // RES3
        for (int i = 15; i >= 0; i--) {
          m_regs.clearBit(OFS_RES3, i, 0);
        }
        // RES2
        for (int i = 15; i >= 0; i--) {
          m_regs.clearBit(OFS_RES2, i, 0);
        }
        // RES1
        for (int i = 15; i >= 0; i--) {
          m_regs.clearBit(OFS_RES1, i, 0);
        }
        // RES0
        for (int i = 15; i >= 0; i--) {
          m_regs.clearBit(OFS_RES0, i, 0);
        }

        if ((ctrl & MPYM__MPYS) != 0) {  // Signed
          // "Signed\n";

          uint16_t op1_L = m_regs.read(OFS_MPYS32L);
          uint16_t op1_H = m_regs.read(OFS_MPYS32H);
          uint16_t op2_L = m_regs.read(OFS_OP2L);
          uint16_t op2_H = m_regs.read(OFS_OP2H);

          uint64_t op1 = (uint32_t)op1_H << 16 | (uint32_t)op1_L;
          uint64_t op2 = (uint32_t)op2_H << 16 | (uint32_t)op2_L;

          op1 = op1 & 0x80000000 ? op1 | 0xffffffff00000000 : op1;
          op2 = op2 & 0x80000000 ? op2 | 0xffffffff00000000 : op2;

          uint64_t res = op1 * op2;
          uint16_t res3 = (uint16_t)((res >> 48) & 0x000000000000ffff);
          uint16_t res2 = (uint16_t)((res >> 32) & 0x000000000000ffff);
          uint16_t res1 = (uint16_t)((res >> 16) & 0x000000000000ffff);
          uint16_t res0 = (uint16_t)((res >> 0) & 0x000000000000ffff);

          // Write results
          m_regs.write(OFS_RES3, res3);
          m_regs.write(OFS_RES2, res2);
          m_regs.write(OFS_RES1, res1);
          m_regs.write(OFS_RES0, res0);

          if ((res3 & 0x8000) != 0) {  // MSB determines the sign
            m_regs.write(OFS_MPY32CTL0, ctrl | MPYC);
            m_regs.write(OFS_SUMEXT, 0xFFFF);
          } else {
            m_regs.write(OFS_MPY32CTL0, ctrl & ~MPYC);
            m_regs.write(OFS_SUMEXT, 0x0000);
          }

        } else {  // Unsigned
          // "Unsigned\n";

          uint16_t op1_L = m_regs.read(OFS_MPY32L);
          uint16_t op1_H = m_regs.read(OFS_MPY32H);
          uint16_t op2_L = m_regs.read(OFS_OP2L);
          uint16_t op2_H = m_regs.read(OFS_OP2H);

          uint64_t op1 = (uint32_t)op1_H << 16 | (uint32_t)op1_L;
          uint64_t op2 = (uint32_t)op2_H << 16 | (uint32_t)op2_L;

          uint64_t res = op1 * op2;
          uint16_t res3 = (uint16_t)((res >> 48) & 0x000000000000ffff);
          uint16_t res2 = (uint16_t)((res >> 32) & 0x000000000000ffff);
          uint16_t res1 = (uint16_t)((res >> 16) & 0x000000000000ffff);
          uint16_t res0 = (uint16_t)((res >> 0) & 0x000000000000ffff);

          // Write results
          m_regs.write(OFS_RES3, res3);
          m_regs.write(OFS_RES2, res2);
          m_regs.write(OFS_RES1, res1);
          m_regs.write(OFS_RES0, res0);

          m_regs.write(OFS_MPY32CTL0, ctrl & ~MPYC);
          m_regs.write(OFS_SUMEXT, 0x0000);
        }
      }
    } else {  // 8x16, 16x8, 16x16, 8x8
      // "16x16\n";
      if ((ctrl & MPYM__MAC) != 0) {  // MAC or MACS
        // "Accumulate\n";
        // Record Previous Results
        uint64_t prev_resC = m_regs.read(OFS_SUMEXT);
        uint64_t prev_res1 = m_regs.read(OFS_RES1);
        uint64_t prev_res0 = m_regs.read(OFS_RES0);

        uint64_t prev_res =
            (prev_resC << 32) | (prev_res1 << 16) | (prev_res0 << 0);

        if ((ctrl & MPYM__MPYS) != 0) {  // Signed
          // "Signed\n";
          uint32_t op1 = m_regs.read(OFS_MACS);
          uint32_t op2 = m_regs.read(OFS_OP2);
          op1 = op1 & 0x8000 ? op1 | 0xffff0000 : op1;
          op2 = op2 & 0x8000 ? op2 | 0xffff0000 : op2;

        } else {
          // "Unsigned\n";
          uint64_t op1 = m_regs.read(OFS_MAC);
          uint64_t op2 = m_regs.read(OFS_OP2);

          uint64_t res = op1 * op2 + prev_res;

          uint16_t resC = (uint16_t)((res >> 32) & 0x0000ffff);
          uint16_t res1 = (uint16_t)((res >> 16) & 0x0000ffff);
          uint16_t res0 = (uint16_t)((res >> 0) & 0x0000ffff);

          m_regs.write(OFS_RES1, res1);
          m_regs.write(OFS_RES0, res0);

          m_regs.write(OFS_SUMEXT, resC);
          if (resC != 0) {
            m_regs.write(OFS_MPY32CTL0, ctrl | MPYC);
          } else {
            m_regs.write(OFS_MPY32CTL0, ctrl & ~MPYC);
          }
        }
      } else {  // MPY or MPYS
        // "No Accumulate\n";
        // Clear Result Registers
        // RES1
        for (int i = 15; i >= 0; i--) {
          m_regs.clearBit(OFS_RES1, i, 0);
        }
        // RES0
        for (int i = 15; i >= 0; i--) {
          m_regs.clearBit(OFS_RES0, i, 0);
        }

        if ((ctrl & MPYM__MPYS) != 0) {  // Signed
          // "Signed\n";

          uint32_t op1 = m_regs.read(OFS_MPYS);
          uint32_t op2 = m_regs.read(OFS_OP2);
          op1 = op1 & 0x8000 ? op1 | 0xffff0000 : op1;
          op2 = op2 & 0x8000 ? op2 | 0xffff0000 : op2;

          uint32_t res = op1 * op2;
          uint16_t res1 = (uint16_t)((res >> 16) & 0x0000ffff);
          uint16_t res0 = (uint16_t)((res >> 0) & 0x0000ffff);
          m_regs.write(OFS_RES1, res1);
          m_regs.write(OFS_RES0, res0);

          if ((res1 & 0x8000) != 0) {  // MSB determines the sign
            m_regs.write(OFS_MPY32CTL0, ctrl | MPYC);
            m_regs.write(OFS_SUMEXT, 0xFFFF);
          } else {
            m_regs.write(OFS_MPY32CTL0, ctrl & ~MPYC);
            m_regs.write(OFS_SUMEXT, 0x0000);
          }

        } else {  // Unsigned
          // "Unsigned\n";

          uint16_t op1 = m_regs.read(OFS_MPY);
          uint16_t op2 = m_regs.read(OFS_OP2);
          uint32_t res = (uint32_t)op1 * (uint32_t)op2;
          uint16_t res1 = (uint16_t)((res >> 16) & 0x0000ffff);
          uint16_t res0 = (uint16_t)((res >> 0) & 0x0000ffff);
          m_regs.write(OFS_RES1, res1);
          m_regs.write(OFS_RES0, res0);

          m_regs.write(OFS_MPY32CTL0, ctrl & ~MPYC);
          m_regs.write(OFS_SUMEXT, 0x0000);
        }
      }
    }

    // RESLO & RESHI for backward compatibility.
    m_regs.write(OFS_RESLO, m_regs.read(OFS_RES0));
    m_regs.write(OFS_RESHI, m_regs.read(OFS_RES1));
  } else {
  }
  return;
}
