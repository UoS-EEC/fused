/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <tlm_utils/simple_initiator_socket.h>
#include <iostream>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/EventLog.hpp"

/**
 * @brief The Spi class Modelling operation of Spi module, based on the one
 * included on the STM32F0 series of microcontrollers.
 *
 */
class Spi : public BusTarget {
  SC_HAS_PROCESS(Spi);

 public:
  /* ------ Ports ------ */
  tlm_utils::simple_initiator_socket<Spi> spiSocket{"spiSocket"};
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};
  sc_core::sc_out<bool> irq{"irq"};  //! Interrupt request output
  sc_core::sc_in<int> returning_exception{
      "returning_exception"};  //! Signals returning exceptions

  /* ------ Register bits ------ */
  // CR1 bits
  static const unsigned BIDIMODE_MASK = 1u << 15;
  static const unsigned BIDIMODE_SHIFT = 15;
  static const unsigned BIDIOE_MASK = 1u << 14;
  static const unsigned BIDIOE_SHIFT = 14;
  static const unsigned CRCEN_MASK = 1u << 13;
  static const unsigned CRCEN_SHIFT = 13;
  static const unsigned CRCNEXT_MASK = 1u << 12;
  static const unsigned CRCNEXT_SHIFT = 12;
  static const unsigned CRCL_MASK = 1u << 11;
  static const unsigned CRCL_SHIFT = 11;
  static const unsigned RXONLY_MASK = 1u << 10;
  static const unsigned RXONLY_SHIFT = 10;
  static const unsigned SSM_MASK = 1u << 9;
  static const unsigned SSM_SHIFT = 9;
  static const unsigned SSI_MASK = 1u << 8;
  static const unsigned SSI_SHIFT = 8;
  static const unsigned LSBFIRST_MASK = 1u << 7;
  static const unsigned LSBFIRST_SHIFT = 7;
  static const unsigned SPE_MASK = 1u << 6;
  static const unsigned SPE_SHIFT = 6;
  static const unsigned BR_MASK = 7u << 3;
  static const unsigned BR_SHIFT = 3;
  static const unsigned MSTR_MASK = 1u << 2;
  static const unsigned MSTR_SHIFT = 2;
  static const unsigned CPOL_MASK = 1u << 1;
  static const unsigned CPOL_SHIFT = 1;
  static const unsigned CPHA_MASK = 1u << 0;
  static const unsigned CPHA_SHIFT = 0;

  // CR2 bits
  static const unsigned LDMA_TX_MASK = 1u << 14;
  static const unsigned LDMA_TX_SHIFT = 14;
  static const unsigned LDMA_RX_MASK = 1u << 13;
  static const unsigned LDMA_RX_SHIFT = 13;
  static const unsigned FRXTH_MASK = 1u << 12;
  static const unsigned FRXTH_SHIFT = 12;
  static const unsigned DS_MASK = 0xfu << 8;
  static const unsigned DS_SHIFT = 8;
  static const unsigned TXEIE_MASK = 1u << 7;
  static const unsigned TXEIE_SHIFT = 7;
  static const unsigned RXNEIE_MASK = 1u << 6;
  static const unsigned RXNEIE_SHIFT = 6;
  static const unsigned ERRIE_MASK = 1u << 5;
  static const unsigned ERRIE_SHIFT = 5;
  static const unsigned FRF_MASK = 1u << 4;
  static const unsigned FRF_SHIFT = 4;
  static const unsigned NSSP_MASK = 1u << 3;
  static const unsigned NSSP_SHIFT = 3;
  static const unsigned SSOE_MASK = 1u << 2;
  static const unsigned SSOE_SHIFT = 2;
  static const unsigned TXDMAEN_MASK = 1u << 1;
  static const unsigned TXDMAEN_SHIFT = 1;
  static const unsigned RXDMAEN_MASK = 1u << 0;
  static const unsigned RXDMAEN_SHIFT = 0;

  // SR bits
  static const unsigned FTLVL_MASK = 3u << 11;
  static const unsigned FTLVL_SHIFT = 11;
  static const unsigned FRLVL_MASK = 3u << 9;
  static const unsigned FRLVL_SHIFT = 9;
  static const unsigned FRE_MASK = 1u << 8;
  static const unsigned FRE_SHIFT = 8;
  static const unsigned BSY_MASK = 1u << 7;
  static const unsigned BSY_SHIFT = 7;
  static const unsigned OVR_MASK = 1u << 6;
  static const unsigned OVR_SHIFT = 6;
  static const unsigned MODF_MASK = 1u << 5;
  static const unsigned MODF_SHIFT = 5;
  static const unsigned CRCERR_MASK = 1u << 4;
  static const unsigned CRCERR_SHIFT = 4;
  static const unsigned TXE_MASK = 1u << 1;
  static const unsigned TXE_SHIFT = 1;
  static const unsigned RXNE_MASK = 1u << 0;
  static const unsigned RXNE_SHIFT = 0;

  /*------ Methods ------*/
  /**
   * @brief Spi Constructor: initialise registers
   * @param name
   * @param start address
   * @param end address
   * @param delay Bus access delay
   */
  Spi(sc_core::sc_module_name name, const unsigned startAddress,
      const unsigned endAddress, const sc_core::sc_time delay);

  /**
   * @brief b_transport Blocking reads and writes to the config/control
   * registers.
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload& trans,
                           sc_core::sc_time& delay) override;

  /**
   * @brief reset Resets the Spi control registers to their default
   * power-up values
   */
  virtual void reset(void) override;

  friend std::ostream& operator<<(std::ostream& os, const Spi& rhs);

 private:
  /* ------ Private types ------- */
  struct Fifo {       // 32-bit FIFO
    int nValidBytes;  // Number of valid bytes (up to 4)
    unsigned data;
    const std::string name;

    Fifo(std::string name_) : nValidBytes(0), data(0), name(name_) {}

    void reset() {
      nValidBytes = 0;
      data = 0;
    }

    unsigned get(const int nbits) {
      sc_assert((nbits >= 4) && (nbits <= 16));  // hw spec
      const int nbytes = ((nbits - 1) / 8) + 1;

      if (nbytes > nValidBytes) {
        spdlog::warn(
            "Spi::Fifo::get requested {:d} bytes, but nValidBytes={:d}. "
            "Returning all 1's.",
            nbytes, nValidBytes);
      }

      nValidBytes -= nbytes;
      if (nbytes == 1) {
        const auto rv = data & 0x000000ff;
        data >>= 8;
        return rv;
      } else {  // 2 bytes
        const auto rv = data & 0x0000ffff;
        data >>= 16;
        return rv;
      }
    }

    bool put(const int nbits, const unsigned val) {
      sc_assert((nbits >= 4) && (nbits <= 16));  // hw spec
      const int nbytes = ((nbits - 1) / 8) + 1;

      if (nbytes + nValidBytes > 4) {
        spdlog::warn(
            "Spi::Fifo::put overflow, put {:d} bytes, but nValidBytes={:d}. "
            "Ignoring put.",
            nbytes, nValidBytes);
        return 1;  // ERROR
      }

      const unsigned mask = (nbytes == 1) ? 0x000000ff : 0x0000ffff;
      data &= ~(mask << (nValidBytes * 8));       // Clear target byte(s)
      data |= (val & mask) << (nValidBytes * 8);  // Set target byte(s)
      nValidBytes += nbytes;
      return 0;  // OK
    }

    int level() const {
      if (nValidBytes <= 2) {
        return nValidBytes;
      } else if (nValidBytes == 3) {
        return 2;
      } else {
        return 3;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Fifo& rhs) {
      // clang-format off
      os << "<Fifo> " << rhs.name
        << fmt::format(" nValidBytes={:d}, data=0x{:08x}",
            rhs.nValidBytes, rhs.data);
      return os;
      // clang-format on
    }
  };

  /* ------ Private variables ------ */
  sc_core::sc_event m_txEvent{"txEvent"};
  sc_core::sc_event m_enableEvent{"enableEvent"};
  sc_core::sc_event m_updateIrqEvent{"updateIrqEvent"};
  bool m_enable{false};
  bool m_setIrq{false};  // Used to asynch control irq flag

  Fifo m_txFifo{"txFifo"};
  Fifo m_rxFifo{"rxFifo"};

  /* ------ Private methods ------ */

  /**
   * @brief process Performs a serial communication type transaction
   * with an external device; payload properties depends on the content
   * of the config/control registers.
   */
  void process();

  /**
   * @brief checkImplemented Checks config registers and errors/warns if
   * unimplemented features are enabled.
   */
  void checkImplemented();

  /**
   * @brief updateStatusRegister update status register flags.
   * @param isBusy indicate if SPI unit is busy (currently transmitting).
   */
  void updateStatusRegister(const bool isBusy);

  /**
   * @brief irqControl SC_METHOD to control the interrupt request line.
   */
  void irqControl();
};
