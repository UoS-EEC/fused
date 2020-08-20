/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <systemc>
#include "include/fused.h"
#include "mcu/SpiTransactionExtension.hpp"
#include "mcu/cortex-m0/Spi.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

using namespace sc_core;

Spi::Spi(sc_module_name name, const unsigned startAddress,
         const unsigned endAddress, const sc_time delay)
    : BusTarget(name, startAddress, endAddress, delay) {
  // Register events

  // Initialise register file
  m_regs.addRegister(OFS_SPI_CR1, 0);
  m_regs.addRegister(OFS_SPI_CR2, 0x0700);
  m_regs.addRegister(OFS_SPI_SR, 0x0002);
  m_regs.addRegister(OFS_SPI_DR, 0);
  m_regs.addRegister(OFS_SPI_CRCPR, 0x0007);
  m_regs.addRegister(OFS_SPI_RXCRCR, 0);
  m_regs.addRegister(OFS_SPI_TXCRCR, 0);

  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_THREAD(process);
}

void Spi::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  // Access register file
  BusTarget::b_transport(trans, delay);
  const auto addr = trans.get_address();
  const auto val = m_regs.read(addr);
  const auto len = trans.get_data_length();

  if (len > 2) {
    SC_REPORT_FATAL(this->name(),
                    "The SPI module only accepts 8-bit and 16-but accesses.");
  }

  // Handle serial communication tasks
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    switch (addr) {
      case OFS_SPI_CR1:  // Control Register 1
        if (!m_enable && (val & Spi::SPE_MASK)) {
          m_enableEvent.notify(delay);
        }
        m_enable = val & Spi::SPE_MASK;
        checkImplemented();
        break;
      case OFS_SPI_CR2:  // Control Register 2
        checkImplemented();
        break;
      case OFS_SPI_DR:  // Data register -- push tx fifo
        m_txFifo.put(8 * len, val);
        m_txEvent.notify(delay);
        break;
      default:
        break;
    }
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    switch (addr) {
      case OFS_SPI_DR:   // Data register -- pop rx fifo
        if (len == 2) {  // 2 bytes
          Utility::unpackBytes(trans.get_data_ptr(),
                               Utility::htots(m_rxFifo.get(16)), 2);
        } else {  // 1 byte
          trans.get_data_ptr()[0] = m_rxFifo.get(8);
        }
        break;
      default:
        break;
    }
  }
}

void Spi::reset(void) {
  if (pwrOn.read()) {  // Posedge of pwrOn
    // Reset register file
    m_regs.reset();
  }
}

void Spi::process(void) {
  // Prepare payload object
  tlm::tlm_generic_payload trans;
  trans.set_command(tlm::TLM_WRITE_COMMAND);
  std::array<uint8_t, 2> dataOut;
  trans.set_data_ptr(&dataOut[0]);
  auto *spiExtension = new SpiTransactionExtension();
  trans.set_extension(spiExtension);
  trans.set_address(0);  // SPI doesn't use address

  wait(SC_ZERO_TIME);  // Wait for start of simulation

  while (1) {
    unsigned cr2 = m_regs.read(OFS_SPI_CR2);
    int nbits = ((cr2 & Spi::DS_MASK) >> Spi::DS_SHIFT) + 1;
    if (nbits < 4) {
      nbits = 8;
    }
    int nbytes = (nbits + 7) / 8;
    bool readyToSend =
        pwrOn.read() && m_enable && (m_txFifo.nValidBytes >= nbytes);

    while (!readyToSend) {
      wait(m_txEvent | m_enableEvent | pwrOn.posedge_event());
      cr2 = m_regs.read(OFS_SPI_CR2);
      nbits = ((cr2 & Spi::DS_MASK) >> Spi::DS_SHIFT) + 1;
      if (nbits < 4) {
        nbits = 8;
      }
      nbytes = (nbits + 7) / 8;
      readyToSend =
          pwrOn.read() && m_enable && (m_txFifo.nValidBytes >= nbytes);
    }

    // Update status register
    m_regs.setBit(OFS_SPI_SR, BSY_SHIFT);  // Set busy flag

    // Prepare  & send payload
    const unsigned cr1 = m_regs.read(OFS_SPI_CR1);
    spiExtension->bitOrder =
        (cr1 & Spi::LSBFIRST_MASK)
            ? SpiTransactionExtension::SpiBitOrder::LSB_FIRST
            : SpiTransactionExtension::SpiBitOrder::MSB_FIRST;
    auto baudRateDivider = 2u << ((cr1 & Spi::BR_MASK) >> Spi::BR_SHIFT);
    spiExtension->clkPeriod = clk->getPeriod() * baudRateDivider;
    spiExtension->polarity = (cr1 & CPOL_MASK)
                                 ? SpiTransactionExtension::SpiPolarity::LOW
                                 : SpiTransactionExtension::SpiPolarity::HIGH;
    spiExtension->phase =
        (cr1 & CPHA_MASK)
            ? SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE
            : SpiTransactionExtension::SpiPhase::CAPTURE_SECOND_EDGE;
    spiExtension->nDataBits = nbits;

    trans.set_data_length(nbytes);
    Utility::unpackBytes(&dataOut[0], m_txFifo.get(nbits), nbytes);
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    sc_time delay = spiExtension->transferTime();
    spiSocket->b_transport(trans, delay);
    if (trans.is_response_error()) {
      SC_REPORT_FATAL(this->name(), "TLM response error");
    }
    wait(delay);

    // Receive
    m_rxFifo.put(nbits, spiExtension->response);

    // Update status register
    // RXNE: when the threshold defined by FRXTH is reached
    // TXE: when TXFIFO level is <= half capacity
    const int FRXTH_nbytes =
        ((cr2 & Spi::FRXTH_MASK) >> Spi::FRXTH_SHIFT) ? 1 : 2;
    const unsigned RXNE = m_rxFifo.nValidBytes >= FRXTH_nbytes;
    const unsigned TXE = m_txFifo.nValidBytes <= 2;
    m_regs.write(OFS_SPI_SR, (m_txFifo.level() << FTLVL_SHIFT) |
                                 (m_rxFifo.level() << FRLVL_SHIFT) |
                                 (TXE << TXE_SHIFT) | (RXNE << RXNE_SHIFT));

    // Set Interrupt request
    const bool RXNEIE = cr2 & Spi::RXNEIE_MASK;
    const bool TXEIE = cr2 & Spi::TXEIE_MASK;
  }
}

void Spi::checkImplemented() {
  unsigned cr1 = m_regs.read(OFS_SPI_CR1);

  if (cr1 & Spi::BIDIMODE_MASK) {
    SC_REPORT_FATAL(
        this->name(),
        fmt::format(
            "BIDIMODE set, but bidirectional data mode not implemented.")
            .c_str());
  }

  if (cr1 & Spi::CRCEN_MASK) {
    SC_REPORT_FATAL(
        this->name(),
        fmt::format("CRCEN set, but hardware CRC calculation not implemented.")
            .c_str());
  }

  if (cr1 & Spi::CRCNEXT_MASK) {
    SC_REPORT_FATAL(
        this->name(),
        fmt::format(
            "CRCNEXT set, but hardware CRC calculation not implemented.")
            .c_str());
  }

  if (cr1 & Spi::RXONLY_MASK) {
    SC_REPORT_FATAL(
        this->name(),
        fmt::format("RXONLY set, but receive-only mode not implemented.")
            .c_str());
  }
}
