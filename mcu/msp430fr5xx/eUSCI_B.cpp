/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "mcu/SpiTransactionExtension.hpp"
#include "mcu/msp430fr5xx/eUSCI_B.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

eUSCI_B::eUSCI_B(sc_module_name name, const uint16_t startAddress,
                 const uint16_t endAddress, const sc_time delay)
    : BusTarget(name, startAddress, endAddress, delay) {
  // Register events

  // Initialise register file
  uint16_t endOffset = endAddress - startAddress + 1;
  for (uint16_t i = 0; i < endOffset; i += 2) {
    m_regs.addRegister(i, 0, RegisterFile::READ_WRITE);
  }

  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_THREAD(dmaEventHandler);
  SC_THREAD(process);
}

void eUSCI_B::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  // Access register file
  BusTarget::b_transport(trans, delay);
  auto addr = trans.get_address();

  // Handle serial communication tasks
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    switch (addr) {
      case OFS_UCB0CTLW0:
        // Control Register 0
        // Setting UCSWRST resets the eUSCI module.
        if (m_regs.read(OFS_UCB0CTLW0) & UCSWRST) {
          this->swreset();
        }
        // Update tlm_generic_paylod extension.
        break;
      case OFS_UCB0BRW:
        // Bit Rate Control Register 1
        // Update tlm_generic_payload clk parameter.
        break;
      case OFS_UCB0STATW:
        // Status Register
        // Setting UCLISTEN enable tx -> rx feedback.
        break;
      case OFS_UCB0RXBUF:
        // Receive Buffer Register. Read only.
        break;
      case OFS_UCB0TXBUF:
        // Transmit Buffer Register
        // Transmission starts after write.
        // UCTXIFG reset.
        m_euscibTxEvent.notify();
        break;
      case OFS_UCB0IE:
        // Interrupt Enable Register
        // Set UCTXIE and UCRXIE to enable interrupts.
        break;
      case OFS_UCB0IFG:
        // Interrupt Flag Register
        break;
      case OFS_UCB0IV:
        // Interrupt Vector Register. Read only.
        // Access resets the highest-pending interrupt flag.
        // If another flag is set, a second interrupt follows.
        break;
      default:
        break;
    }
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    switch (addr) {
      case OFS_UCB0STATW:
        // UCBUSY bit indicates TX or RX in progress
        break;
      case OFS_UCB0RXBUF:
        // Reading from the RX buffer clears UCRXIFG and UCOE.
        m_regs.write(OFS_UCB0IFG, m_regs.read(OFS_UCB0IFG) & ~UCRXIFG);
        break;
      case OFS_UCB0IV:
        if (m_regs.read(OFS_UCB0IFG) & UCRXIFG) {
          m_regs.write(OFS_UCB0IV, 0x02);
        } else {
          m_regs.write(OFS_UCB0IV, 0x00);
        }
        break;
      default:
        break;
    }
  }
}

void eUSCI_B::reset(void) {
  if (pwrOn.read()) {  // Posedge of pwrOn
    // Reset register file
    for (uint16_t i = 0; i < m_regs.size(); i++) {
      m_regs.write(2 * i, 0);
    }
    m_regs.write(OFS_UCB0CTLW0, 0x01c1);
    m_regs.write(OFS_UCB0IFG, 0x0002);
  }
}

void eUSCI_B::swreset(void) {
  if (pwrOn.read()) {  // Posedge of pwrOn
    // Reset register file
    m_regs.write(OFS_UCB0IE, 0x0000);
    m_regs.write(OFS_UCB0IFG, 0x0002);
  }
}

void eUSCI_B::dmaEventHandler(void) {
  wait(SC_ZERO_TIME);
  while (1) {
    wait(m_dmaTriggerEvent);

    sc_core::sc_time trigger_period{sc_core::SC_ZERO_TIME};
    if (m_regs.read(OFS_UCB0CTLW0) & UCSSEL1) {
      trigger_period = smclk->getPeriod();
    } else {
      trigger_period = aclk->getPeriod();
    }

    dmaTrigger.write(true);
    wait(trigger_period);
    dmaTrigger.write(false);
  }
}

void eUSCI_B::process(void) {
  // Prepare payload object
  tlm::tlm_generic_payload trans;
  auto *spiExtension = new SpiTransactionExtension();
  trans.set_extension(spiExtension);
  trans.set_address(0);      // SPI doesn't use address
  trans.set_data_length(1);  // Transfer size up to 1 byte

  wait(SC_ZERO_TIME);  // Wait for sim to start

  while (1) {
    if (pwrOn.read() == false) {
      wait(pwrOn.posedge_event());
    }
    wait(m_euscibTxEvent);

    // Clear the TXIFG flag
    m_regs.write(OFS_UCB0IFG, m_regs.read(OFS_UCB0IFG) & ~(UCTXIFG));
    // Set the eUSCI busy flag
    m_regs.write(OFS_UCB0STATW, m_regs.read(OFS_UCB0STATW) | UCBUSY);

    auto data = m_regs.readByte(OFS_UCB0TXBUF);
    auto spiParameters = m_regs.read(OFS_UCB0CTLW0);
    sc_core::sc_time trigger_period{sc_core::SC_ZERO_TIME};
    if (m_regs.read(OFS_UCB0CTLW0) & UCSSEL1) {
      spiExtension->clkPeriod = smclk->getPeriod() * m_regs.read(OFS_UCB0BRW);
    } else {
      spiExtension->clkPeriod = aclk->getPeriod() * m_regs.read(OFS_UCB0BRW);
    }
    spiExtension->nDataBits = (spiParameters & UC7BIT) ? 7 : 8;
    spiExtension->phase =
        (spiParameters & UCCKPH)
            ? SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE
            : SpiTransactionExtension::SpiPhase::CAPTURE_SECOND_EDGE;
    spiExtension->polarity = (spiParameters & UCCKPL)
                                 ? SpiTransactionExtension::SpiPolarity::HIGH
                                 : SpiTransactionExtension::SpiPolarity::LOW;
    spiExtension->bitOrder =
        (spiParameters & UCMSB)
            ? SpiTransactionExtension::SpiBitOrder::MSB_FIRST
            : SpiTransactionExtension::SpiBitOrder::LSB_FIRST;
    sc_time delay = spiExtension->transferTime();

    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_data_ptr(&data);
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    // Blocking transport call
    iEusciSocket->b_transport(trans, delay);

    // Check response status
    if (trans.is_response_error()) {
      SC_REPORT_FATAL(this->name(), "Response error");
    }

    wait(delay);

    // Tx Done, Rx Done; Set Interrupt Flags
    m_regs.write(OFS_UCB0IFG, m_regs.read(OFS_UCB0IFG) | UCTXIFG | UCRXIFG);
    // Received payload in RXBUF
    m_regs.write(OFS_UCB0RXBUF, spiExtension->response);

    // DMA
    // Due to ready to transmit new data
    bool dma_flag = 0;
    if (!(m_regs.read(OFS_UCB0IE) & UCTXIE)) {
      dma_flag = 1;
    }
    // Due to receiving new data
    if (!(m_regs.read(OFS_UCB0IE) & UCRXIE)) {
      dma_flag = 1;
    }
    // Pull dma trigger low
    if (dma_flag) {
      m_dmaTriggerEvent.notify();
    }

    // Generate peripheral interrupt vector
    if (m_regs.read(OFS_UCB0IE) & UCTXIE) {  // Prioritize TX
      m_regs.write(OFS_UCB0IV, 0x04);
    } else if (m_regs.read(OFS_UCB0IE) & UCRXIE) {
      m_regs.write(OFS_UCB0IV, 0x02);
    }

    // Generate Interrupt if TX or RX Interrupt Enabled
    if (m_regs.read(OFS_UCB0IE)) {
      // Set IRQ if TX or RX interrupt flag set
      irq.write(m_regs.read(OFS_UCB0IFG) > 0);
      wait(ira.posedge_event());
      if (m_regs.read(OFS_UCB0IFG) & UCTXIFG &&
          m_regs.read(OFS_UCB0IE) & UCTXIE) {  // Interrupt was due to TX
        irq.write(false);
        // Another interrupt immediately if UCRXIE & UCRxIFG set
        if ((m_regs.read(OFS_UCB0IE) & UCRXIE) &&
            (m_regs.read(OFS_UCB0IFG) & UCRXIFG)) {
          // Reguest another interrupt
          wait(SC_ZERO_TIME);
          irq.write(true);
          wait(ira.posedge_event());
          irq.write(false);
        }
      } else if (m_regs.read(OFS_UCB0IFG) & UCRXIFG &&
                 m_regs.read(OFS_UCB0IE) & UCRXIE) {  // due to RX
        irq.write(false);
      }
    }

    // eUSCI no longer busy
    m_regs.write(OFS_UCB0STATW, m_regs.read(OFS_UCB0STATW) & ~(UCBUSY));
  }
}
