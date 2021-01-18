/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "libs/make_unique.hpp"
#include "ps/ConstantCurrentState.hpp"
#include "sd/Nrf24Radio.hpp"

using namespace sc_core;

Nrf24Radio::Nrf24Radio(const sc_core::sc_module_name name) : SpiDevice(name) {
  // Build registers file
  // Control registers
  m_regs.addRegister(NRF_CONFIG, EN_CRC);
  m_regs.addRegister(EN_AA,
                     ENAA_P5 | ENAA_P4 | ENAA_P3 | ENAA_P2 | ENAA_P1 | ENAA_P0);
  m_regs.addRegister(EN_RXADDR, ERX_P1 | ERX_P0);
  m_regs.addRegister(SETUP_AW, AW_1 | AW_0);
  m_regs.addRegister(SETUP_RETR, ARC_1 | ARC_0);
  m_regs.addRegister(RF_CH, 2);
  m_regs.addRegister(RF_SETUP, RF_DR_HIGH);
  m_regs.addRegister(NRF_STATUS, RX_P_NO_2 | RX_P_NO_1 | RX_P_NO_0);
  m_regs.addRegister(OBSERVE_TX, 0);
  m_regs.addRegister(RPD, 0);
  m_regs.addRegister(RX_ADDR_P0, 0xE7);  // 5 byte register access point
  m_regs.addRegister(RX_ADDR_P1, 0xC2);  // 5 byte register access point
  m_regs.addRegister(RX_ADDR_P2, 0xC3);  // 1 byte (byte 1:4 from P1)
  m_regs.addRegister(RX_ADDR_P3, 0xC4);  // 1 byte (byte 1:4 from P1)
  m_regs.addRegister(RX_ADDR_P4, 0xC5);  // 1 byte (byte 1:4 from P1)
  m_regs.addRegister(RX_ADDR_P5, 0xC6);  // 1 byte (byte 1:4 from P1)
  m_regs.addRegister(TX_ADDR, 0xE7);     // 5 byte register access point
  m_regs.addRegister(RX_PW_P0, 0);
  m_regs.addRegister(RX_PW_P1, 0);
  m_regs.addRegister(RX_PW_P2, 0);
  m_regs.addRegister(RX_PW_P3, 0);
  m_regs.addRegister(RX_PW_P4, 0);
  m_regs.addRegister(RX_PW_P5, 0);
  m_regs.addRegister(FIFO_STATUS, TX_EMPTY | RX_EMPTY);
  m_regs.addRegister(DYNPD, 0);
  m_regs.addRegister(FEATURE, 0);

  // Initialise State Machines
  m_radio_state = OpModes::UNDEFINED;
  m_payloadType = PayloadType::COMMAND;
}

void Nrf24Radio::end_of_elaboration() {
  // Register power modelling states
  m_porStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "por"));
  m_powerDownStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "power_down"));
  m_startUpStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "start_up"));
  m_standbyOneStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "standby_one"));
  m_standbyTwoStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "standby_two"));
  m_rxSettlingStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "rx_settling"));
  m_txSettlingStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "tx_settling"));
  m_rxModeStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "rx_mode"));
  m_txModeStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "tx_mode"));

  // Set up methods & threads
  SC_METHOD(payloadReceivedHandler);
  sensitive << m_transactionEvent;

  SC_METHOD(chipSelectHandler);
  sensitive << chipSelect.value_changed_event();

  SC_METHOD(chipEnableHandler);
  sensitive << chipEnable.value_changed_event();

  SC_METHOD(nResetHandler);
  sensitive << nReset.value_changed_event();

  SC_METHOD(irqEventHandler);
  sensitive << m_irqEvent;

  SC_THREAD(txEventHandler);

  SC_THREAD(stateChangeHandler);
}

void Nrf24Radio::reset(void) {
  // Restore register values to power on defaults
  m_regs.write(NRF_CONFIG, EN_CRC, true);
  m_regs.write(EN_AA, ENAA_P5 | ENAA_P4 | ENAA_P3 | ENAA_P2 | ENAA_P1 | ENAA_P0,
               true);
  m_regs.write(EN_RXADDR, ERX_P1 | ERX_P0, true);
  m_regs.write(SETUP_AW, AW_1 | AW_0, true);
  m_regs.write(SETUP_RETR, ARC_1 | ARC_0, true);
  m_regs.write(RF_CH, 2, true);
  m_regs.write(RF_SETUP, RF_DR_HIGH, true);
  m_regs.write(NRF_STATUS, RX_P_NO_2 | RX_P_NO_1 | RX_P_NO_0, true);
  m_regs.write(OBSERVE_TX, 0, true);
  m_regs.write(RPD, 0, true);
  m_regs.write(RX_ADDR_P0, 0xE7, true);  // 5 byte register access point
  m_regs.write(RX_ADDR_P1, 0xC2, true);  // 5 byte register access point
  m_regs.write(RX_ADDR_P2, 0xC3, true);  // 1 byte (byte 1:4 from P1)
  m_regs.write(RX_ADDR_P3, 0xC4, true);  // 1 byte (byte 1:4 from P1)
  m_regs.write(RX_ADDR_P4, 0xC5, true);  // 1 byte (byte 1:4 from P1)
  m_regs.write(RX_ADDR_P5, 0xC6, true);  // 1 byte (byte 1:4 from P1)
  m_regs.write(TX_ADDR, 0xE7, true);     // 5 byte register access point
  m_regs.write(RX_PW_P0, 0, true);
  m_regs.write(RX_PW_P1, 0, true);
  m_regs.write(RX_PW_P2, 0, true);
  m_regs.write(RX_PW_P3, 0, true);
  m_regs.write(RX_PW_P4, 0, true);
  m_regs.write(RX_PW_P5, 0, true);
  m_regs.write(FIFO_STATUS, TX_EMPTY | RX_EMPTY, true);
  m_regs.write(DYNPD, 0, true);
  m_regs.write(FEATURE, 0, true);

  // Reset State Machines
  m_radio_state = OpModes::UNDEFINED;
  m_payloadType = PayloadType::COMMAND;

  // Reset SPI shift registers.
  SpiDevice::reset();

  // IRQ Line Defaut High
  m_irqEvent.notify();
}

void Nrf24Radio::payloadReceivedHandler(void) {
  if (nReset.read()) {
    static uint32_t command;
    static uint32_t targetRegister;
    static uint32_t targetPipe;
    static uint32_t maxDataBytes;
    static uint32_t processedDataBytes;

    const auto payload = readSlaveIn();
    spdlog::info("{:s}: @{:s} Received 0x{:08x}", this->name(),
                 sc_time_stamp().to_string(), payload);
    if (m_payloadType == PayloadType::COMMAND) {
      command = payload;
      processedDataBytes = 0;
      switch (payload) {
        case R_RX_PAYLOAD:
          m_payloadType = PayloadType::DATA;
          maxDataBytes = 32;
          break;
        case W_TX_PAYLOAD:
          m_payloadType = PayloadType::DATA;
          maxDataBytes = 32;
          if (m_txFifo.isFull()) {
            spdlog::warn("{:s}: @{:s} TX Fifo Full.", this->name(),
                         sc_time_stamp().to_string());
          } else {
            m_txFifo.push();
          }
          writeSlaveOut(0x00);
          break;
        case FLUSH_TX:
          break;
        case FLUSH_RX:
          break;
        case REUSE_TX_PL:
          break;
        case R_RX_PL_WID:
          m_payloadType = PayloadType::DATA;
          maxDataBytes = 32;
          break;
        case W_TX_PAYLOAD_NO_ACK:
          m_payloadType = PayloadType::DATA;
          maxDataBytes = 32;
          break;
        case RF24_NOP:
          break;
        default:
          if ((payload & RW_COMMAND_MASK) == R_REGISTER) {
            m_payloadType = PayloadType::DATA;
            maxDataBytes = 5;
            command &= RW_COMMAND_MASK;
            targetRegister = payload & RW_ADDRESS_MASK;
            writeSlaveOut(m_regs.read(targetRegister));
          } else if ((payload & RW_COMMAND_MASK) == W_REGISTER) {
            m_payloadType = PayloadType::DATA;
            maxDataBytes = 5;
            command &= RW_COMMAND_MASK;
            targetRegister = payload & RW_ADDRESS_MASK;
            writeSlaveOut(0x00);
          } else if ((payload & WAP_COMMAND_MASK) == W_ACK_PAYLOAD) {
            m_payloadType = PayloadType::DATA;
            maxDataBytes = 32;
            targetPipe = payload & WAP_PIPE_MASK;
            command &= WAP_COMMAND_MASK;
            writeSlaveOut(0x00);
          } else {
            spdlog::warn("{:s}: @{:s} Bad payload.", this->name(),
                         sc_time_stamp().to_string());
          }
      }
    } else {
      if (processedDataBytes > maxDataBytes) {
        spdlog::warn("{:s}: @{:s} Overflowed data bytes.", this->name(),
                     sc_time_stamp().to_string());
      }
      switch (command) {
        case R_REGISTER:
          writeSlaveOut(m_regs.read(++targetRegister));
          break;
        case W_REGISTER:
          m_regs.write(targetRegister, payload, true);
          // Handle Tx irq clear
          if (targetRegister == NRF_STATUS) {
            if ((payload & TX_DS) != 0) {
              m_regs.write(targetRegister,
                           m_regs.read(targetRegister) & ~TX_DS);
              m_irqEvent.notify();
            }
          }
          targetRegister++;
          m_stateChangeEvent.notify();
          break;
        case R_RX_PAYLOAD:
          break;
        case W_TX_PAYLOAD:
          m_txFifo.appendPayload(payload);
          break;
        case R_RX_PL_WID:
          break;
        case W_ACK_PAYLOAD:
          break;
        case W_TX_PAYLOAD_NO_ACK:
          break;
        default:
          spdlog::warn("{:s}: @{:s} Bad payload.", this->name(),
                       sc_time_stamp().to_string());
      }
    }
  }
}

void Nrf24Radio::chipSelectHandler(void) {
  if (enabled()) {
    spdlog::info("{:s}: @{:s} CS |_; Communication starts.", this->name(),
                 sc_time_stamp().to_string());
    writeSlaveOut(m_regs.read(NRF_STATUS));
    m_payloadType =
        PayloadType::COMMAND;  // The next payload is expected to be a command
  } else {
    spdlog::info("{:s}: @{:s} CS _|; Communication halted.", this->name(),
                 sc_time_stamp().to_string());
    if (m_radio_state != OpModes::UNDEFINED) {
      m_stateChangeEvent.notify();
    }
  }
}

void Nrf24Radio::chipEnableHandler(void) {
  if (nReset.read()) {
    m_stateChangeEvent.notify();
  }
}

void Nrf24Radio::nResetHandler(void) {
  if (nReset.read()) {
    m_stateChangeEvent.notify();
  } else {
    m_radio_state = OpModes::UNDEFINED;
  }
}

void Nrf24Radio::stateChangeHandler(void) {
  while (1) {
    wait(m_stateChangeEvent);
    const bool ce = chipEnable.read() == sc_dt::SC_LOGIC_1 ? true : false;
    switch (m_radio_state) {
      case OpModes::UNDEFINED:
        m_radio_state = OpModes::POWER_ON_RESET;
        powerModelPort->reportState(m_porStateId);
        wait(sc_time(10, SC_MS));
        powerModelPort->reportState(m_powerDownStateId);
        m_radio_state = OpModes::POWER_DOWN;
        break;
      case OpModes::POWER_DOWN:
        if (m_regs.read(NRF_CONFIG) & PWR_UP) {
          m_radio_state = OpModes::OSC_STARTUP;
          powerModelPort->reportState(m_startUpStateId);
          wait(sc_time(150, SC_US));  // For External crystal
          powerModelPort->reportState(m_standbyOneStateId);
          m_radio_state = OpModes::STANDBY1;
        }
        break;
      case OpModes::STANDBY1:
        if ((m_regs.read(NRF_CONFIG) & PWR_UP) == 0) {
          powerModelPort->reportState(m_powerDownStateId);
          m_radio_state = OpModes::POWER_DOWN;
        } else if ((m_regs.read(NRF_CONFIG) & PRIM_RX) && ce) {
          powerModelPort->reportState(m_rxSettlingStateId);
          m_radio_state = OpModes::RX_SETTLING;
          wait(sc_time(130, SC_US));
          powerModelPort->reportState(m_rxModeStateId);
          m_radio_state = OpModes::RX_MODE;
        } else if (!(m_regs.read(NRF_CONFIG) & PRIM_RX) && ce &&
                   !m_txFifo.isEmpty()) {
          powerModelPort->reportState(m_txSettlingStateId);
          m_radio_state = OpModes::TX_SETTLING;
          wait(sc_time(130, SC_US));
          powerModelPort->reportState(m_txModeStateId);
          m_radio_state = OpModes::TX_MODE;
          m_txEvent.notify();
        } else if (!(m_regs.read(NRF_CONFIG) & PRIM_RX) && ce &&
                   m_txFifo.isEmpty()) {
          powerModelPort->reportState(m_standbyTwoStateId);
          m_radio_state = OpModes::STANDBY2;
        }
        break;
      case OpModes::STANDBY2:
        if ((m_regs.read(NRF_CONFIG) & PWR_UP) == 0) {
          powerModelPort->reportState(m_powerDownStateId);
          m_radio_state = OpModes::POWER_DOWN;
        } else if (ce && !m_txFifo.isEmpty()) {
          powerModelPort->reportState(m_txSettlingStateId);
          m_radio_state = OpModes::TX_SETTLING;
          wait(sc_time(130, SC_US));
          powerModelPort->reportState(m_txModeStateId);
          m_radio_state = OpModes::TX_MODE;
          m_txEvent.notify();
        }
        break;
      case OpModes::RX_MODE:
        if ((m_regs.read(NRF_CONFIG) & PWR_UP) == 0) {
          powerModelPort->reportState(m_powerDownStateId);
          m_radio_state = OpModes::POWER_DOWN;
        } else if (!ce) {
          powerModelPort->reportState(m_standbyOneStateId);
          m_radio_state = OpModes::STANDBY1;
        }
        break;
      case OpModes::TX_MODE:
        if ((m_regs.read(NRF_CONFIG) & PWR_UP) == 0) {
          powerModelPort->reportState(m_powerDownStateId);
          m_radio_state = OpModes::POWER_DOWN;
        } else if (!ce) {
          powerModelPort->reportState(m_standbyOneStateId);
          m_radio_state = OpModes::STANDBY1;
        } else if (ce && !m_txFifo.isEmpty()) {
          m_txEvent.notify();
        } else if (ce && m_txFifo.isEmpty()) {
          powerModelPort->reportState(m_standbyTwoStateId);
          m_radio_state = OpModes::STANDBY2;
        }
        break;
      default:
        spdlog::warn("{:s}: @{:s} Bad state.", this->name(),
                     sc_time_stamp().to_string());
    }
  }
}

void Nrf24Radio::txEventHandler(void) {
  while (1) {
    wait(m_txEvent);
    // Construct Tx packet
    m_txPacket.channelNumber = m_regs.read(RF_CH) & 0b01111111;

    if ((m_regs.read(RF_SETUP) & 0b00000110) == RF_PWR_0) {
      m_txPacket.outputPower = RadioPacket::OutputPower::_n12dBm;
    } else if ((m_regs.read(RF_SETUP) & 0b00000110) == RF_PWR_1) {
      m_txPacket.outputPower = RadioPacket::OutputPower::_n6dBm;
    } else if ((m_regs.read(RF_SETUP) & 0b00000110) == (RF_PWR_1 | RF_PWR_0)) {
      m_txPacket.outputPower = RadioPacket::OutputPower::_0dBm;
    } else {
      m_txPacket.outputPower = RadioPacket::OutputPower::_n18dBm;
    }

    m_txPacket.dataRate = RadioPacket::DataRate::_1Mbps;
    if ((m_regs.read(RF_SETUP) & 0b00100000) == RF_DR_LOW) {
      m_txPacket.dataRate = RadioPacket::DataRate::_250kbps;
    } else if ((m_regs.read(RF_SETUP) & 0b00001000) == RF_DR_HIGH) {
      m_txPacket.dataRate = RadioPacket::DataRate::_2Mbps;
    } else {
      m_txPacket.dataRate = RadioPacket::DataRate::_1Mbps;
    }

    if ((m_regs.read(SETUP_AW) & (AW_1 | AW_0)) == AW_0) {
      m_txPacket.addressSize = 3;
    } else if ((m_regs.read(SETUP_AW) & (AW_1 | AW_0)) == AW_1) {
      m_txPacket.addressSize = 4;
    } else if ((m_regs.read(SETUP_AW) & (AW_1 | AW_0)) == (AW_1 | AW_0)) {
      m_txPacket.addressSize = 5;
    }
    for (int i = 0; i < m_txPacket.addressSize; i++) {
      m_txPacket.address[i] = m_regs.read(TX_ADDR);
    }

    for (int i = 0; i < m_txFifo.getPayloadSize(); i++) {
      m_txPacket.payload[i] = m_txFifo.readPayload(i);
    }
    m_txPacket.payloadSize = m_txFifo.getPayloadSize();

    // Tx
    wait(sc_time(m_txPacket.packetDuration(), SC_US));
    m_txFifo.pop();
    spdlog::info("{:s}: @{:s} Packet Transmitted", this->name(),
                 sc_time_stamp().to_string());

    // Tx interrupt request
    m_regs.write(NRF_STATUS, m_regs.read(NRF_STATUS) | TX_DS);
    m_irqEvent.notify();

    m_stateChangeEvent.notify();
  }
}

void Nrf24Radio::irqEventHandler(void) {
  if (nReset.read()) {
    // The TX_DS bit in STATUS will be set/cleared already
    // This handler simple controls the irqEventHandler signal
    if ((m_regs.read(NRF_STATUS) & TX_DS) != 0) {             // Irq not set
      if (((m_regs.read(NRF_CONFIG) & (MASK_TX_DS)) == 0)) {  // Not masked
        interruptRequest.write(sc_dt::sc_logic(false));
      }
    } else {  // Irq set
      interruptRequest.write(sc_dt::sc_logic(true));
    }
  }
}
