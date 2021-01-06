/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <iostream>
#include <systemc>

#include "sd/SpiDevice.hpp"
#include "sd/nRF24L01.h"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

class RadioFifo {
 public:
  RadioFifo() {
    m_fifoWriteIndex = 0;
    m_fifoReadIndex = 1;
    m_payloadIndex = 0;
    m_size = 0;
  };

  void appendPayload(unsigned int b) {
    m_fifo[m_fifoWriteIndex][m_payloadIndex] = b;
    m_payloadIndex++;
    m_payloadSize[m_fifoWriteIndex] = m_payloadIndex;
  }

  unsigned int readPayload(unsigned int index) {
    return m_fifo[m_fifoReadIndex][index];
  }

  unsigned int getPayloadSize(void) { return m_payloadSize[m_fifoReadIndex]; }

  void push(void) {
    if (m_size < 3) {
      m_fifoWriteIndex = m_fifoWriteIndex == 2 ? 0 : m_fifoWriteIndex + 1;
      m_size++;
      m_payloadIndex = 0;
    }
  }

  void pop(void) {
    if (m_size > 0) {
      m_fifoReadIndex = m_fifoReadIndex == 2 ? 0 : m_fifoReadIndex + 1;
      m_size--;
    }
  }

  bool isEmpty(void) { return m_size == 0; }

  bool isFull(void) { return m_size == FifoSize; }

 private:
  static const unsigned int FifoSize = 3;
  static const unsigned int PayloadSize = 32;
  unsigned int m_size;
  unsigned int m_fifoWriteIndex;
  unsigned int m_fifoReadIndex;
  unsigned int m_payloadIndex;
  unsigned int m_fifo[FifoSize][PayloadSize];
  unsigned int m_payloadSize[FifoSize];
};

class RadioPacket {
 public:
  /* ------ Member Types ------*/
  enum class DataRate { _1Mbps, _2Mbps, _250kbps };
  enum class OutputPower { _n18dBm, _n12dBm, _n6dBm, _0dBm };

  /* ------ Constants ------*/
  unsigned int PREAMBLE = 0b10101010;
  static const unsigned int MAX_ADDRESS_SIZE = 5;
  static const unsigned int MAX_PAYLOAD_SIZE = 32;
  static const unsigned int MAX_CRC_SIZE = 2;

  /* ------ Public Variables ------ */
  DataRate dataRate;
  OutputPower outputPower;
  unsigned int channelNumber;
  unsigned int address[MAX_ADDRESS_SIZE];
  unsigned int addressSize;
  unsigned int payload[MAX_PAYLOAD_SIZE];
  unsigned int payloadSize;
  unsigned int crc[MAX_CRC_SIZE];
  unsigned int crcSize;

  /* ------ Public Methods ------ */

  RadioPacket() {
    dataRate = DataRate::_1Mbps;
    outputPower = OutputPower::_0dBm;
    channelNumber = 0;
    addressSize = 0;
    payloadSize = 0;
    crcSize = 0;
  }

  unsigned int packetSize(void) const {
    return addressSize + payloadSize + crcSize +
           1;  // There is always a preamble
  }

  unsigned int bitPeriod(void) const {
    unsigned int period = 1;  // 1 us
    switch (dataRate) {
      case DataRate::_1Mbps:
        period = 1 * 8;
        break;
      case DataRate::_2Mbps:
        period = 0.5 * 8;
        break;
      case DataRate::_250kbps:
        period = 4 * 8;
        break;
      default:
        break;
    }
    return period;
  }

  unsigned int packetDuration(void) const { return packetSize() * bitPeriod(); }

  friend std::ostream &operator<<(std::ostream &os, const RadioPacket &rhs) {
    os << "<Nrf24 Radio Packet>\n";

    switch (rhs.dataRate) {
      case DataRate::_1Mbps:
        os << "\tData Rate: 1Mbps\n";
        break;
      case DataRate::_2Mbps:
        os << "\tData Rate: 2Mbps\n";
        break;
      case DataRate::_250kbps:
        os << "\tData Rate: 250kbps\n";
        break;
      default:
        break;
    }

    switch (rhs.outputPower) {
      case OutputPower::_n18dBm:
        os << "\tOutput Power: -18dBm\n";
        break;
      case OutputPower::_n12dBm:
        os << "\tOutput Power: -12dBm\n";
        break;
      case OutputPower::_n6dBm:
        os << "\tOutput Power: -6dBm\n";
        break;
      case OutputPower::_0dBm:
        os << "\tOutput Power: 0dBm\n";
        break;
      default:
        break;
    }

    os << "\tChannel Number: " << rhs.channelNumber << "\n";
    os << "\tPreamble: " << fmt::format("0b{:08b}", rhs.PREAMBLE) << "\n";
    os << "\tAddress: ";
    for (int i = 0; i < rhs.addressSize; i++) {
      os << fmt::format("{:02x}", rhs.address[i]);
    }
    os << "\n";
    os << "\tPayload: ";
    for (int i = 0; i < rhs.payloadSize; i++) {
      os << fmt::format("{:02x}", rhs.payload[i]);
    }
    os << "\n";
    os << "\tCRC: ";
    for (int i = 0; i < rhs.crcSize; i++) {
      os << fmt::format("{:02x}", rhs.crc[i]);
    }
    os << "\n";
    os << "\tPacket Duration: " << rhs.packetDuration() << "\n";

    return os;
  }
};

class Nrf24Radio : public SpiDevice {
  SC_HAS_PROCESS(Nrf24Radio);

 public:
  /* ------ Ports ------ */
  sc_core::sc_out_resolved interruptRequest{"interruptRequest"};
  sc_core::sc_in_resolved chipEnable{"chipEnable"};

  /* ------ Public Types ------ */
  enum class OpModes {
    UNDEFINED,
    POWER_ON_RESET,
    POWER_DOWN,
    OSC_STARTUP,
    STANDBY1,
    STANDBY2,
    TX_SETTLING,
    TX_MODE,
    RX_SETTLING,
    RX_MODE
  };

  enum class PayloadType { COMMAND, DATA };

  /* ------ State Machine ------ */
  OpModes m_radio_state{OpModes::UNDEFINED};
  PayloadType m_payloadType{PayloadType::COMMAND};

  /* ----- Public Members ------ */
  RadioFifo m_rxFifo;
  RadioFifo m_txFifo;
  RadioPacket m_txPacket;

  //! Constructor
  Nrf24Radio(const sc_core::sc_module_name nm);

  /**
   * @brief end_of_elaboration used to register SC_METHODs, register power
   * modelling states, and build sensitivity list.
   */
  void end_of_elaboration() override;

  /**
   * @brief reset Restore registers to power on default,
   *              also flushes the SPI shift registers.
   */
  virtual void reset(void) override;

  /**
   * @brief payloadReceivedHandler Handles tasks upon receiving SPI payload.
   */
  void payloadReceivedHandler(void);

  /**
   * @brief stateChangeHandler Manages the radio state machine.
   */
  void stateChangeHandler(void);

  /**
   * @breif chipSelectHandler Updates the content of the Slave Out registers
   * with the current value of the status register.
   */
  void chipSelectHandler(void);

  void chipEnableHandler(void);

  void nResetHandler(void);

  void txEventHandler(void);

  void irqEventHandler(void);

 private:
  sc_core::sc_event m_stateChangeEvent{"m_stateChangeEvent"};
  sc_core::sc_event m_txEvent{"m_txEvent"};
  sc_core::sc_event m_irqEvent{"m_irqEvent"};

  /* Event & state ids */
  int m_porStateId{-1};
  int m_powerDownStateId{-1};
  int m_startUpStateId{-1};
  int m_standbyOneStateId{-1};
  int m_standbyTwoStateId{-1};
  int m_rxSettlingStateId{-1};
  int m_txSettlingStateId{-1};
  int m_rxModeStateId{-1};
  int m_txModeStateId{-1};
};
