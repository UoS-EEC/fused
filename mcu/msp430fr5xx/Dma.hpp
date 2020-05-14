/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <spdlog/spdlog.h>
#include <stdint.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <array>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "ps/EventLog.hpp"

// class DmaChannel : public sc_core::sc_module {
SC_MODULE(DmaChannel) {
 public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};  //! clock input
  sc_core::sc_out<bool> pending;  //! Flag if a transfer is pending
  sc_core::sc_in<bool> accept;    //! Flag if transfer was accepted

  // Flags
  bool interruptFlag;

  // Types
  enum class AutoIncrementMode { Unchanged, Decrement, Increment };
  enum class TransferMode {
    Single,
    Block,
    BurstBlock,
    RepeatedSingle,
    RepeatedBlock,
    RepeatedBurstBlock
  };
  enum class Bytes { Byte, Word };  //! Word size

  /*------ Configuration variables ------*/
  int size{0};                     //! Block-size DMAxSZ
  unsigned destinationAddress{0};  //! DMAxDA
  unsigned sourceAddress{0};       //! DMAxSA
  AutoIncrementMode destinationAutoIncrement{AutoIncrementMode::Unchanged};
  AutoIncrementMode sourceAutoIncrement{AutoIncrementMode::Unchanged};
  Bytes destinationBytes{Bytes::Byte};  //! Word size
  Bytes sourceBytes{Bytes::Byte};       //! Word size
  bool enable{false};
  bool levelSensitive{false};  //! 0: edge sensitive, 1: level sensitive
  bool interruptEnable{false};
  bool abort{false};  //! Indicate if transfer was aborted by an NMI
  TransferMode transferMode{TransferMode::Single};

  /*------ Public methods ------*/

  SC_CTOR(DmaChannel) { SC_THREAD(process); }

  /**
   * @brief updateAddresses Update source and destination addresses according
   * to word size and auto-increment mode.
   */
  void updateAddresses();

  /**
   * @brief updateTrigger set a new trigger for the channel
   * @param e event to be sensitive to
   */
  void setTrigger(const sc_core::sc_event &e);

  /**
   * @brief updateConfig updated channel configuration from DMAxCTL value
   */
  void updateConfig(const unsigned cfg);

  /**
   * @brief reset reset state to power-on defaults
   */
  void reset();

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const DmaChannel &rhs);

  /*------ Private variables ------*/
  sc_core::sc_event_or_list m_trigger;
  sc_core::sc_event m_softwareTrigger{"m_softwareTrigger"};

  int m_tSize{0};                //! Local copy
  int m_tSourceAddress{0};       //! Local copy
  int m_tDestinationAddress{0};  //! Local copy

  /*------  Private methods ------*/
 private:
  /**
   * @brief process main control loop
   */
  void process();
};

class Dma : public BusTarget {
  SC_HAS_PROCESS(Dma);

 public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};  //! clock input
  sc_core::sc_in<bool> ira{"ira"};               //! Interrupt request accepted
  std::array<sc_core::sc_in<bool>, 30> trigger;  //! External triggers
  sc_core::sc_out<bool> irq{"irq"};              //! Interrupt request output
  sc_core::sc_out<bool> stallCpu{"stallCpu"};    // Stall CPU while transfering
  tlm_utils::simple_initiator_socket<Dma> iSocket{
      "iSocket"};  //! Outgoing socket

  /*------ Public methods ------*/

  /**
   * @brief Dma constructor
   * @param name
   */
  Dma(const sc_core::sc_module_name name, const sc_core::sc_time delay);

  /**
   * @brief reset Reset registers and member values to their power-on values.
   */
  virtual void reset(void) override;

  /**
   * @brief b_transport Blocking reads and writes
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /*------ Static constants ------*/
  static const unsigned NCHANNELS = 6;
  /*------ Private types ------*/

  /*------ Submodules ------*/
  std::array<DmaChannel *, NCHANNELS> m_channels;

 private:
  /*------ Private variables ------*/
  bool m_clearIfg;
  std::array<sc_core::sc_signal<bool>, NCHANNELS> m_channelPending;
  std::array<sc_core::sc_signal<bool>, NCHANNELS> m_channelAccept;

  sc_core::sc_event m_updateIrqEvent{"m_updateIrqEvent"};

  /*------ Private methods ------*/
  /**
   * @brief process main control loop
   */
  void process();

  /**
   * @brief updateChannelAddresses update addresses & size from register values
   * for all channels.
   */
  void updateChannelAddresses();

  /**
   * @brief interruptUpdate update interrupt state & flags
   */
  void interruptUpdate();
};
