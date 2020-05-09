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
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "ps/EventLog.hpp"

class GenericDma : public BusTarget {
  SC_HAS_PROCESS(GenericDma);

 public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};  //! clock input
  sc_core::sc_out<bool> irq{"irq"};  //! Interrupt request output
  sc_core::sc_in<bool> ira{"ira"};   //! Interrupt request accepted
  tlm::tlm_initiator_socket<> iSocket{"iSocket"};    //! Outgoing socket
  std::array<sc_core::sc_signal<bool>, 30> trigger;  //! External triggers

  /*------ Public methods ------*/

  /**
   * @brief GenericDma constructor
   * @param name
   */
  GenericDma(const sc_core::sc_module_name name, const sc_core::sc_time delay);

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

  /**
   * @brief end_of_elaboration used to register SC_METHODs and build sensitivity
   * list.
   */
  virtual void end_of_elaboration() override;

 private:
  /*------ Private types ------*/
  /*------ Submodules ------*/

  /*------ Static constants ------*/

  /*------ Private variables ------*/
  const unsigned nChannels = 6;
  std::array<DmaChannel, nChannels> m_channels;
  sc_core::sc_event m_updateEvent{"m_updateEvent"};

  /*------ Private methods ------*/
 private:
  /**
   * @brief process main control loop
   */
  void process();
}

SC_MODULE(DmaChannel) {
 public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};  //! clock input
  sc_core::sc_out<bool> interruptFlag;
  sc_core::sc_out<bool> pending;  //! Flag if a transfer is pending
  sc_core::sc_in<bool> accept;    //! Flag if transfer was accepted

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
  int size;                     //! Block-size DMAxSZ
  unsigned destinationAddress;  //! DMAxDA
  unsigned sourceAddress;       //! DMAxSA
  AutoIncrementMode destinationAutoIncrement;
  AutoIncrementMode sourceAutoIncrement;
  Bytes destinationBytes;  //! Word size
  Bytes sourceBytes;       //! Word size
  bool enable;
  bool levelSensitive;  //! 0: edge sensitive, 1: level sensitive
  bool interruptFlag;
  bool interruptEnable;
  bool abort;  //! Indicate if transfer was aborted by an NMI
  bool softwareTrigger;
  TransferMode transferMode;

  /*------ Public methods ------*/

  SC_CTOR(DmaChannel) {}

  /**
   * @brief used to set sensitivity
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief updateAddresses Update source and destination addresses according
   * to word size and auto-increment mode.
   */
  void updateAddresses();

  /**
   * @brief updateTrigger set a new trigger for the channel
   * @param e event to be sensitive to
   */
  void setTrigger(const sc_core::sc_event *e);

  /**
   * @brief updateConfig updated channel configuration from DMAxCTL value
   */
  updateConfig(const unsigned cfg);

  /*------ Private variables ------*/
  sc_core::sc_event *m_trigger;
  sc_core::sc_event m_softwareTrigger;

  int m_tSize;                //! Local copy
  int m_tSourceAddress;       //! Local copy
  int m_tDestinationAddress;  //! Local copy

  /*------  Private methods ------*/
 private:
  /**
   * @brief process main control loop
   */
  void process();

  /**
   * @brief updateState Utility for updating channel state after each transfer
   */
  void updateState();
};
