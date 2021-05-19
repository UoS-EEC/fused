/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * DESCRIPTION:
 *
 * Simple multi-channel DMA module, based on the one from MSP430FR5994
 * (mcu/msp430fr5xx/Dma.{cpp,hpp}), but adapted to fit the CortexM0.
 *
 */

#pragma once

#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include <array>
#include <spdlog/spdlog.h>
#include <stdint.h>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>

namespace CortexM0Peripherals {

// Trigger multiplexer for one DMA channel
SC_MODULE(TriggerMux) {
public:
  sc_core::sc_in<int> select;
  std::array<sc_core::sc_in<bool>, 30> in;
  sc_core::sc_in<bool> nreset{"nreset"};
  sc_core::sc_out<bool> out{"out"};

  SC_CTOR(TriggerMux) {}

  virtual void end_of_elaboration() override {
    SC_METHOD(process);
    sensitive << select.value_changed_event() << in[0].value_changed_event()
              << nreset.negedge_event();
    dont_initialize();
  }

  void process() {
    if (!nreset.read()) {
      out.write(false);
      next_trigger(nreset.posedge_event());
    } else {
      const auto s = select.read();
      out.write(in[s].read());
      next_trigger(in[s].value_changed_event() | select.value_changed_event() |
                   nreset.negedge_event());
    }
  }
};

// DMA  channel
SC_MODULE(DmaChannel) {
public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceConsumerIf> systemClk{
      "systemClk"};              //! clock input
  sc_core::sc_out<bool> pending; //! Flag if a transfer is pending
  sc_core::sc_in<bool> accept;   //! Flag if transfer was accepted
  sc_core::sc_in<bool> trigger;  //! Transfer trigger

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
  enum class Bytes { Byte, Word }; //! Word size

  /*------ Configuration variables ------*/
  int size{0};                    //! Block-size DMAxSZ
  unsigned destinationAddress{0}; //! DMAxDA
  unsigned sourceAddress{0};      //! DMAxSA
  AutoIncrementMode destinationAutoIncrement{AutoIncrementMode::Unchanged};
  AutoIncrementMode sourceAutoIncrement{AutoIncrementMode::Unchanged};
  Bytes destinationBytes{Bytes::Byte}; //! Word size
  Bytes sourceBytes{Bytes::Byte};      //! Word size
  bool enable{false};
  bool levelSensitive{false}; //! 0: edge sensitive, 1: level sensitive
  bool interruptEnable{false};
  bool abort{false}; //! Indicate if transfer was aborted by an NMI
  TransferMode transferMode{TransferMode::Single};

  /*------ Public methods ------*/

  SC_CTOR(DmaChannel) { SC_THREAD(process); }

  /**
   * @brief updateAddresses Update source and destination addresses according
   * to word size and auto-increment mode.
   */
  void updateAddresses();

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
  sc_core::sc_event m_softwareTrigger{"m_softwareTrigger"};

  int m_tSize{0};               //! Local copy
  int m_tSourceAddress{0};      //! Local copy
  int m_tDestinationAddress{0}; //! Local copy

  /*------  Private methods ------*/
private:
  /**
   * @brief process main control loop
   */
  void process();
};

// DMA module -- implements the register file & orchestrates channels
class Dma : public BusTarget {
  SC_HAS_PROCESS(Dma);

public:
  /*------ Ports ------*/
  //! Bus initiator socket
  tlm_utils::simple_initiator_socket<Dma> iSocket{"iSocket"};

  //! When high, this signal stalls other bus initiators
  sc_core::sc_out<bool> busStall{"busStall"};

  // Interrupt ports
  sc_core::sc_out<bool> irq{"irq"}; //! Interrupt request output
  sc_core::sc_in<int> active_exception{
      "active_exception"}; //! Signals exception taken by cpu

  //! External DMA triggers
  std::array<sc_core::sc_in<bool>, 30> trigger;

  /* ------ Public constants ------ */

  // Register addresses
  // Data registers
  struct RegisterAddress;

  // Bit masks/patterns/shifts
  struct BitMask;

  /*------ Public methods ------*/

  /**
   * @brief Dma constructor
   * @param name
   * @param startAddress start address of control registers
   */
  Dma(const sc_core::sc_module_name name, const unsigned startAddress);

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

  /*------ Submodules ------*/
  static const int NCHANNELS = 6; //! Number of DMA channels
  std::array<DmaChannel *, NCHANNELS> m_channels;
  std::array<TriggerMux *, NCHANNELS> m_triggerMuxes;

  /*------ Private types ------*/

private:
  /*------ Private variables ------*/
  bool m_setIrq{false}; //! 1 if irq should be set
  bool m_clearIfg;
  std::array<sc_core::sc_signal<bool>, NCHANNELS> m_channelPending;
  std::array<sc_core::sc_signal<bool>, NCHANNELS> m_channelAccept;
  std::array<sc_core::sc_signal<bool>, NCHANNELS> m_channelTrigger;
  std::array<sc_core::sc_signal<int>, NCHANNELS> m_channelTriggerSelect;

  sc_core::sc_event m_updateIrqEvent{"m_updateIrqEvent"};
  sc_core::sc_event m_updateIrqFlagEvent{"m_updateIrqFlagEvent"};

  /*------ Private methods ------*/
  /**
   * @brief process main control loop
   */
  void process();

  /**
   * @brief updateChannelAddresses update addresses & size from register
   * values for all channels.
   */
  void updateChannelAddresses();

  /**
   * @brief interruptUpdate update interrupt state & flags
   */
  void interruptUpdate();

  /**
   * @brief irqControl control irq signalling (sets & clears the actual irq
   * signal)
   */
  void irqControl();
};

struct Dma::RegisterAddress {
  // See the MSP430fr5xxx TRM  for register descriptions
  //
  // Register addresses have been modified to make each register 32 bit,
  // instead of the MSP430's 16-bit registers

  // clang-format off

  // DMA-wide control registers
  static const unsigned DMACTL0            = 0x0000;
  static const unsigned DMACTL1            = 0x0004;
  static const unsigned DMACTL2            = 0x0008;
  static const unsigned DMACTL4            = 0x000C;

  // Interrupt vector
  static const unsigned DMAIV              = 0x000E;

  // Channel 0
  static const unsigned DMA0CTL            = 0x0010;
  static const unsigned DMA0SA             = 0x0014;
  static const unsigned DMA0DA             = 0x0018;
  static const unsigned DMA0SZ             = 0x001C;

  // Channel 1
  static const unsigned DMA1CTL            = 0x0020;
  static const unsigned DMA1SA             = 0x0024;
  static const unsigned DMA1DA             = 0x0028;
  static const unsigned DMA1SZ             = 0x002C;

  // Channel 2
  static const unsigned DMA2CTL            = 0x0030;
  static const unsigned DMA2SA             = 0x0034;
  static const unsigned DMA2DA             = 0x0038;
  static const unsigned DMA2SZ             = 0x003C;

  // Channel 3
  static const unsigned DMA3CTL            = 0x0040;
  static const unsigned DMA3SA             = 0x0044;
  static const unsigned DMA3DA             = 0x0048;
  static const unsigned DMA3SZ             = 0x004C;

  // Channel 4
  static const unsigned DMA4CTL            = 0x0050;
  static const unsigned DMA4SA             = 0x0054;
  static const unsigned DMA4DA             = 0x0058;
  static const unsigned DMA4SZ             = 0x005C;

  // Channel 5
  static const unsigned DMA5CTL            = 0x0060;
  static const unsigned DMA5SA             = 0x0064;
  static const unsigned DMA5DA             = 0x0068;
  static const unsigned DMA5SZ             = 0x006C;
  // clang-format on
};

struct Dma::BitMask {
  // See the MSP430fr5xxx TRM  for register descriptions
  static const unsigned DMAREQ = 0x0001;     /* DMA abort */
  static const unsigned DMAABORT = 0x0002;   /* DMA abort */
  static const unsigned DMAIE = 0x0004;      /* DMA interrupt enable */
  static const unsigned DMAIFG = 0x0008;     /* DMA interrupt flag */
  static const unsigned DMAEN = 0x0010;      /* DMA enable */
  static const unsigned DMALEVEL = 0x0020;   /* DMA level */
  static const unsigned DMASRCBYTE = 0x0040; /* DMA source byte */
  static const unsigned DMADSTBYTE = 0x0080; /* DMA destination byte */
  static const unsigned DMASRCINCR = 0x0300; /* DMA source increment */
  static const unsigned DMADSTINCR = 0x0c00; /* DMA destination increment */
  static const unsigned DMADT = 0x7000;      /* DMA transfer mode */
};
} // namespace CortexM0Peripherals
