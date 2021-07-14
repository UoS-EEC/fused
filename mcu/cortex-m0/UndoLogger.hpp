/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include "mcu/BusTarget.hpp"
#include "utilities/Config.hpp"
#include <iostream>
#include <stdint.h>
#include <string>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <vector>

/**
 * @brief UndoLogger class. A module that optionally undo-logs writes from the
 * cache to main memory. To be instantiated between main memory and the cache,
 * i.e.:
 *
 *              bus
 *    ===========o==============o==
 *                     |        |
 *                     |        |
 *                     |        |
 *              ,------------.  |
 *              | Data cache |  |
 *              `------------'  |
 *                     |        |
 *                     |        |
 *                     |        |
 *              ,--------------------.
 *              | memSocket  tSocket |
 *              |                    |
 *              |     UndoLogger     |---irq
 *              |                    |---dmaTrigger
 *              |   iSocket          |
 *              `--------------------'
 *                     |
 *                     |
 *                     |
 *              ,-------------.
 *              | Main memory | (usually NVM)
 *              `-------------'
 *
 *
 *
 * Operation:
 * ----------
 *
 * When disabled, simply forwards writes from the cache to main memory without
 * delay.
 *
 * When enabled, creates an undo-log of writes from the cache by:
 *      1) reading the old value from main memory, then
 *      2) storing it in an undo-log entry, then
 *      3) forwarding the write from the cache to main memory.
 *
 * This log can then be applied to main memory, to undo changes that occured
 * during the enabled period. E.g. undoing writes to NVM during a failure-atomic
 * section that could not complete.
 *
 * Applying the undo log:
 * ---------
 * The log is applied (and thereby emptied) by setting a control bit (see below
 * register/bit field definitions). This bit is automatically cleared when the
 * log is empty. Applying the undo-log means writing old values to main memory,
 * thereby restoring the state that it had immediately before logging was
 * enabled.
 *
 * Unsafe Zone:
 * ------------
 * Two registers define an unsafe zone: an address region where logging is
 * disabled even when this module is enabled. This zone is intended to be used
 * by the programmer for data that is known to be free from Write-after-Read
 * hazards. Allocating such data to the unsafe zone reduces the pressure on the
 * undo log.
 *
 * Fill level interrupt:
 * ---------------------
 * If the internal log fills to/beyond a software-settable threshold, an
 * interrupt can be generated to notify the CPU, so that it can read the log
 * into (unsafe-zone) memory. Reading from the FIFO frees up space for new
 * entries. Software can then e.g. persist the log to non-volatile memory, so
 * that it can be restored from after a power failure. Alternatively, the log
 * can be kept in volatile memory, and, in the event of a power failure, be
 * applied through the FIFO write interface as described in the next section.
 *
 * Fill level DMA trigger:
 * -----------------------
 * Similarly to the fill level interrupt, a DMA trigger output can be issued
 * when the internal log fills tothe software-settable threshold. The trigger
 * is sent as a one-cycle pulse.
 *
 * Flush:
 * ------
 *  When the internal log fills to capacity, it can flush entries to a dedicated
 * space in downstream (NVM) memory to avoid overflow.
 *
 * Writing to the FIFO:
 * --------------------
 * Logging entries can be added by writing to the FIFO. Write the address first,
 * then the data. This write mechanism is intended to be used when restoring
 * from logs that were previously read into memory by software (see above
 * section).
 *
 */

class UndoLogger : public BusTarget {
public:
  /* ------ Ports ------ */
  //! Socket bound to upstream memory (cache)
  tlm_utils::simple_target_socket<UndoLogger> memSocket{"memSocket"};

  //! Socket bound to downstream memory
  tlm_utils::simple_initiator_socket<UndoLogger> iSocket{"iSocket"};

  //! Interrupt request port
  sc_core::sc_out<bool> irq{"irq"};

  //! DMA trigger output
  sc_core::sc_out<bool> dmaTrigger{"dmaTrigger"};

  //! Signal used for retiring interrupts
  sc_core::sc_in<int> returning_exception{"returning_exception"};

  /* ------ Public Methods ------ */
  /**
   * @brief UndoLogger constructor.
   * @param name module name
   * @param memStartAddress start address of the downstream memory
   * @param ctrlStartAddress bus target start address
   * @param capacity log capacity in number of entries
   * @param cacheLineWidth line widht of preceding cache, i.e. transaction size.
   * @param exceptionId exception number this module is attached to.
   *
   */
  UndoLogger(sc_core::sc_module_name name, unsigned memStartAddress,
             unsigned ctrlStartAddress, int capacity, int cacheLineWidth,
             int exceptionId);

  /**
   * @brief reset Reset to power-on defaults.
   */
  virtual void reset() override;

  /**
   * @brief set up methods/threads after module construction complete.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief b_transport handle the bus interface to the internal registers.
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /**
   * @brief mem_b_transport handle transactions between upstream and
   * downstream memories.
   */
  void mem_b_transport(tlm::tlm_generic_payload &trans,
                       sc_core::sc_time &delay);
  /**
   * @brief mem_transport_dbg forwards debug transports directly to downstream
   * memory.
   */
  unsigned int mem_transport_dbg(tlm::tlm_generic_payload &trans);

private:
  /**
   * @brief logLine read a cache line of memory from downstream memory and log
   * its address and value.
   * @param address base address of cache line to be logged
   * @param delay cumulative access delay
   */
  void logLine(unsigned address, sc_core::sc_time &delay);

  /**
   * @brief apply SC_THREAD that applies the log to downstream memory when the
   * CTRL_FLUSH bit is set.
   */
  void apply();

  /**
   * @brief flush an undo log entry to  downstream memory.
   */
  void flush();

  /**
   * @brief size count of stored log entries.
   * @retval number of stored log entries.
   */
  int size() const;

  /**
   * @brief filter filters out addresses in the unsafe zone.
   * @param address address to filter
   * @retval false if address belongs to unsafe zone, true otherwise.
   */
  bool filter(unsigned address) const;

  /**
   */
  void updateThresholdFlag();

  /**
   * @brief irqControl Control the interrupt request signal (irq)
   */
  void irqControl();

  /**
   * @brief dmaTriggerControl Control the dma trigger signal
   */
  void dmaTriggerControl();

  /**
   * @brief pushLogWord utility method for pushing a 32-bit word to the undo
   * log.
   * @param val value to push
   */
  void pushLogWord(const unsigned val);

  /**
   * @brief popLogWord utility method for popping a 32-bit word off the undo
   * log.
   * @retval value popped
   */
  unsigned popLogWord();

  /**
   * @brief updateStatusRegister utility method for updating the contents of the
   * status register as well as the internal interrupt request flag.
   */
  void updateStatusRegister();

public:
  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const UndoLogger &rhs);

  /* ------ Public constants ------ */

  // Register addresses
  // Data registers
  struct RegisterAddress {
    // clang-format off

    // General control register
    // Bits:
    //   0:     ENABLE, Enable logging
    //   1:     CLEAR, Clear all FIFO entries
    //   2:     APPLY, apply all valid entries to memory
    //   3:     IE, Interrupt Enable
    //   4:     DMAEN, DMA trigger output Enable
    //   5:     FLUSHEN, Enable autmatic flushing
    //   6..31: undefined
    static const unsigned CTRL =                0x00;

    // Status register
    // Fields:
    //   0:      EMPTY, indicate if log is empty
    //   1:      OVERFLOW, indicates overflowed log, clears when FIFO emptied
    //   2:      THRESHOLD, indicates if number of entries >= FIFO_THR
    //   3-10:   FREESLOTS, Remaining capacity in number of entries
    //   11-8:   CAPACITY, Total capacity, in number of entries
    //   12..31: undefined
    static const unsigned STATUS =              0x04;

    // Undo log entry register. Note that this is a virtual register where
    // accesses are actually forwarded to the internal FIFO. If the FIFO is
    // empty, a read from this register returns 0, and a warning is issued on
    // the console.
    static const unsigned FIFO =                0x08;


    // FIFO threshold register. Sets the FIFO threshold in number of entries.
    // If CTRL_IE is set, an interrupt pulse is generated when FIFO is filled
    // to or beyond the FIFO threshold.
    static const unsigned FIFO_THR =            0x0c;

    // Unsafe-zone absolute base address (i.e. bus base address)
    // Writes to addresses in the unsafe zone are excluded from the undo-log
    // even when logging is enabled.
    static const unsigned UNSAFE_BASE =         0x10;

    // Unsafe-zone size
    static const unsigned UNSAFE_SIZE =         0x14;

    // Flush base address
    static const unsigned FLUSH_BASE =          0x18;

    // Flush size, i.e. how many entries can be stored in NVM
    static const unsigned FLUSH_SIZE =          0x1c;

    // clang-format on
  };

  // Bit masks/patterns
  struct BitMasks {
    // clang-format off

    // ------ CTRL Settings ------
    //! Enable logging
    static const uint32_t CTRL_ENABLE =                   (1u << 0);

    //! Clear log
    static const uint32_t CTRL_CLEAR =                    (1u << 1);

    //! Apply all entries to memory
    static const uint32_t CTRL_APPLY =                    (1u << 2);

    //! Enable interrupt output.
    static const uint32_t CTRL_IE =                       (1u << 3);

    //! Enable DMA trigger output.
    static const uint32_t CTRL_DMAEN =                    (1u << 4);

    //! Enable flushing
    static const uint32_t CTRL_FLUSHEN =                  (1u << 5);

    // ------ STATUS Fields ------
    //! Indicate whether the log is empty
    static const uint32_t STATUS_EMPTY_SHIFT =            0;
    static const uint32_t STATUS_EMPTY_MASK =             (1u << 0);

    //! Indicate log overflow
    static const uint32_t STATUS_OVERFLOW_SHIFT =         1;
    static const uint32_t STATUS_OVERFLOW_MASK =          (1u << 1);

    //! Indicate number of log entries >= FIFO_THR
    static const uint32_t STATUS_THRESHOLD_SHIFT =        2;
    static const uint32_t STATUS_THRESHOLD_MASK =         (1u << 2);

    //! Indicate remaining capacity, given in number of entries
    static const uint32_t STATUS_FREESLOTS_SHIFT =        3;
    static const uint32_t STATUS_FREESLOTS_MASK =         (0b1111111u << 3);

    //! Total capacity, given in number of entries
    static const uint32_t STATUS_CAPACITY_SHIFT =        11;
    static const uint32_t STATUS_CAPACITY_MASK =         (0b1111111u << 11);

    // clang-format on
  };

public:
  /* ------ Public constants ------ */
  //! Start address of downstream memory, used here to adjust UNSAFE_BASE
  const int m_memStartAddress;

  //! Cache line width in bytes
  const int m_cacheLineWidth;

  //! Capacity in number of log entries
  const int m_capacity;

  //! Exception ID
  const int m_exceptionId;

private:
  /*------ Private variables ------*/
  std::deque<uint8_t> m_log; //! Undo-log

  bool m_setIrq{false};        //! Flag to control irq signal
  bool m_setDmaTrigger{false}; //! Flag to control dmaTrigger signal
  bool m_enable{false};        //! Indicate whether or not logging is enabled
  bool m_overflow{false};      //! Indicate that an overflow has occurred
  bool m_applying{false};      //! Indicate when a apply operation is ongoing

  //! Track how many entries have been flushed to NVM
  int m_nFlushedEntries{0};

  //! Events & States for power modelling
  int m_readLogByteEventId{-1};  //! Counts bytes read from internal log
  int m_writeLogByteEventId{-1}; //! Counts bytes written to internal log

  int m_offStateId{-1};
  int m_disabledStateId{-1};
  int m_enabledStateId{-1};

  //! Event to signal that irq should be updated
  sc_core::sc_event m_updateIrqEvent{"updateIrqEvent"};

  //! Event to signal that dmaTrigger should be updated
  sc_core::sc_event m_updateDmaTriggerEvent{"updateDmaTriggerEvent"};

  //! Event to signal that a new item has been pushed to the log
  sc_core::sc_event m_logEvent{"m_logEvent"};

  /* ------ Private methods ------ */
  /**
   * @brief process main process loop
   */
  void process();
};
