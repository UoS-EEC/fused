/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>
#include <systemc>
#include "utilities/Config.hpp"

using namespace sc_core;

/**
 * @brief The InterruptArbiter class Arbitrate between interrupt requests
 * @param N Total number of interrupt vectors
 */
template <unsigned N>
class InterruptArbiter : public sc_module {
 public:
  sc_port<sc_signal_in_if<bool>, 1, SC_ZERO_OR_MORE_BOUND> irqIn[N];
  sc_port<sc_signal_write_if<bool>, 1, SC_ZERO_OR_MORE_BOUND> iraOut[N];
  sc_out<bool> irqOut{"IrqOut"};
  sc_in<bool> iraIn{"IrqIn"};
  sc_out<unsigned> idxOut{"idxOut"};
  sc_out<bool> iraConnected{"iraConnected"};

  SC_HAS_PROCESS(InterruptArbiter);

  explicit InterruptArbiter(sc_module_name nm, bool doVcd = false)
      : sc_module(nm), m_doVcd(doVcd) {
    // Main process loop -- sensitive to all IRQ inputs as well as CPUs IRA
    SC_METHOD(process);
    sensitive << iraIn;
    for (unsigned int i = 0; i < N; i++) {
      sensitive << irqIn[i];
    }

    if (m_doVcd) {
      m_wf = sc_create_vcd_trace_file(
          (Config::get().getString("OutputDirectory") + "/" + this->name())
              .c_str());
      sc_trace(m_wf, irqOut, "irqOut");
      sc_trace(m_wf, iraIn, "iraIn");
      sc_trace(m_wf, idxOut, "idxOut");
    }
  }

 private:
  /* ------ Internal signals ------ */

  /* ------ Private variables ------ */
  bool m_doVcd;
  sc_trace_file *m_wf;

  /* ------ Private methods ------ */

  /**
   * @brief process Forwards the highest priority interrupt and its
   * associated index to the cpu, and forwards the interrupt accepted signal
   * from the cpu to the relevant peripheral.
   */
  void process() {
    // Forward the highest priority interrupt to the cpu
    if (iraIn.read()) {  // Wait for currently processed interrupt
      uint16_t crntIdx = idxOut->read();

      // Forward IRA signal if bound
      if (iraOut[crntIdx].bind_count() > 0) {
        iraOut[crntIdx]->write(true);
      }

      // Forward IRQ signal to CPU
      irqOut.write(irqIn[crntIdx]->read());
    } else {  // Process next interrupt
      if (iraOut[idxOut->read()].bind_count() > 0) {
        iraOut[idxOut->read()]->write(false);  // reset ira line
      }
      bool pending = false;
      for (unsigned i = 0; i < N; i++) {
        if (irqIn[i].bind_count() > 0) {
          if (irqIn[i]->read()) {
            idxOut->write(i);
            pending = true;
            iraConnected.write(iraOut[idxOut->read()].bind_count() > 0);
          }
        }
      }
      irqOut.write(pending);
    }
  }
};
