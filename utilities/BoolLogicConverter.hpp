/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <systemc>

namespace Utility {
SC_MODULE(BoolInResolvedOut) {
  sc_core::sc_in<bool> in{"in"};
  sc_core::sc_out_resolved out{"out"};

  SC_CTOR(BoolInResolvedOut) {
    SC_METHOD(process);
    sensitive << in;
  }

  void process() { out.write(sc_dt::sc_logic(in.read())); }
};

SC_MODULE(ResolvedInBoolOut) {
  sc_core::sc_out<bool> out{"out"};
  sc_core::sc_in_resolved in{"in"};

  SC_CTOR(ResolvedInBoolOut) {
    SC_METHOD(process);
    sensitive << in;
  }

  // Everything else than  '1' gets written as false
  void process() { out.write(in.read().is_01() ? in.read().to_bool() : false); }
};
}  // namespace Utility
