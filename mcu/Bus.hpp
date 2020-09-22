/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <spdlog/spdlog.h>
#include <tlm_utils/multi_passthrough_initiator_socket.h>
#include <tlm_utils/multi_passthrough_target_socket.h>
#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include <systemc>
#include <tlm>
#include <utility>
#include <vector>
#include "mcu/BusTarget.hpp"
#include "utilities/Config.hpp"

class Bus : sc_core::sc_module {
 public:
  /* ------ Ports ------ */
  tlm_utils::multi_passthrough_initiator_socket<Bus> iSocket;
  tlm_utils::multi_passthrough_target_socket<Bus> tSocket;

  /* ------ Public methods ------ */
  explicit Bus(const sc_core::sc_module_name name);

  /**
   * @brief bindTarget bind a new bus target and add its start & end addresses
   * to the routing table.
   * @param t target
   */
  void bindTarget(BusTarget &t);

  /**
   * @brief routeForward Find outgoing port of a transaction, and adjust the
   * transaction's address by subtracting the target's start address.
   * @param trans transaction object
   * @retval the outgoing port number, returns -1 if the target was not found.
   */
  int routeForward(tlm::tlm_generic_payload &trans) const;

  /**
   * @brief b_transport blocking bus transaction. Adjust address and forward
   * transaction to target.
   */
  void b_transport([[maybe_unused]] const int id,
                   tlm::tlm_generic_payload &trans, sc_core::sc_time &delay);

  /**
   * @brief transport_dbg Transport without timing
   */
  unsigned int transport_dbg([[maybe_unused]] const int id,
                             tlm::tlm_generic_payload &trans);

  /* ------ Trace variables ------ */
 public:
  unsigned addressTrace{0xffffffff};
  unsigned sizeTrace{0};
  unsigned dataTrace{0};

 private:
  /* ------ Private variables ------ */
  /* Routing table, index is port number, holds <startAddress, endAddress> */
  std::vector<std::pair<const unsigned, const unsigned>> m_routingTable{};

  /* ------ Private methods ------ */
  /**
   * @brief Check if a is within the bounds specified by min and max
   * @param a address to check
   * @param min minimum bound
   * @param max maximum bound
   * @retval true iff min =< a <= max
   */
  bool inRange(const unsigned a, const unsigned min, const unsigned max) const {
    return (a <= max && a >= min);
  }

  /**
   * @brief Check if a is within the bounds specified by b
   * @param a address to check
   * @param b pair of min and max bounds.
   * @retval true iff b<0> <= a <= b<1>
   */
  bool inRange(const unsigned a,
               const std::pair<const unsigned, const unsigned> &b) const {
    return (a >= b.first) && (a <= b.second);
  }

  /**
   * @brief Check whether the address range [startAddress, endAddress] overlaps
   * with an existing entry in the routing table.
   * @param startAddress
   * @param endAddress
   * @retval true if startAddress or endAddress overlaps with an existing bus
   * target
   */
  bool overlapsExistingTarget(const int startAddress,
                              const int endAddress) const;

  /**
   * @brief checkTransaction Check the that a transaction's address and length
   * conforms to its target. Issue errors otherwise.
   * @param trans transaction
   * @param targetPort target port number
   */
  void checkTransaction(const tlm::tlm_generic_payload &trans,
                        const int targetPort) const;

  /**
   * @brief updateTrace update trace variables according to current transaction.
   * @param trans transaction after completion (i.e. with loaded data)
   * @param originalAddress Transaction address before decoding
   */
  void updateTrace(const tlm::tlm_generic_payload &trans,
                   const unsigned originalAddress);

  /**
   * @brief << debug printout.
   * @note the rhs reference should be const, but SystemC prevents this because
   * the size() method used below is not declared as const (by the SystemC
   * standard).
   */
  friend std::ostream &operator<<(std::ostream &os, Bus &rhs);
};
