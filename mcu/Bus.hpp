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
  tlm_utils::multi_passthrough_initiator_socket<Bus> iSocket;
  tlm_utils::multi_passthrough_target_socket<Bus> tSocket;

  explicit Bus(const sc_core::sc_module_name name)
      : sc_core::sc_module(name), iSocket("iSocket"), tSocket("tSocket") {
    tSocket.register_b_transport(this, &Bus::b_transport);
    tSocket.register_transport_dbg(this, &Bus::transport_dbg);
  }

  /**
   * @brief bindTarget bind a new bus target and add its start & end addresses
   * to the routing table.
   * @param t target
   */
  void bindTarget(BusTarget &t) {
    m_routingTable.emplace_back(
        std::make_pair(t.startAddress(), t.endAddress()));
    iSocket.bind(t.tSocket);
    sc_assert(m_routingTable.size() == iSocket.size());
  }

  /**
   * @brief routeForward Find outgoing port of a transaction, and adjust the
   * transaction's address by subtracting the target's start address.
   * @param trans transaction object
   * @retval the outgoing port number, returns -1 if the target was not found.
   */
  int routeForward(tlm::tlm_generic_payload &trans) const {
    auto addr = trans.get_address();
    auto it = std::find_if(
        m_routingTable.begin(), m_routingTable.end(),
        [addr, this](const std::pair<const unsigned, const unsigned> &rt) {
          return this->inRange(addr, rt);
        });
    if (it == m_routingTable.end()) {
      return -1;
    }
    trans.set_address(addr - it->first);
    return it - m_routingTable.begin();
  }

  /**
   * @brief b_transport blocking bus transaction. Adjust address and forward
   * transaction to target.
   */
  void b_transport([[maybe_unused]] const int id,
                   tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
    auto port = routeForward(trans);
    if (port == -1) {
      std::stringstream s;
      s << *this;
      SC_REPORT_FATAL(
          this->name(),
          fmt::format("{:s}:routeForward Target for transaction with address "
                      "0x{:08x} not found.\n{:s}",
                      this->name(), trans.get_address(), s.str())
              .c_str());
    }
    checkTransaction(trans, port);
    iSocket[port]->b_transport(trans, delay);
  }

  /**
   * @brief transport_dbg Transport without timing
   */
  unsigned int transport_dbg([[maybe_unused]] const int id,
                             tlm::tlm_generic_payload &trans) {
    auto len = trans.get_data_length();
    auto addr = trans.get_address();
    auto port = routeForward(trans);

    if (port >= 0) {
      // Check address bounds, any size permitted
      sc_assert(inRange(addr, m_routingTable[port]));  // Start address
      sc_assert(inRange(addr + len - 1, m_routingTable[port]));  // End address
      return iSocket[port]->transport_dbg(trans);
    } else {
      std::stringstream s;
      s << *this;
      spdlog::warn(
          "{:s}::transport_dbg: Target for transaction with address "
          "0x{:08x} not found, returning 0.\n{:s}",
          this->name(), addr, s.str());
      std::memset(trans.get_data_ptr(), 0, len);
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
      return len;
    }
  }

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
                              const int endAddress) const {
    const auto hit =
        std::find_if(m_routingTable.begin(), m_routingTable.end(),
                     [startAddress,
                      endAddress](std::pair<const unsigned, const unsigned> a) {
                       return (a.second > startAddress) &&  // end1 > start0
                              (a.first < endAddress);       // start1 < end0
                     });
    return hit != m_routingTable.end();
  }

  /**
   * @brief checkTransaction Check the that a transaction's address and length
   * conforms to its target. Issue errors otherwise.
   * @param trans transaction
   * @param targetPort target port number
   */
  void checkTransaction(const tlm::tlm_generic_payload &trans,
                        const int targetPort) const {
    const auto addr = trans.get_address();
    const auto len = trans.get_data_length();
    sc_assert((addr + len - 1) < (m_routingTable[targetPort].second -
                                  m_routingTable[targetPort].first));
    sc_assert(addr % len == 0);          // Alignment
    sc_assert(len <= TARGET_WORD_SIZE);  // Size
  }

  /**
   * @brief << debug printout.
   * @note the rhs reference should be const, but SystemC prevents this because
   * the size() method used below is not declared as const (by the SystemC
   * standard).
   */
  friend std::ostream &operator<<(std::ostream &os, Bus &rhs) {
    os << "<Bus> " << rhs.name() << "\n";
    os << "Initiators: " << rhs.tSocket.size() << "\n";
    os << "Targets: " << rhs.iSocket.size() << "\n";
    os << "Memory map:\n"
       << "Port    address(start)    address(end)\n";
    for (unsigned int i = 0; i < rhs.m_routingTable.size(); i++) {
      os << fmt::format("{: <8d}0x{:08x}        0x{:08x}\n", i,
                        rhs.m_routingTable[i].first,
                        rhs.m_routingTable[i].second);
    }
    return os;
  }
};
