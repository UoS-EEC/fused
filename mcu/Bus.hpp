/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <spdlog/spdlog.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <string>
#include <systemc>
#include <tlm>
#include <tuple>
#include <vector>
#include "mcu/BusTarget.hpp"
#include "utilities/Config.hpp"

class Bus : sc_core::sc_module,
            tlm::tlm_bw_transport_if<>,
            tlm::tlm_fw_transport_if<> {
 public:
  std::vector<tlm::tlm_target_socket<> *> tSockets;
  std::vector<tlm::tlm_initiator_socket<> *> iSockets;

  explicit Bus(const sc_core::sc_module_name name) : sc_core::sc_module(name) {}

  /**
   * @brief addInitiator Add an initiator (master) socket and return its port
   * number.
   * @retval port number of the new initiator socket.
   */
  int addInitiator() {
    tSockets.push_back(new tlm::tlm_target_socket<>());
    tSockets.back()->bind(*this);
    return tSockets.size() - 1;
  }

  /**
   * @brief addInitiator Add a target (slave) socket and return its port
   * number.
   * @param startAddress start address of new target
   * @param endAddress end address of new target
   * @retval port number of the new target socket.
   */
  int addTarget(const int startAddress, const int endAddress) {
    checkOverlapExistingTarget(startAddress, endAddress);
    m_routingTable.push_back(
        std::tuple<const unsigned, const unsigned>(startAddress, endAddress));
    iSockets.push_back(new tlm::tlm_initiator_socket<>());
    iSockets.back()->bind(*this);
    return iSockets.size() - 1;
  }

  /**
   * @brief addInitiator Add a target (slave) socket and return its port
   * number.
   * @param BusTarget new target.
   * @retval port number of the new target socket.
   */
  int addTarget(const BusTarget &target) {
    checkOverlapExistingTarget(target.startAddress(), target.endAddress());
    auto portNumber = m_routingTable.size();
    m_routingTable.push_back(std::tuple<const unsigned, const unsigned>(
        target.startAddress(), target.endAddress()));
    iSockets.push_back(new tlm::tlm_initiator_socket<>());
    iSockets[portNumber]->bind(*this);
    return portNumber;
  }

  /**
   * @brief checkTransaction Check the that a transaction's address and length
   * conforms to its target. Issue errors otherwise.
   * @param trans transaction
   * @param targetPort target port number
   */
  void checkTransaction(const tlm::tlm_generic_payload &trans,
                        int targetPort) const {
    const auto addr = trans.get_address();
    const auto len = trans.get_data_length();
    sc_assert(inRange(addr, m_routingTable[targetPort]));  // Start address
    sc_assert(
        inRange(addr + len - 1, m_routingTable[targetPort]));  // End address
    sc_assert(addr % len == 0);                                // Alignment
    sc_assert(len <= TARGET_WORD_SIZE);                        // Size
  }

  /**
   * @brief b_transport blocking bus transaction. Adjust address and forward
   * transaction to target.
   */
  void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
    auto addr = trans.get_address();

    // Address decode
    int port = -1;
    for (unsigned int i = 0; i < m_routingTable.size(); i++) {
      if (Bus::inRange(addr, m_routingTable[i])) {
        port = i;
        break;
      }
    }
    if (port == -1) {
      spdlog::error(
          "{:s}:b_transport Target for transaction with address "
          "0x{:08x} not found.",
          this->name(), addr);
      SC_REPORT_FATAL(this->name(), "Target for transaction not found");
    }

    checkTransaction(trans, port);
    trans.set_address(addr - std::get<0>(m_routingTable[port]));
    (*iSockets[port])->b_transport(trans, delay);
  }

  unsigned int transport_dbg(tlm::tlm_generic_payload &trans) {
    auto addr = trans.get_address();
    auto len = trans.get_data_length();
    // Address decode
    int port = -1;
    for (unsigned int i = 0; i < m_routingTable.size(); i++) {
      if (Bus::inRange(addr, m_routingTable[i])) {
        port = i;
        break;
      }
    }
    if (port == -1) {
      spdlog::warn(
          "{:s}::transport_dbg: Target for transaction with address "
          "0x{:08x} not found, returning 0.",
          this->name(), addr);
      spdlog::warn("Memory map:");
      for (unsigned int i = 0; i < m_routingTable.size(); i++) {
        spdlog::warn("Port {}, startAddress 0x{:08x}, endAddress 0x{:08x}", i,
                     std::get<0>(m_routingTable[i]),
                     std::get<1>(m_routingTable[i]));
      }

      // SC_REPORT_FATAL(this->name(), "Target for transaction not found");
      std::memset(trans.get_data_ptr(), 0, len);
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
      return len;
    }

    // Check address bounds
    sc_assert(inRange(addr, m_routingTable[port]));            // Start address
    sc_assert(inRange(addr + len - 1, m_routingTable[port]));  // End address
    // Note: Any size permitted

    trans.set_address(addr - std::get<0>(m_routingTable[port]));
    return (*iSockets[port])->transport_dbg(trans);
  }

 private:
  /* ------ Private variables ------ */
  /* Routing table port number = index, holds <startAddress, endAddress> */
  std::vector<std::tuple<const unsigned, const unsigned>> m_routingTable{};

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
   * @param b tuple of min and max bounds.
   * @retval true iff b<0> <= a <= b<1>
   */
  bool inRange(const unsigned a,
               const std::tuple<const unsigned, const unsigned> &b) const {
    return (a <= std::get<1>(b) && a >= std::get<0>(b));
  }

  /**
   * @brief Check whether the address range [startAddress, endAddress] overlaps
   * with an existing entry in the routing table.
   */
  bool checkOverlapExistingTarget(const int startAddress,
                                  const int endAddress) const {
    auto hit = std::find_if(
        m_routingTable.begin(), m_routingTable.end(),
        [startAddress, endAddress](std::tuple<unsigned, unsigned> a) {
          return (std::get<1>(a) > startAddress) &&  // end1 > start0
                 (std::get<0>(a) < endAddress);      // start1 < end0
        });
    bool retval = (hit != m_routingTable.end());
    if (retval) {
      spdlog::error(
          "{}: Overlapping target addresses:\n\t0x{:08x} - "
          "0x{:08x}\n\t0x{:08x} - 0x{:08x}",
          this->name(), std::get<0>(*hit), std::get<1>(*hit), startAddress,
          endAddress);
      SC_REPORT_FATAL(this->name(), "Overlapping target addresses");
    }
    return retval;
  }

  /*------ Dummy methods --------------------------------------------------*/

 public:
  // Dummy method
  [[noreturn]] virtual tlm::tlm_sync_enum nb_transport_fw(
      tlm::tlm_generic_payload &trans[[maybe_unused]],
      tlm::tlm_phase &phase[[maybe_unused]],
      sc_core::sc_time &delay[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "nb_transport_fw is not implemented");
    exit(1);
  }

  // Dummy method
  [[noreturn]] bool get_direct_mem_ptr(
      tlm::tlm_generic_payload &trans[[maybe_unused]],
      tlm::tlm_dmi &dmi_data[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "get_direct_mem_ptr is not implemented");
    exit(1);
  }

      // Dummy method:
      [[noreturn]] void invalidate_direct_mem_ptr(
          sc_dt::uint64 start_range[[maybe_unused]],
          sc_dt::uint64 end_range[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "invalidate_direct_mem_ptr not implement");
    exit(1);
  }

  // Dummy method:
  [[noreturn]] tlm::tlm_sync_enum nb_transport_bw(
      tlm::tlm_generic_payload &trans[[maybe_unused]],
      tlm::tlm_phase &phase[[maybe_unused]],
      sc_core::sc_time &delay[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "nb_transport_bw is not implemented");
    exit(1);
  }
};
