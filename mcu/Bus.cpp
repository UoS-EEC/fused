/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
#include "mcu/Bus.hpp"
#include "mcu/BusTarget.hpp"
#include "utilities/Config.hpp"

Bus::Bus(const sc_core::sc_module_name name)
    : sc_core::sc_module(name), iSocket("iSocket"), tSocket("tSocket") {
  tSocket.register_b_transport(this, &Bus::b_transport);
  tSocket.register_transport_dbg(this, &Bus::transport_dbg);
}

void Bus::bindTarget(BusTarget &t) {
  m_routingTable.emplace_back(std::make_pair(t.startAddress(), t.endAddress()));
  iSocket.bind(t.tSocket);
  sc_assert(m_routingTable.size() == iSocket.size());
}

int Bus::routeForward(tlm::tlm_generic_payload &trans) const {
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

void Bus::b_transport([[maybe_unused]] const int id,
                      tlm::tlm_generic_payload &trans,
                      sc_core::sc_time &delay) {
  const auto addr = trans.get_address();
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
  updateTrace(trans, addr);
}

unsigned int Bus::transport_dbg([[maybe_unused]] const int id,
                                tlm::tlm_generic_payload &trans) {
  auto len = trans.get_data_length();
  auto addr = trans.get_address();
  auto port = routeForward(trans);

  if (port >= 0) {
    // Check address bounds, any size permitted
    sc_assert(inRange(addr, m_routingTable[port]));            // Start address
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

bool Bus::overlapsExistingTarget(const int startAddress,
                                 const int endAddress) const {
  const auto hit = std::find_if(
      m_routingTable.begin(), m_routingTable.end(),
      [startAddress, endAddress](std::pair<const unsigned, const unsigned> a) {
        return (a.second > startAddress) &&  // end1 > start0
               (a.first < endAddress);       // start1 < end0
      });
  return hit != m_routingTable.end();
}

void Bus::checkTransaction(const tlm::tlm_generic_payload &trans,
                           const int targetPort) const {
  const auto addr = trans.get_address();
  const auto len = trans.get_data_length();
  sc_assert((addr + len - 1) <= (m_routingTable[targetPort].second -
                                 m_routingTable[targetPort].first));
  sc_assert(addr % len == 0);          // Alignment
  sc_assert(len <= TARGET_WORD_SIZE);  // Size
}

std::ostream &operator<<(std::ostream &os, Bus &rhs) {
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

void Bus::updateTrace(const tlm::tlm_generic_payload &trans,
                      const unsigned originalAddress) {
  dataTrace = 0;
  for (int i = 0; i < sizeof(dataTrace); ++i) {
    dataTrace <<= 8;
    if (i < trans.get_data_length()) {
      dataTrace |= trans.get_data_ptr()[i];
    }
  }
  addressTrace = originalAddress;
  sizeTrace = trans.get_data_length();
}
