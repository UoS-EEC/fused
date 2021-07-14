/*
 * Copyright (c) 2018-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#include "include/fused.h"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/NonvolatileMemory.hpp"
#include "mcu/cortex-m0/UndoLogger.hpp"
#include "ps/PowerModelChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"
#include <algorithm>
#include <array>
#include <random>
#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
public:
  // Signals
  sc_signal<bool> nReset{"nReset", false};
  sc_signal<bool> dmaTrigger{"dmaTrigger", false};
  sc_signal<bool> irq{"irq", false};
  sc_signal<unsigned> waitStates{"waitStates", 1};
  sc_signal<int> returning_exception{"returning_exception", 1};
  tlm_utils::simple_initiator_socket<dut> busSocket{"busSocket"};
  tlm_utils::simple_initiator_socket<dut> memSocket{"memSocket"};
  ClockSourceChannel clk{"clk", sc_time(1, SC_NS)};
  PowerModelChannel powerModelChannel{"powerModelChannel", "/tmp",
                                      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(nReset);
    m_dut.powerModelPort.bind(powerModelChannel);
    m_dut.systemClk.bind(clk);
    m_dut.tSocket.bind(busSocket);
    m_dut.memSocket.bind(memSocket);
    m_dut.iSocket.bind(nvm.tSocket);
    m_dut.irq.bind(irq);
    m_dut.returning_exception.bind(returning_exception);
    m_dut.dmaTrigger.bind(dmaTrigger);

    nvm.pwrOn.bind(nReset);
    nvm.systemClk.bind(clk);
    nvm.powerModelPort.bind(powerModelChannel);
    nvm.waitStates.bind(waitStates);
  }

  void reset() {
    nReset.write(false);
    wait(sc_time(10, SC_US));
    std::clog << "------ Reset ------\n";
    nReset.write(true);
  }

  // Parameters
  const int cacheLineWidth = 8;
  const int capacity = 32;
  const int exceptionId = 0;

  UndoLogger m_dut{"dut",
                   /*memStartAddress=*/0,
                   /*ctrlStartAddress=*/0,
                   /*capacity=*/capacity,
                   /*cacheLineWidth=*/cacheLineWidth,
                   /*exceptionId=*/exceptionId};
  NonvolatileMemory nvm{"nvm", 0, 64 * 1024 - 1};
};

SC_MODULE(tester) {
public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  struct UndoLogEntry {
    unsigned address;
    std::vector<uint8_t> data;

    UndoLogEntry(std::vector<uint8_t> &bytes, const int lineWidth) {
      address = Utility::packBytes(&bytes[0], 4);

      for (int i = 0; i < lineWidth; ++i) {
        data.push_back(bytes[4 + i]);
      }
    }

    friend std::ostream &operator<<(std::ostream &os, const UndoLogEntry &rhs) {
      os << fmt::format("@0x{:08x}: [", rhs.address);

      for (int i = 0; i < rhs.data.size(); ++i) {
        os << fmt::format("0x{:02x}", rhs.data[i]);
        if (i < rhs.data.size() - 1) {
          os << ", ";
        }
        if (i + 1 % 8 == 0) {
          os << "\n";
        }
      }
      os << "]\n";
      return os;
    }
  };

  void runtests() {
    test.reset();
    wait(SC_ZERO_TIME);

    std::vector<uint8_t> writeData(test.cacheLineWidth, 0x55);
    std::vector<uint8_t> readData(test.cacheLineWidth, 0);

    const unsigned undoLogSize = 1024;
    const unsigned undoLogBaseAddress = test.nvm.size() - undoLogSize;
    spdlog::info("undoLogBaseAddress = 0x{:08x}", undoLogBaseAddress);

    // ------ Some useful lambdas ------

    auto getFreeSlots = [=]() -> int {
      return readWord(test.busSocket, UndoLogger::RegisterAddress::STATUS) >>
             UndoLogger::BitMasks::STATUS_FREESLOTS_SHIFT;
    };

    // ------ TEST: By default, accesses go right through
    spdlog::info("------ TEST: By default, accesses go right through");

    // Fill up memory with 5's
    for (int addr = 0; addr < test.nvm.size(); addr += test.cacheLineWidth) {
      writeLine(test.memSocket, addr, writeData);
    }

    // Check that log is empty
    std::clog << test.m_dut << "\n";
    sc_assert(readWord(test.busSocket, UndoLogger::RegisterAddress::STATUS) &
              UndoLogger::BitMasks::STATUS_EMPTY_MASK);

    // Read them back
    for (int addr = 0; addr < test.nvm.size(); addr += test.cacheLineWidth) {
      readLine(test.memSocket, addr, readData);
      for (int i = 0; i < test.cacheLineWidth; ++i) {
        sc_assert(readData[i + 1] = 0x55);
      }
    }

    // ----- TEST: Log some entries
    spdlog::info("------ TEST: Log some entries");

    // Enable logging
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL,
              UndoLogger::BitMasks::CTRL_ENABLE);

    // write some A's
    std::fill(writeData.begin(), writeData.end(), 0xAA);
    const auto nwrite = test.capacity / 2;
    for (int addr = 0; addr < nwrite * test.cacheLineWidth;
         addr += test.cacheLineWidth) {
      writeLine(test.memSocket, addr, writeData);
    }

    // Check that they went through to memory
    for (int addr = 0; addr < test.nvm.size(); addr += test.cacheLineWidth) {
      readLine(test.memSocket, addr, readData);
      for (int i = 0; i < test.cacheLineWidth; ++i) {
        sc_assert(readData[i + 1] = 0xAA);
      }
    }

    // Check that log size is at half capacity
    sc_assert(getFreeSlots() == test.capacity - nwrite);

    // ----- TEST: Flush log
    spdlog::info("------ TEST: Flush log to recover state");

    // Disable logging, and start flushing
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL,
              UndoLogger::BitMasks::CTRL_APPLY);

    // Wait until done
    int limit = 1000;
    while (!(readWord(test.busSocket, UndoLogger::RegisterAddress::STATUS) &
             UndoLogger::BitMasks::STATUS_EMPTY_MASK)) {
      wait(1, SC_NS);
      if (--limit == 0) {
        sc_assert(false /*timeout*/);
      }
    }

    // Check recovered state
    for (int addr = 0; addr < test.nvm.size(); addr += test.cacheLineWidth) {
      readLine(test.memSocket, addr, readData);
      for (int i = 0; i < test.cacheLineWidth; ++i) {
        sc_assert(readData[i + 1] = 0x55);
      }
    }

    // ------ TEST: FIFO threshold interrupt
    spdlog::info("------ TEST: FIFO threshold interrupt");
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL,
              UndoLogger::BitMasks::CTRL_ENABLE |
                  UndoLogger::BitMasks::CTRL_IE);
    writeWord(test.busSocket, UndoLogger::RegisterAddress::FIFO_THR, 8);

    // write some A's
    std::fill(writeData.begin(), writeData.end(), 0xAA);
    const auto nwrite2 = 8;
    for (int addr = 0; addr < nwrite2 * test.cacheLineWidth;
         addr += test.cacheLineWidth) {
      writeLine(test.memSocket, addr, writeData);
    }

    // Should receive interrupt by now
    wait(1, SC_NS);
    sc_assert(test.irq.read() == true);

    // Disable logging, and start flushing
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL,
              UndoLogger::BitMasks::CTRL_APPLY);

    // Wait until done
    limit = 1000;
    while (!(readWord(test.busSocket, UndoLogger::RegisterAddress::STATUS) &
             UndoLogger::BitMasks::STATUS_EMPTY_MASK)) {
      wait(5, SC_NS);
      if (--limit == 0) {
        sc_assert(false /*timeout*/);
      }
    }

    // Retire the interrupt
    test.returning_exception = 16 + test.m_dut.m_exceptionId;
    wait(1, SC_NS);
    sc_assert(test.irq.read() == false);

    // ------ TEST: Read FIFO from software
    spdlog::info("------ TEST: Read FIFO from software");
    // Enable logging
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL,
              UndoLogger::BitMasks::CTRL_ENABLE);

    // write some A's
    std::fill(writeData.begin(), writeData.end(), 0xAA);
    const auto nwrite3 = 8;
    for (unsigned addr = 0; addr < nwrite3 * test.cacheLineWidth;
         addr += test.cacheLineWidth) {
      writeLine(test.memSocket, addr, writeData);
    }

    // Disable logging
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL, 0);

    // Read undo log through the bus interface
    std::fill(writeData.begin(), writeData.end(), 0xAA);
    for (unsigned addr = 0; addr < nwrite3 * test.cacheLineWidth;
         addr += test.cacheLineWidth) {
      // Check fill level
      sc_assert(getFreeSlots() ==
                test.capacity - nwrite3 + (addr / test.cacheLineWidth));

      // Check address
      sc_assert(readWord(test.busSocket, UndoLogger::RegisterAddress::FIFO) ==
                addr);

      // Check data (should contain old values)
      for (int i = 0; i < test.cacheLineWidth / 4; ++i) {
        sc_assert(readWord(test.busSocket, UndoLogger::RegisterAddress::FIFO) ==
                  0x55555555);
      }
    }

    // ------ TEST: Flush software undo log through FIFO
    spdlog::info("------ TEST: Read FIFO from software");
    // Disable logging
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL, 0);

    // write some 5's through the FIFO to undo changes
    for (unsigned addr = 0; addr < nwrite3 * test.cacheLineWidth;
         addr += test.cacheLineWidth) {
      // Write address
      writeWord(test.busSocket, UndoLogger::RegisterAddress::FIFO, addr);

      // Write data (should contain old values)
      for (int i = 0; i < test.cacheLineWidth / 4; ++i) {
        writeWord(test.busSocket, UndoLogger::RegisterAddress::FIFO,
                  0x55555555);
      }

      // Enable flushing
      writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL,
                UndoLogger::BitMasks::CTRL_APPLY);

      // Wait for flushing if log is getting full
      int limit1 = 1000;
      while (getFreeSlots() < 3 && --limit1)
        ;
      sc_assert(limit1 > 0);
    }

    // Check data
    for (int i = 0; i < test.cacheLineWidth / 4; ++i) {
      std::vector<uint8_t> data(test.cacheLineWidth, 0);
      readLine(test.memSocket, UndoLogger::RegisterAddress::FIFO, data);
      for (const auto v : data) {
        sc_assert(v == 0x55);
      }
    }

    // ------ TEST: Writes to unsafe zone are not logged
    spdlog::info("------ TEST: Writes to unsafe zone are not logged");

    // Set up unsafe zone
    writeWord(test.busSocket, UndoLogger::RegisterAddress::UNSAFE_BASE, 0);
    writeWord(test.busSocket, UndoLogger::RegisterAddress::UNSAFE_SIZE,
              nwrite3 * test.cacheLineWidth);

    // Enable logging
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL,
              UndoLogger::BitMasks::CTRL_ENABLE);

    // write some A's
    std::fill(writeData.begin(), writeData.end(), 0xAA);
    for (unsigned addr = 0; addr < nwrite3 * test.cacheLineWidth;
         addr += test.cacheLineWidth) {
      writeLine(test.memSocket, addr, writeData);
      // Check fill level
      sc_assert(getFreeSlots() == test.capacity);
    }

    // Disable logging
    writeWord(test.busSocket, UndoLogger::RegisterAddress::CTRL, 0);

    // Check data
    for (int i = 0; i < test.cacheLineWidth / 4; ++i) {
      std::vector<uint8_t> data(test.cacheLineWidth, 0);
      readLine(test.memSocket, UndoLogger::RegisterAddress::FIFO, data);
      for (const auto v : data) {
        sc_assert(v == 0xAA);
      }
    }

    // TODO: Check overflow

    // TODO: Filter / unsafe zone

    spdlog::info("All tests successful.");
    sc_stop();
  }

  void writeWord(tlm_utils::simple_initiator_socket<dut> & socket,
                 const uint32_t addr, const uint32_t val, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[TARGET_WORD_SIZE];
    trans.set_data_ptr(data);
    trans.set_data_length(TARGET_WORD_SIZE);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);

    if (TARGET_WORD_SIZE == 4) {
      Utility::unpackBytes(data, Utility::htotl(val), 4);
    } else if (TARGET_WORD_SIZE == 2) {
      Utility::unpackBytes(data, Utility::htots(val), 2);
    } else {
      SC_REPORT_FATAL(this->name(), "Invalid TARGET_WORD_SIZE");
    }

    socket->b_transport(trans, delay);
    if (doWait) {
      wait(delay);
    }
  }
  uint32_t readWord(tlm_utils::simple_initiator_socket<dut> & socket,
                    const uint32_t addr, bool doWait = true,
                    bool debug = false) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[TARGET_WORD_SIZE];
    trans.set_data_ptr(data);
    trans.set_data_length(TARGET_WORD_SIZE);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);
    if (!debug) {
      socket->b_transport(trans, delay);
    } else {
      socket->transport_dbg(trans);
    }

    if (doWait) {
      wait(delay);
    }

    if (TARGET_WORD_SIZE == 4) {
      return Utility::ttohl(Utility::packBytes(data, 4));
    } else if (TARGET_WORD_SIZE == 2) {
      return Utility::ttohs(Utility::packBytes(data, 2));
    } else {
      SC_REPORT_FATAL(this->name(), "Invalid TARGET_WORD_SIZE");
      return -1;
    }
  }

  void writeLine(tlm_utils::simple_initiator_socket<dut> & socket,
                 const unsigned address, std::vector<uint8_t> &val,
                 bool doWait = true) {
    sc_assert(val.size() == test.cacheLineWidth);
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    trans.set_data_ptr(&val[0]);
    trans.set_data_length(test.cacheLineWidth);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(address);
    socket->b_transport(trans, delay);
    if (doWait) {
      wait(delay);
    }
  }

  // Read a cache line into val
  void readLine(tlm_utils::simple_initiator_socket<dut> & socket,
                const unsigned address, std::vector<uint8_t> &val,
                bool doWait = true) {
    sc_assert(val.size() == test.cacheLineWidth);
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    trans.set_data_ptr(&val[0]);
    trans.set_data_length(test.cacheLineWidth);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(address);
    socket->b_transport(trans, delay);
    if (doWait) {
      wait(delay);
    }
  }

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  // Set up paths
  // Parse CLI arguments & config file
  Config::get().parseFile("../config/MemicBoard-config.yml");

  tester t("tester");
  sc_start();
  return false;
}
