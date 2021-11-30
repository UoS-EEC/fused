/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//! Maximum size of a RSP packet is used to return the value of all the
//! registers, each of which takes 8 chars. There are a total of 32 GPRs plus
//! PPC, SR and NPC. Plus one byte for end of string marker.
#define RSP_MAX_PKT_SIZE ((32 + 3) * 8 + 1)

//! Default port for RSP to listen on
#define DEFAULT_RSP_PORT 51000
//#define TARGET_LITTLE_ENDIAN
//#define TARGET_WORD_SIZE 4

#include "boards/Board.hpp"
#include "boards/Cm0SensorNode.hpp"
#include "boards/Cm0TestBoard.hpp"
#include "boards/MemicBoard.hpp"
#include "boards/Msp430TestBoard.hpp"
#include "utilities/Config.hpp"
#include "utilities/SimulationController.hpp"
#include <chrono>
#include <cstdlib>
#include <ihex-parser/IntelHexFile.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <systemc-ams>
#include <systemc>
#include <thread>

#ifdef GDB_SERVER
#include <gdb-server/GdbServer.hpp>
#endif

using namespace sc_core;

// clang-format off
// A dummy module to implement the end_of_simulation callback.
SC_MODULE(DummyModule){
  public:
  DummyModule(sc_module_name nm, SimulationController *simCtrl)
    : sc_module(nm), m_simCtrl(simCtrl) {};

  /**
   * @brief end_of_simulation callback. Called by systemc after sc_stop(), but
   * before module destruction. Use this to stop debugging server before
   * module destruction.
   */
  virtual void end_of_simulation() override {
    if (Config::get().getBool("GdbServer")) {
      m_simCtrl->stopServer(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
      while(m_simCtrl->isServerRunning()) {
        spdlog::info("Waiting for debug server to exit...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  private:
  SimulationController *m_simCtrl;
};
// clang-format on

int sc_main(int argc, char *argv[]) {
  // sc_set_time_resolution(1, SC_NS);
  int rspPort = DEFAULT_RSP_PORT;

  // Parse CLI arguments & config file
  Config::get().parseCli(argc, argv);
  if (Config::get().contains("ConfigFile")) {
    // Config file parsed as argument
    Config::get().parseFile(Config::get().getString("ConfigFile"));
  } else {
    // Default config for board
    Config::get().parseFile(std::string("../config/") +
                            Config::get().getString("Board") + "-config.yml");
  }

  // Instantiate board
  Board *board;
  const auto &bstring = Config::get().getString("Board");
  if (bstring == "Cm0TestBoard") {
    board = new Cm0TestBoard("Cm0TestBoard");
  } else if (bstring == "Msp430TestBoard") {
    board = new Msp430TestBoard("Msp430TestBoard");
  } else if (bstring == "Cm0SensorNode") {
    board = new Cm0SensorNode("Cm0SensorNode");
  } else if (bstring == "MemicBoard") {
    board = new MemicBoard("MemicBoard");
  } else {
    SC_REPORT_FATAL(
        "sc_main",
        fmt::format("invalid setting for Board \"{:s}\"", bstring).c_str());
    exit(1); // supress "board may be uninitialized" warning
  }

  // Set up output folder
  // When <filesystem> is available:
  // std::filesystem::create_directories(Config::get().getString("OutputDirectory"))
  auto sysStatus = system(
      std::string("mkdir -p " + Config::get().getString("OutputDirectory"))
          .c_str());
  if (sysStatus) {
    spdlog::error("Failed to create output directory at {} ... exiting",
                  Config::get().getString("OutputDirectory"));
    exit(1);
  }

  /* ------ Simulation control ------ */
  SimulationController simCtrl(&board->getMicrocontroller());
  [[maybe_unused]] DummyModule d(
      "dummy", &simCtrl); // Used to access end_of_simulation callback

#ifdef GDB_SERVER
  GdbServer *gdbServer;
  std::thread *dbgThread;
#endif

  if (Config::get().getBool("GdbServer")) {
#ifdef GDB_SERVER
    spdlog::info("Starting gdb server");
    gdbServer = new GdbServer(&simCtrl, rspPort);
    dbgThread = new std::thread(&GdbServer::serverThread, gdbServer);
#else
    spdlog::error("'GdbServer' true in config, but GDB_SERVER is undefined.");
    exit(1);
#endif
  } else {
    // Load binary to mcu
    auto fn = Config::get().getString("ProgramHexFile");
    if (fn.find(".hex") == std::string::npos &&
        fn.find(".ihex") == std::string::npos) {
      spdlog::error(
          "-x: Invalid file format for input file {:s}, must be '.hex' or "
          "'.ihex'",
          fn);
      return 1;
    }
    IntelHexFile programFile(fn);
    sc_start(SC_ZERO_TIME); // Finish elaboration before programming
    for (const auto &s : programFile.getProgramData()) {
      simCtrl.writeMem(&s.second[0], s.first, s.second.size());
    }
    simCtrl.unstall();
  }

  auto timeLimit =
      sc_time::from_seconds(Config::get().getDouble("SimTimeLimit"));

  spdlog::info("Starting simulation with time limit {:s}.",
               timeLimit.to_string());
  sc_start(timeLimit);

  if (sc_time_stamp() >= timeLimit) {
    spdlog::warn("Simulation stopped at SimTimeLimit {:s}",
                 sc_time_stamp().to_string());
    sc_stop();
  } else if (!sc_end_of_simulation_invoked()) {
    spdlog::warn("Simulation stopped without explicit sc_stop() at {:s}",
                 sc_time_stamp().to_string());
    sc_stop();
  }

#ifdef GDB_SERVER
  if (Config::get().getBool("GdbServer")) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    dbgThread->join();
#pragma GCC diagnostic pop
  }
#endif
  delete board;

  return 0;
}
