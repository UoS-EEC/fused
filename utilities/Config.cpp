/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <exception>
#include <iostream>
#include <map>
#include <string>
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

void Config::parseCli(int argc, char *argv[]) {
  if (argc == 1) {
    return;
  }

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "-h" || std::string(argv[i]) == "--help") {
      std::cout << "\nusage: fused [-B board] [-O odir] [-x program] [-C config] \n\n";
      std::cout << "-B, --board \t : which board to run\n";
      std::cout << "-O, --odir \t : path to output directory\n";
      std::cout << "-x, --program \t : path to program hex file\n";
      std::cout << "-C, --config \t : path to config file\n";
      exit(0);
    } else if (std::string(argv[i]) == "-C" || std::string(argv[i]) == "--config") {
      m_config["ConfigFile"] = std::string(argv[i + 1]);
      spdlog::info("Loading config from file: {:s}", argv[i + 1]);
      i++;
    } else if (std::string(argv[i]) == "-O" ||
               std::string(argv[i]) == "--odir") {
      m_config["OutputDirectory"] = argv[i + 1];
      spdlog::info("Writing output to file: {:s}: ",
                   m_config["OutputDirectory"]);
      i++;
    } else if ((std::string(argv[i]) == "-x") ||
               (std::string(argv[i]) == "--program")) {
      m_config["GdbServer"] = "False";
      m_config["ProgramHexFile"] = std::string(argv[i + 1]);
      spdlog::info("Loading and immediately running program from: {:s}",
                   m_config["ProgramHexFile"]);
      i++;
    } else if ((std::string(argv[i]) == "-B") ||
               (std::string(argv[i]) == "--board")) {
      m_config["Board"] = std::string(argv[i + 1]);
      i++;
    } else {
      // Unrecognized option
      spdlog::error("Unrecognized CLI option \"{}\" exiting...",
                    std::string(argv[i]));
      exit(1);
    }
  }
}

void Config::parseFile(const std::string &fn) {
  if (fn == "") {
    if (m_config.find("ConfigFile") != m_config.end()) {
      m_configFileName = m_config["ConfigFile"];
    } else {
      m_configFileName = "config.yaml";  // Debug convenience
    }
  }
  Utility::assertFileExists(m_configFileName);
  auto ymlconfig =
      YAML::LoadFile(m_configFileName).as<std::map<std::string, std::string>>();
  m_config.insert(ymlconfig.begin(),
                  ymlconfig.end());  // Note: CLI arguments override yaml-config
}

const std::string &Config::getString(const std::string &key) const {
  auto it = m_config.find(key);
  if (it != m_config.end()) {
    return it->second;
  } else {
    throw std::invalid_argument(key + ": not found in config file " +
                                m_configFileName);
  }
}

unsigned int Config::getUint(const std::string &key) const {
  return static_cast<unsigned int>(std::stoi(getString(key)));
}

double Config::getDouble(const std::string &key) const {
  return std::stod(getString(key));
}

bool Config::getBool(const std::string &key) const {
  const auto &val = getString(key);
  if (val == "True") {
    return true;
  } else if (val == "False") {
    return false;
  } else {
    throw std::invalid_argument(key + " is not a boolean value.");
  }
}

bool Config::contains(const std::string &key) const {
  return m_config.find(key) != m_config.end();
}
