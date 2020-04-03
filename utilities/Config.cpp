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
    if (std::string(argv[i]) == "-C" || std::string(argv[i]) == "--config") {
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
    } else {
      // Unrecognized option
      spdlog::error("Unrecognized CLI option \"{}\" exiting...",
                    std::string(argv[i]));
      exit(1);
    }
  }
}

void Config::parseFile(std::string fn) {
  if (fn == "") {
    if (m_config.find("ConfigFile") != m_config.end()) {
      fn = m_config["ConfigFile"];
    } else {
      fn = "config.yaml";  // Debug convenience
    }
  }
  Utility::assertFileExists(fn);
  auto ymlconfig = YAML::LoadFile(fn).as<std::map<std::string, std::string>>();
  m_config.insert(ymlconfig.begin(),
                  ymlconfig.end());  // Note: CLI arguments override yaml-config
}

std::string Config::getString(std::string key) {
  if (m_config.find(key) != m_config.end()) {
    return m_config[key];
  } else {
    std::cerr << "ERROR: key " << key << " not found in config.";
    return "";
  }
}

unsigned int Config::getUint(std::string key) {
  if (m_config.find(key) != m_config.end()) {
    return static_cast<unsigned int>(std::stoi(m_config[key]));
  } else {
    std::cerr << "ERROR: key " << key << " not found in config.";
    exit(1);
  }
}

double Config::getDouble(std::string key) {
  if (m_config.find(key) != m_config.end()) {
    return std::stod(m_config[key]);
  } else {
    std::cerr << "ERROR: key " << key << " not found in config.";
    exit(1);
  }
}

bool Config::getBool(std::string key) {
  if (m_config.find(key) != m_config.end()) {
    if (m_config[key] == "True") {
      return true;
    } else if (m_config[key] == "False") {
      return false;
    } else {
      throw std::invalid_argument(key + ": not a boolean value.");
    }
  } else {
    std::cerr << "ERROR: key " << key << " not found in config.";
    exit(1);
  }
}
