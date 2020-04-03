/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <string>

/**
 * @brief The Config class Singleton class to store CLI and yaml configs
 */
class Config {
 public:
  static Config &get() {
    static Config instance;
    return instance;
  }

  /**
   * @brief parseFile Parse yaml file for config
   * @param fn path to config yaml file
   */
  void parseFile(std::string fn = "");

  /**
   * @brief parseCli Parse command line arguments
   * @param argc
   * @param argv
   */
  void parseCli(int argc, char *argv[]);

  std::string getString(std::string key);

  unsigned int getUint(std::string key);

  double getDouble(std::string key);

  bool getBool(std::string key);

 private:
  /* ------ Private variables ------ */
  std::map<std::string, std::string> m_config{};

  /* ------ Private methods ------ */
  Config() {}

  Config(const Config &);

  Config &operator=(const Config &);
};
