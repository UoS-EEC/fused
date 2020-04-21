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
  void parseFile(const std::string &fn = "");

  /**
   * @brief parseCli Parse command line arguments, these override yaml-configs.
   * @param argc
   * @param argv
   */
  void parseCli(int argc, char *argv[]);

  /**
   * @brief getString get a string value from the configuration
   * @param key configuration key (yaml key).
   * @retval configuration string.
   */
  const std::string &getString(const std::string &key) const;

  /**
   * @brief getUint get an unsigned integer value from the configuration.
   * @param key configuration key (yaml key).
   * @retval configuration unsigned integer.
   */
  unsigned int getUint(const std::string &key) const;

  /**
   * @brief getDouble get a floating point  value from the configuration.
   * @param key configuration key (yaml key).
   * @retval configuration double.
   */
  double getDouble(const std::string &key) const;

  /**
   * @brief getBool get a boolean value from the configuration.
   * @param key configuration key (yaml key).
   * @retval configuration string.
   */
  bool getBool(const std::string &key) const;

  /**
   * @brief contains check if the configuration contains a value for the key
   * specified
   * @param key configuration key (yaml key).
   * @retval true if the key is found in the configuration, false otherwise.
   */
  bool contains(const std::string &key) const;

 private:
  /* ------ Private variables ------ */
  std::map<std::string, std::string> m_config{};  //! Configuration
  std::string m_configFileName;                   //! Path to Yaml-file

  /* ------ Private methods ------ */

  // Private constructor
  Config() {}

  Config(const Config &);

  Config &operator=(const Config &);
};
