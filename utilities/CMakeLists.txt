#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 2.8)
add_library(
  Utilities
  Config.cpp
  Config.hpp
  Utilities.cpp
  Utilities.hpp
  IoSimulationStopper.hpp
  SimpleMonitor.hpp
  SimulationController.hpp
  )
