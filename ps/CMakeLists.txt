#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
    PowerSystem
    Capacitor.hpp
    ConstantEnergyEvent.hpp
    LinearRegulator.hpp
    PowerModelBridge.hpp
    PowerModelEventBase.hpp
    PowerModelChannelIf.hpp
    PowerModelChannel.hpp
    PowerModelChannel.cpp
    PowerSupplies.hpp
    )

target_link_libraries(
    PowerSystem
    PUBLIC
      systemc-ams
      systemc
    PRIVATE
      spdlog::spdlog
    )
