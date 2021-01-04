#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
    PowerSystem
    ConstantEnergyEvent.hpp
    DynamicEnergyChannel.hpp
    DynamicEnergyIf.hpp
    EnergyConsumer.cpp
    EnergyConsumer.hpp
    EnergyProducer.cpp
    EnergyProducer.hpp
    EventLog.cpp
    EventLog.hpp
    PowerCalculator.cpp
    PowerCalculator.hpp
    PowerCombine.hpp
    PowerModelEventBase.hpp
    PowerModelChannelIf.hpp
    PowerModelChannel.hpp
    PowerModelChannel.cpp
    )

target_link_libraries(
    PowerSystem
    systemc-ams
    systemc
    )
