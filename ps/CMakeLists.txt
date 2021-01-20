#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
    PowerSystem
    ConstantEnergyEvent.hpp
    PowerModelBridge.hpp
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
