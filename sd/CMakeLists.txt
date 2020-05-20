#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
    SerialDevices
    SpiDevice.cpp
    SpiDevice.hpp
    DummySpiDevice.cpp
    DummySpiDevice.hpp
    )

target_link_libraries(
    SerialDevices
    systemc-ams
    systemc
    Microcontroller
    )
