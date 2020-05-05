#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
    SerialDevices
    SpiDevice.hpp
    SpiDevice.cpp
    DummySpiDevice.hpp
    DummySpiDevice.cpp
    )

target_link_libraries(
    SerialDevices 
    systemc-ams
    systemc
    )
