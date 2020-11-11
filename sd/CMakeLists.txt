#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
    SerialDevices
    Accelerometer.cpp
    Accelerometer.hpp
    Bme280.cpp
    Bme280.hpp
    LoopBackWire.cpp
    LoopBackWire.hpp
    SpiDevice.cpp
    SpiDevice.hpp
    )

target_link_libraries(
    SerialDevices
    PUBLIC
      systemc-ams
      systemc
    PRIVATE
    Cm0Microcontroller # FIXME Necessary for RegisterFile, but shouldn't be here
    )
