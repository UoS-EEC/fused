#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

# ------ Generic module tests ------
add_executable(testClockSourceChannel test_ClockSourceChannel.cpp)
target_link_libraries(testClockSourceChannel PRIVATE systemc)

add_executable(testCache test_Cache.cpp)

target_link_libraries(testCache
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Utilities
    Microcontroller
  )

add_executable(testRegisterFile
  test_RegisterFile.cpp
  ../mcu/RegisterFile.cpp
)

target_link_libraries(testRegisterFile
  PRIVATE
    Utilities
  )

# ------ Target-specific tests ------
IF(${TARGET_ARCH} STREQUAL "msp430")
add_executable(testDigitalIo
  test_DigitalIo.cpp
)

target_link_libraries(testDigitalIo
  PRIVATE
    PowerSystem
    systemc
    Msp430fr5xx
    Microcontroller
)


add_executable(testMsp430fr5xxClockSystem test_msp430fr5xxClockSystem.cpp)

target_link_libraries(testMsp430fr5xxClockSystem
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Utilities
    Microcontroller
  )

add_executable(testMsp430fr5xxTimerA test_msp430fr5xxTimerA.cpp)

target_link_libraries(testMsp430fr5xxTimerA
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Utilities
    Microcontroller
  )

add_executable(testMsp430fr5xxeUsciB
  test_msp430fr5xxeUsciB.cpp
  ../mcu/BusTarget.cpp
  ../mcu/RegisterFile.cpp
  ../mcu/SpiTransactionExtension.hpp
  ../mcu/msp430fr5xx/eUSCI_B.hpp
  ../mcu/msp430fr5xx/eUSCI_B.cpp
)

target_link_libraries(testMsp430fr5xxeUsciB
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Utilities
  )

add_executable(testSpiDevice
  test_SpiDevice.cpp
  ../sd/SpiDevice.hpp
  ../sd/SpiDevice.cpp
  ../sd/DummySpiDevice.hpp
  ../sd/DummySpiDevice.cpp
  ../mcu/SpiTransactionExtension.hpp
  ../ps/DynamicEnergyChannel.hpp
)

target_link_libraries(testSpiDevice
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Utilities
  )


ELSEIF(${TARGET_ARCH} STREQUAL "cm0")

add_executable(testCm0SysTick
  test_cm0SysTick.cpp
  ../mcu/BusTarget.cpp
  ../mcu/RegisterFile.cpp
  ../mcu/cortex-m0/SysTick.cpp
)

target_link_libraries(testCm0SysTick
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Utilities
  )

add_executable(testCm0Nvic
  test_cm0Nvic.cpp
  ../mcu/BusTarget.cpp
  ../mcu/RegisterFile.cpp
  ../mcu/cortex-m0/Nvic.cpp
)

target_link_libraries(testCm0Nvic
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Utilities
  )
ELSE()
  message(FATAL_ERROR "Invalid TARGET_ARCH ${TARGET_ARCH}")
ENDIF()

