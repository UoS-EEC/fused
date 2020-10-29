#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

# ------ Generic module tests ------
add_executable(testClockSourceChannel
  test_ClockSourceChannel.cpp)
target_link_libraries(testClockSourceChannel
  PRIVATE
    systemc
    Cm0Microcontroller)

# ------ Cache ------
add_executable(testMsp430Cache
  test_Cache.cpp
  )

target_link_libraries(
  testMsp430Cache
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Msp430Utilities
    Msp430Microcontroller
  )

target_compile_definitions(
  testMsp430Cache
  PRIVATE
    MSP430_ARCH
    TARGET_WORD_SIZE=2
  )

# ------ RegisterFile ------
add_executable(testMsp430RegisterFile
  test_RegisterFile.cpp
)

target_link_libraries(
  testMsp430RegisterFile
  PRIVATE
    Msp430Utilities
    Msp430Microcontroller
  )

add_executable(testCm0RegisterFile
  test_RegisterFile.cpp
)

target_link_libraries(
  testCm0RegisterFile
  PRIVATE
    Cm0Utilities
    Cm0Microcontroller
  )

# ------ BME280 ------
add_executable(testBme280
  test_Bme280.cpp
)

target_link_libraries(testBme280
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    SerialDevices
    Msp430Utilities
    Msp430Microcontroller
  )

# ------ Accelerometer ------
add_executable(testAccelerometer
  test_Accelerometer.cpp
)

target_link_libraries(testAccelerometer
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    SerialDevices
    Cm0Utilities
    Cm0Microcontroller
  )

# ------ MSP430 DigitalIo ------
add_executable(testDigitalIo
  test_DigitalIo.cpp
)

target_link_libraries(testDigitalIo
  PRIVATE
    PowerSystem
    systemc
    Msp430Microcontroller
)

# ------ MSP430 ClockSystem ------
add_executable(testMsp430fr5xxClockSystem test_msp430fr5xxClockSystem.cpp)

target_link_libraries(testMsp430fr5xxClockSystem
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Msp430Utilities
    Msp430Microcontroller
  )

# ------ MSP430 TimerA ------
add_executable(testMsp430fr5xxTimerA test_msp430fr5xxTimerA.cpp)

target_link_libraries(testMsp430fr5xxTimerA
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Msp430Utilities
    Msp430Microcontroller
  )

# ------ MSP430 EUSCI_B ------
add_executable(testMsp430fr5xxeUsciB
  test_msp430fr5xxeUsciB.cpp
)

target_link_libraries(testMsp430fr5xxeUsciB
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Msp430Utilities
    Msp430Microcontroller
    SerialDevices
  )

# ------ SPI device ------
add_executable(testSpiDevice
  test_SpiDevice.cpp
)

target_link_libraries(testSpiDevice
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Msp430Utilities
    Msp430Microcontroller
    SerialDevices
  )

# ------ MSP430 DMA ------
add_executable(testMsp430fr5xxDma
  test_msp430fr5xxDma.cpp
  )

target_link_libraries(testMsp430fr5xxDma
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Msp430Utilities
    Msp430Microcontroller
  )

# ------ MSP430 CPU ------
add_executable(testMsp430fr5xxCpu
  test_msp430fr5xxCpu.cpp
  )

target_link_libraries(testMsp430fr5xxCpu
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Msp430Utilities
    Msp430Microcontroller
  )

# ------ CM0 SysTick ------
add_executable(testCm0SysTick
  test_cm0SysTick.cpp
)

target_link_libraries(testCm0SysTick
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Cm0Utilities
    Cm0Microcontroller
  )

# ------ CM0 NVIC ------
add_executable(testCm0Nvic
  test_cm0Nvic.cpp
)

target_link_libraries(testCm0Nvic
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Cm0Utilities
    Cm0Microcontroller
  )


# ------ CM0 SPI ------
add_executable(testCm0Spi
  test_cm0Spi.cpp
)

target_link_libraries(testCm0Spi
  PRIVATE
    systemc
    spdlog::spdlog
    PowerSystem
    Cm0Utilities
    Cm0Microcontroller
  )
