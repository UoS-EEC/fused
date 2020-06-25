/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Model of the Bosh BME280 SPI-enabled temperature+humidity+pressure sensor.
 */

#include <array>
#include <systemc>
#include <vector>
#include "sd/SpiDevice.hpp"

class Bme280 : public SpiDevice {
 public:
  SC_HAS_PROCESS(Bme280);

  /* ------ Public methods ------ */

  //! Constructor
  Bme280(const sc_core::sc_module_name nm);

  /**
   * @brief reset Clear control registers and reset state.
   */
  virtual void reset(void) override;

  /**
   * @brief set up methods & sensitivity
   */
  virtual void end_of_elaboration() override;

  /* ------ Constants ------ */

  // Register addresses
  // Data registers
  static const unsigned ADDR_HUM_LSB = 0xfe;
  static const unsigned ADDR_HUM_MSB = 0xfd;
  static const unsigned ADDR_TEMP_XLSB = 0xfc;
  static const unsigned ADDR_TEMP_LSB = 0xfb;
  static const unsigned ADDR_TEMP_MSB = 0xfa;
  static const unsigned ADDR_PRESS_XLSB = 0xf9;
  static const unsigned ADDR_PRESS_LSB = 0xf8;
  static const unsigned ADDR_PRESS_MSB = 0xf7;

  // Control & status registers
  static const unsigned ADDR_CONFIG = 0xf5;
  static const unsigned ADDR_CTRL_MEAS = 0xf4;
  static const unsigned ADDR_STATUS = 0xf3;
  static const unsigned ADDR_CTRL_HUM = 0xf2;
  static const unsigned ADDR_RESET = 0xe0;
  static const unsigned ADDR_ID = 0xd0;

  // Calibration registers
  static const unsigned ADDR_CALIB_00 = 0x88;
  static const unsigned ADDR_CALIB_01 = 0x89;
  static const unsigned ADDR_CALIB_02 = 0x8a;
  static const unsigned ADDR_CALIB_03 = 0x8b;
  static const unsigned ADDR_CALIB_04 = 0x8c;
  static const unsigned ADDR_CALIB_05 = 0x8d;
  static const unsigned ADDR_CALIB_06 = 0x8e;
  static const unsigned ADDR_CALIB_07 = 0x8f;
  static const unsigned ADDR_CALIB_08 = 0x90;
  static const unsigned ADDR_CALIB_09 = 0x91;
  static const unsigned ADDR_CALIB_10 = 0x92;
  static const unsigned ADDR_CALIB_11 = 0x93;
  static const unsigned ADDR_CALIB_12 = 0x94;
  static const unsigned ADDR_CALIB_13 = 0x95;
  static const unsigned ADDR_CALIB_14 = 0x96;
  static const unsigned ADDR_CALIB_15 = 0x97;
  static const unsigned ADDR_CALIB_16 = 0x98;
  static const unsigned ADDR_CALIB_17 = 0x99;
  static const unsigned ADDR_CALIB_18 = 0x9a;
  static const unsigned ADDR_CALIB_19 = 0x9b;
  static const unsigned ADDR_CALIB_20 = 0x9c;
  static const unsigned ADDR_CALIB_21 = 0x9d;
  static const unsigned ADDR_CALIB_22 = 0x9e;
  static const unsigned ADDR_CALIB_23 = 0x9f;
  static const unsigned ADDR_CALIB_24 = 0xa0;
  static const unsigned ADDR_CALIB_25 = 0xa1;
  static const unsigned ADDR_CALIB_26 = 0xe1;
  static const unsigned ADDR_CALIB_27 = 0xe2;
  static const unsigned ADDR_CALIB_28 = 0xe3;
  static const unsigned ADDR_CALIB_29 = 0xe4;
  static const unsigned ADDR_CALIB_30 = 0xe5;
  static const unsigned ADDR_CALIB_31 = 0xe6;
  static const unsigned ADDR_CALIB_32 = 0xe7;
  static const unsigned ADDR_CALIB_33 = 0xe8;
  static const unsigned ADDR_CALIB_34 = 0xe9;
  static const unsigned ADDR_CALIB_35 = 0xea;
  static const unsigned ADDR_CALIB_36 = 0xeb;
  static const unsigned ADDR_CALIB_37 = 0xec;
  static const unsigned ADDR_CALIB_38 = 0xed;
  static const unsigned ADDR_CALIB_39 = 0xee;
  static const unsigned ADDR_CALIB_40 = 0xef;
  static const unsigned ADDR_CALIB_41 = 0xf0;

  // Scaling factors to convert from physical parameters to LSB's
  // adc_val = physical_unit_sample * x_SCALING;
  const double HUM_SCALE{0.008};     // [%RH/lsb]
  const double HUM_OFFSET{0.0};      // [%RH]
  const double PRESS_SCALE{0.0018};  // [hPa/lsb]
  const double PRESS_OFFSET{300.0};  // [hPa]
  const double TEMP_SCALE{0.01};     // [C/lsb]
  const double TEMP_OFFSET{-40.0};   // [C]

 private:
  /* ------ Private types ------ */
  enum class SpiState { Address, Data };
  enum class MeasurementState { PowerOff, Sleep, Normal, Forced };

  /* ------ Private variables ------ */
  SpiState m_spiState{SpiState::Address};
  MeasurementState m_measurementState{MeasurementState::Sleep};
  unsigned m_activeAddress{0xffff};
  bool m_isWriteAccess{false};  // Indicate if current spi access is write/read
  sc_core::sc_event m_modeUpdateEvent{"modeUpdateEvent"};

  // Trace file data
  struct InputTraceEntry {
    double temperature;  // [C]
    double humidity;     //  Percent Relative Humidity [%RH]
    double pressure;     // [hPa]

    InputTraceEntry(const double temperature_, const double humidity_,
                    const double pressure_)
        : temperature(temperature_), humidity(humidity_), pressure(pressure_) {}
  };

  std::vector<InputTraceEntry> m_inputTrace;
  sc_core::sc_time m_inputTraceTimestep;

  /* ------ Private methods ------ */

  /**
   * @brief SPI interface process. Handles register accesses.
   */
  void spiInterface();

  /**
   * @brief main measurement state machine / loop.
   */
  void measurementLoop();
};
