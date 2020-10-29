/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Model of the Bosh BME280 SPI-enabled temperature+humidity+pressure sensor.
 */

#include <array>
#include <deque>
#include <systemc>
#include <vector>
#include "sd/SpiDevice.hpp"

/**
 * @brief Accelerometer class to implement a simple 8-bit 3-axis accelerometer.
 * Roughly imitates the interface to that of BMA400, but in a simplified
 * version. Optionally samples 3 axes, and stores the result in a FIFO.
 *
 * SPI interface:
 * --------------
 * The first byte after asserting chipSelect is an address.
 * Similarly to the BMA400, we use the top bit of the SPI address to determine
 * whether the next access is a write or read. Burst reads are performed by
 * keeping chipSelect asserted after the first read, and issuing more 8-bit SPI
 * transactions. Burst reads  do not autoincrement the address, so can be used
 * to repeatedly take values from the output fifo.
 *
 * IRQ output:
 * -----------
 * When CTRL_IE is set:
 *   - In SingleMeasurement mode, the irq signal will be asserted when the FIFO
 *     is nonempty, and cleared when the FIFO is empty again (after being read)
 *   - In ContinuousMeasurement mode, the irq signal will be set when the
 *     measurement FIFO is filled to/beyond FIFO_THR*4 bytes, and cleared when
 *     the FIFO holds less than FIFO_THR*4 bytes.
 *
 *
 * Example hookup interface:
 * -------------------------
 *
 *  MCU                                           Accelerometer
 *  -----.                                      .--------------.
 *       |  <SpiTransactionExtension>           |              |
 *   SPI |------------------------------------->| SPI          |
 *       |                                      |              |
 *       |  <sc_logic_resolved>                 |              |
 *  GPIO |<-------------------------------------| irq          |
 *       |                                      |              |
 *       |  <sc_logic_resolved>                 |              |
 *  GPIO |------------------------------------->| chipSelect   |
 *  -----'                                      '--------------'
 *
 */
class Accelerometer : public SpiDevice {
 public:
  SC_HAS_PROCESS(Accelerometer);

  /* ------ Ports ------ */
  sc_core::sc_out_resolved irq{"irq"};

  /* ------ Public methods ------ */

  //! Constructor
  Accelerometer(const sc_core::sc_module_name nm);

  /**
   * @brief reset Clear control registers and reset state.
   */
  virtual void reset(void) override;

  /**
   * @brief set up methods & sensitivity
   */
  virtual void end_of_elaboration() override;

  /* ------ Public constants ------ */

  // Register addresses
  // Data registers
  struct RegisterAddress {
    // clang-format off

    // General control register
    static const unsigned CTRL =                0x00;

    // Sampling clock divider register, divides a 10KHz sampling frequency by
    // val + 1, i.e. fs = 10KHz / (CTRL_FS + 1)
    static const unsigned CTRL_FS =             0x04;

    // Status register
    static const unsigned STATUS =              0x08;

    // Data register. Note that this is a virtual read-only register where
    // reads are actually forwarded to the output fifo. If the output fifo is
    // empty, a read from this register returns 0, and a warning is issued.
    static const unsigned DATA =                0x0c;

    // FIFO threshold register. Sets the FIFO threshold to 4*FIFO_THR, i.e. a
    // value of 16, sets the FIFO threshold to 64 bytes.
    // If CTRL_IE is set, an interrupt pulse is generated when FIFO is filled
    // to or beyond the FIFO threshold.
    static const unsigned FIFO_THR =            0x10;

    // clang-format on
  };

  // Bit masks/patterns
  struct BitMasks {
    // clang-format off
    // CTRL Masks
    static const uint8_t CTRL_MODE =                    (3u << 0);

    // ------ CTRL Settings ------
    //! Sleep mode, for low power consumption
    static const uint8_t CTRL_MODE_SLEEP =              (0u << 0);

    //! Standby mode, ready for taking measurements
    static const uint8_t CTRL_MODE_STANDBY =            (1u << 0);

    //! Continuous sampling until disabled by changing to a different mode. If
    //! CTRL_IE is set, an interrupt is generated when the FIFO has filled
    //! to/beyond FIFO_THR*4
    static const uint8_t CTRL_MODE_CONTINUOUS =         (2u << 0);

    //! Single sampling: captures a single sample frame and returns to standby
    //! mode. If CTRL_IE is set, an interrupt is generated when the FIFO is
    //! nonempty
    static const uint8_t CTRL_MODE_SINGLE =             (3u << 0);

    //! Software reset. Clears mode and state.
    static const uint8_t CTRL_SW_RESET =                (1u << 2);

    //! Enable sampling of X axis
    static const uint8_t CTRL_X_EN =                    (1u << 3);

    //! Enable sampling of Y axis
    static const uint8_t CTRL_Y_EN =                    (1u << 4);

    //! Enable sampling of Z axis
    static const uint8_t CTRL_Z_EN =                    (1u << 5);

    //! Enable interrupt output.
    static const uint8_t CTRL_IE =                      (1u << 6);

    // Sampling clock divider
    static const uint8_t CTRL_FS =                      0xff;

    // STATUS bits
    static const uint8_t STATUS_BUSY =                  (1u << 0);
    static const uint8_t STATUS_OVERFLOW =              (1u << 1);

    // clang-format on
  };

  // Scaling factors to convert from physical parameters to LSB's
  // adc_val = physical_unit_sample * x_SCALING;
  static constexpr double ACC_SCALE{255.0 / 40.0};  // [lsb/ms^-2]
  static constexpr double ACC_OFFSET{0.0};

  // Static delays (startup time etc), specified in seconds
  static constexpr double DELAY_SLEEP_TO_STANDBY{1.0E-3};
  static constexpr double DELAY_STANDBY_TO_SLEEP{1.0E-3};

  /* ------  Types ------ */
  struct SpiState {
    enum class Mode { Address, DataRead, DataWrite };
    Mode mode{Mode::Address};
    unsigned address{0xffffffff};

    void reset() {
      mode = Mode::Address;
      address = 0xffffffff;
    }
  };

  enum class MeasurementState {
    Sleep = 0,
    Standby = 1,
    ContinuousMeasurement = 2,
    SingleMeasurement = 3
  };

  /**
   * @brief struct to hold a fifo frame consisting of a header and samples.
   *
   * Header format: [xxxxxXYZ]
   *                      ^^^
   *     X axis enabled --'|`-- Z axis enabled
   *                       `-- Y axis enabled
   * Header indicates whether a sample is available for each of the three axes.
   * For example, when only the X axis is enabled, the header will be xxxxx100,
   * and the frame will be 2 bytes long (header + x axis sample).
   *
   */
  struct FifoFrame {
    uint8_t header;
    std::deque<uint8_t> data;
    bool headerTaken{false};

    struct HeaderFields {
      static const uint8_t X_AXIS_ENABLE = (1u << 0);
      static const uint8_t Y_AXIS_ENABLE = (1u << 1);
      static const uint8_t Z_AXIS_ENABLE = (1u << 2);
    };

    int size() const {
      return sizeof(header) +
             sizeof(uint8_t) * (((header & HeaderFields::X_AXIS_ENABLE) != 0) +
                                ((header & HeaderFields::Y_AXIS_ENABLE) != 0) +
                                ((header & HeaderFields::Z_AXIS_ENABLE) != 0));
    };

    uint8_t takeByte() {
      if (headerTaken) {
        auto res = data.front();
        data.pop_front();
        spdlog::info("FifoFrame::takeByte {:d} bytes left in data",
                     data.size());
        return res;
      } else {
        headerTaken = true;
        spdlog::info("FifoFrame::takeByte {:d} bytes left in data",
                     data.size());
        return header;
      }
    }

    /**
     * @brief deserialize construct FifoFrame from an array of bytes.
     */
    static FifoFrame deserialize(const std::vector<uint8_t>& data) {
      FifoFrame res;
      int i = 0;
      res.header = data[i++];
      if (res.header & HeaderFields::X_AXIS_ENABLE) {
        res.data.push_front(data[i++]);
      }
      if (res.header & HeaderFields::Y_AXIS_ENABLE) {
        res.data.push_front(data[i++]);
      }
      if (res.header & HeaderFields::Z_AXIS_ENABLE) {
        res.data.push_front(data[i++]);
      }
      return res;
    }
  };

  /**
   * @brief OutputFifo bounded fifo to hold measurement frames. Bounded by the
   * number of bytes held, rather than frame count, so can store many small
   * frames or fewer large frames.
   */
  struct OutputFifo {
    std::deque<FifoFrame> data;
    int nbytes = 0;
    const int capacity = 1024;  // Capacity in bytes (not elements)

    /**
     * @brief Push a new element to the front of the queue, pop oldest
     * element(s) if overflow.
     * @param val fifo frame to push
     * @retval true if the fifo overflows, false otherwise
     */
    bool push_front(const FifoFrame& val) {
      data.push_front(val);
      nbytes += val.size();
      while (nbytes >= capacity) {
        nbytes -= data.back().size();
        data.pop_back();
        return true;
      }
      return false;
    }

    /**
     * @brief takeByte take a byte from the oldest fifo entry.
     * @retval uint8_t one byte from oldest fifo entry if available, 0 if empty.
     */
    uint8_t takeByte() {
      if (nbytes <= 0) {
        SC_REPORT_WARNING("Accelerometer::OutputFifo::takeByte",
                          "Take byte while empty. Returning 0.");
        sc_assert(data.empty());
        return 0;
      }
      auto& e = data.back();
      uint8_t res = e.takeByte();
      if (e.data.empty()) {
        data.pop_back();
      }
      nbytes--;

      spdlog::info(
          "OutputFifo::takeByte returning {:d}, new size {:d}, nframes left "
          "{:d}",
          res, nbytes, data.size());
      return res;
    }

    /**
     * @brief get the number of bytes stored in the FIFO
     */
    int size() const { return nbytes; }

    /**
     * @brief reset clear state
     */
    void reset() {
      data.clear();
      nbytes = 0;
    }
  };

  // Trace file data
  struct InputTraceEntry {
    double acc_x;  // [ms^-2]
    double acc_y;  // [ms^-2]
    double acc_z;  // [ms^-2]
  };

  /* ------ Member variables ------ */
  SpiState m_spiState;
  MeasurementState m_measurementState{MeasurementState::Sleep};
  OutputFifo m_fifo;
  bool m_isWriteAccess{false};  // Indicate if current spi access is write/read
  sc_core::sc_event m_modeUpdateEvent{"modeUpdateEvent"};
  sc_core::sc_event m_irqUpdateEvent{"irqUpdateEvent"};
  bool m_setIrq{false};
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

  /**
   * @brief nextMeasurementState method to determine the next state of the
   * measurement loop state machine, based on current state and configuration
   * register(s).
   * @retval next state for the measurement state machine.
   */
  MeasurementState nextMeasurementState();

  /**
   * @brief updateIrq update irq signal according to m_setIrq when
   * m_irqUpdateEvent is triggered triggered.
   */
  void updateIrq();
};
