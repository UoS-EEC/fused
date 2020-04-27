/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <spdlog/spdlog.h>
#include <systemc>
#include <tlm>

struct SpiTransaction : pubic tlm::tlm_extension<SpiTransaction> {
 public:
  /* ------ Types ------ */
  //! Polarity, clock idles high or low.
  enum class SpiPolarity { SPI_POLARITY_HIGH, SPI_POLARITY_LOW };

  enum class SpiPhase {
    SPI_PHASE_0,  //! Data transmitted on first edge, captured on second.
    SPI_PHASE_1   //! Data is transmitted on second edge, captured on first.
  };

  //! Transfer order, LSB or MSB first
  enum class SpiBitOrder { SPI_LSB_FIRST, SPI_MSB_FIRST };

  /* ------ Public variables ------ */
  int nDataBits;
  sc_core::sc_time clkPeriod;
  SpiPolarity polarity;
  SpiPhase phase;
  SpiBitOrder bitOrder;

 public:
  /* ------ Public methods ------ */

  /**
   * @brief constructor
   */
  SpiTransaction(const nDataBits_, const sc_time clkPeriod_,
                 const SpiPolarity polarity_, const SpiPhase phase_,
                 const SpiBitOrder bitOrder_)
      : nDataBits(nDataBits_),
        clkPeriod(clkPeriod_),
        polarity(polarity_),
        phase(phase_),
        bitOrder(bitOrder_) {}

  /**
   * @brief maskData compute the masked data, according to nDataBits
   * @param word data to mask
   * @retval word masked according to nDataBits
   */
  int maskData(const unsigned word) const {
    return (word & ((1u << (nDataBits + 1)) - 1));
  }

  /**
   * @brief transferTime return the transfer time of this transaction.
   */
  const sc_core::sc_time transferTime() const { return nDataBits * clkPeriod; }

  /**
   * Mandatory function for tlm payload extensions
   */
  virtual tlm::tlm_extension_base *clone() const override {
    return new SpiTransaction(nDatabits, clkPeriod, polarity, phase, bitOrder);
  }

  /**
   * Mandatory function for tlm payload extensions
   */
  virtual void copy_from(const tlm::tlm_extension_base &ext) override {
    auto &source = static_cast<SpiTransaction>;
    nDataBits = source.nDataBits;
    clkPeriod = source.clkPeriod;
    polarity = source.polarity;
    phase = source.phase;
    bitOrder = source.bitOrder;
  }

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const SpiTransaction &rhs) {
    os << "<SpiTransaction>:\n";
    os << "\tnDataBits: " << rhs.nDataBits << "\n";
    os << "\tclkPeriod: " << rhs.clkPeriod.to_string() << "\n";
    os << "\tpolarity: "
       << (rhs.polarity == SpiPolarity::SPI_POLARITY_HIGH ? "HIGH" : "LOW")
       << "\n";
    os << "\tphase: " << (rhs.phase == SpiPhase::SPI_PHASE_0 ? "0" : "1")
       << "\n";
    os << "\tbitOrder: "
       << (rhs.bitOrder == SpiBitOrder::SPI_LSB_FIRST ? "LSB_FIRST"
                                                      : "MSB_FIRST")
       << "\n";
    os << "\tTransfer time: " << rhs.transferTime.to_seconds() << "\n";
    return os;
  }
};
