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

struct SpiTransactionExtension
    : public tlm::tlm_extension<SpiTransactionExtension> {
 public:
  /* ------ Types ------ */
  //! Polarity, clock idles high or low.
  enum class SpiPolarity { HIGH, LOW };

  enum class SpiPhase {
    CAPTURE_SECOND_EDGE,  //! Data transmitted on first edge, captured on
                          //! second.
    CAPTURE_FIRST_EDGE    //! Data is transmitted on second edge, captured on
                          //! first.
  };

  //! Transfer order, LSB or MSB first
  enum class SpiBitOrder { LSB_FIRST, MSB_FIRST };

  /* ------ Public variables ------ */
  int nDataBits{0};
  sc_core::sc_time clkPeriod{sc_core::SC_ZERO_TIME};
  SpiPolarity polarity{SpiPolarity::LOW};
  SpiPhase phase{SpiPhase::CAPTURE_FIRST_EDGE};
  SpiBitOrder bitOrder{SpiBitOrder::LSB_FIRST};
  int response{0};  //! Response message

 public:
  /* ------ Public methods ------ */

  SpiTransactionExtension(void) = default;
  /**
   * @brief constructor
   */
  SpiTransactionExtension(const int nDataBits_,
                          const sc_core::sc_time clkPeriod_,
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
    return new SpiTransactionExtension(nDataBits, clkPeriod, polarity, phase,
                                       bitOrder);
  }

  /**
   * Mandatory function for tlm payload extensions
   */
  virtual void copy_from(const tlm::tlm_extension_base &ext) override {
    auto &source = static_cast<const SpiTransactionExtension &>(ext);
    nDataBits = source.nDataBits;
    clkPeriod = source.clkPeriod;
    polarity = source.polarity;
    phase = source.phase;
    bitOrder = source.bitOrder;
    response = source.response;
  }

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os,
                                  const SpiTransactionExtension &rhs) {
    os << "<SpiTransactionExtension>:\n";
    os << "\tnDataBits: " << rhs.nDataBits << "\n";
    os << "\tclkPeriod: " << rhs.clkPeriod.to_string() << "\n";
    os << "\tpolarity: " << (rhs.polarity == SpiPolarity::HIGH ? "HIGH" : "LOW")
       << "\n";
    os << "\tphase: "
       << (rhs.phase == SpiPhase::CAPTURE_FIRST_EDGE ? "CAPTURE_FIRST_EDGE"
                                                     : "CAPTURE_SECOND_EDGE")
       << "\n";
    os << "\tbitOrder: "
       << (rhs.bitOrder == SpiBitOrder::LSB_FIRST ? "LSB_FIRST" : "MSB_FIRST")
       << "\n";
    os << "\tresponse: " << fmt::format("0x{:08x}", rhs.response) << "\n";
    os << "\tTransfer time: " << rhs.transferTime().to_string() << "\n";
    return os;
  }
};
