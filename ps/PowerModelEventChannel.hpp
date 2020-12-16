/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>
#include <systemc>
#include <vector>
#include "ps/PowerModelEventBase.hpp"
#include "ps/PowerModelEventChannelIf.hpp"

class PowerModelEventChannel : public virtual PowerModelEventChannelOutIf,
                               public virtual PowerModelEventChannelInIf {
 public:
  //! Constructor
  PowerModelEventChannel();

  virtual int registerEvent(
      std::unique_ptr<PowerModelEventBase> eventPtr) override;

  virtual void write(const int eventId, const int n = 1) override;

  virtual int pop(const int eventId) override;

  virtual size_t size() const override;

  virtual double popEnergy(const int eventId, double supplyVoltage) override;

  virtual double popEnergy(double supplyVoltage) override;

 private:
  std::vector<std::unique_ptr<PowerModelEventBase>> m_events;
  std::vector<int> m_eventRates;
};
