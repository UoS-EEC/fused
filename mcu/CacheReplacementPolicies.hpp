/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <algorithm>
#include <array>
#include <list>
#include <queue>

/**
 * Collection of cache replacement policies
 *
 */
class CacheReplacementIf {
 public:
  /**
   * @brief hit register an access hit on a cache line, update internal dirty
   * bit if the access was a write.
   * @param lineNo line number that was hit.
   * @param isWrite Set to true if the access was a write, set to false if it
   * was a read.
   */
  virtual void hit(const int lineNo, bool isWrite) = 0;

  /**
   * @brief miss register an access miss, and return a victim cache line.
   * @param isWrite Set to true if the access was a write, set to false if it
   * @retval victim cache line number.
   */
  virtual int miss(bool isWrite) = 0;

  /**
   * @brief reset reset to power-on defaults.
   */
  virtual void reset() = 0;
};

/**
 * Least recently used replacement policy
 */
class CacheReplacementLru : public CacheReplacementIf {
 public:
  CacheReplacementLru(const int nLines) {
    // Initialize in arbitrary order
    for (auto i = 0; i < nLines; i++) {
      m_lru.push_back(i);
    }
  }

  virtual void hit(const int lineNo, [[maybe_unused]] bool isWrite) override {
    m_lru.remove(lineNo);
    m_lru.push_front(lineNo);
  }

  virtual int miss([[maybe_unused]] bool isWrite) override {
    m_lru.push_front(m_lru.back());
    m_lru.pop_back();
    return m_lru.front();
  }

  virtual void reset() override {
    int i = 0;
    for (auto &e : m_lru) {
      e = i++;
    }
  }

 private:
  /* ------ Private variables ------ */
  std::list<unsigned int> m_lru{};
};

class CacheReplacementRoundRobin : public CacheReplacementIf {
 public:
  CacheReplacementRoundRobin(const int nLines) : m_nLines(nLines) {}

  virtual void hit([[maybe_unused]] const int lineNo,
                   [[maybe_unused]] bool isWrite) override {
    // Do nothing
  }

  virtual int miss([[maybe_unused]] bool isWrite) override {
    m_cnt = (m_cnt + 1) % m_nLines;
    return m_cnt;
  }

  virtual void reset() override { m_cnt = 0; }

 private:
  unsigned m_cnt{0};
  const unsigned m_nLines;
};

/**
 * Least frequently used replacement policy
 */
class CacheReplacementLfu : public CacheReplacementIf {
 public:
  CacheReplacementLfu(const int nLines, const int saturation)
      : m_counters(nLines, 0), m_saturation(saturation){};

  virtual void hit([[maybe_unused]] const int lineNo,
                   [[maybe_unused]] bool isWrite) override {
    if (m_counters[lineNo] < m_saturation) {
      m_counters[lineNo]++;
    }
  }

  virtual int miss([[maybe_unused]] bool isWrite) override {
    int victim = 0;
    int tie = -1;
    int min = m_counters[0];
    for (unsigned int i = 1; i < m_counters.size(); i++) {
      if (m_counters[i] < min) {
        victim = i;
        min = m_counters[i];
        tie = -1;
      } else if (m_counters[i] == min) {
        tie = i;
      }
    }

    if (tie > 0) {
      // Break tie
      victim = m_tieBreaker ? victim : tie;
      m_tieBreaker = !m_tieBreaker;
    }

    m_counters[victim] = 0;
    return victim;
  }

  virtual void reset() override {
    for (auto &e : m_counters) {
      e = 0;
    }
  }

 private:
  /* ------ Private variables ------ */
  std::vector<int> m_counters;
  const int m_saturation;
  bool m_tieBreaker{false};
};

/**
 * PseudoRandom: Pseudo random replacement using 16-bit LFSR
 */
class CacheReplacementPseudoRandom : public CacheReplacementIf {
 public:
  CacheReplacementPseudoRandom(const int nLines) : m_outputMask(nLines - 1) {}

  virtual void hit([[maybe_unused]] const int lineNo,
                   [[maybe_unused]] bool isWrite) override {
    // Do nothing
  }

  virtual int miss([[maybe_unused]] bool isWrite) override {
    /* taps: 16 14 13 11; feedback polynomial: x^16 + x^14 + x^13 + x^11 + 1
     */
    static uint16_t lfsr = 0xBEEF;  //! Only need one lfsr for the whole cache
    uint16_t bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5));
    lfsr = (lfsr >> 1) | (bit << 15);
    return lfsr & m_outputMask;
  }

  virtual void reset() override {
    // Do nothing
  }

 private:
  /* ------ Private variables ------ */
  const unsigned m_outputMask;
};
