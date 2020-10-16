#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

IF(TARGET_ARCH STREQUAL "msp430")
  include_directories(.)
  add_subdirectory(mathtest)
  add_subdirectory(spi)
  add_subdirectory(dma)
ENDIF()

