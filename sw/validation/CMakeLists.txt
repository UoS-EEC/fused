#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

if (${TARGET_ARCH} STREQUAL cm0)
  add_subdirectory(Cm0TestBoard)
  add_subdirectory(Cm0SensorNode)
elseif(${TARGET_ARCH} STREQUAL msp430)
  add_subdirectory(Msp430TestBoard)
endif()
