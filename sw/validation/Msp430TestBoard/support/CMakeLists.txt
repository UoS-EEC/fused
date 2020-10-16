#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13)

add_library(support "")

target_sources(support
  PRIVATE msp430-support.c msp430-support.h
  PUBLIC support.h
  )

