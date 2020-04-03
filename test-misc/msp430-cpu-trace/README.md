# CPU trace tests for MSP430

This test setup records CPU traces by stepping through a binary instruction by
instruction and saving the register contents at each step. This is done while
running the program on a development platform, and running it in Fused. The
traces are then compared to check whether Fused simulates execution correctly.

This directory contains
  + `binaries`: compiled MSP430FR5994 binaries used in these tests
  + `meas-cpu-traces.tar.bz2`: measured cpu-state traces
  + `trace.gdb`: a template gdb script for generating traces
  + `verify-execution-traces.py`: a python script that:
    + emits gdb scripts for simulation and measurement
    + starts simulation/gdb and records traces
    + programs a development board to record measured traces (assumes a
      gdbserver is running at port :55000)
    + verifies simulation traces against measured traces
    + emits `reproduce-{sim|meas}.gdb` if a test fails; the gdb script brings
      the target back to immediately before the failing instruction executes.
