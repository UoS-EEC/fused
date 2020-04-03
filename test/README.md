# Tests
This directory contains test programs for Fused. Each program tests a specific
module/functionality of a module. Ideally this would be done with unit tests,
but unit testing is not supported in SystemC (the kernel cannot be torn down
and restarted at runtime).

Each test program returns `0` if successful and `1` otherwise. Each test gets
built to its own binary (e.g. `fused/build/test/exampleTest`. To debug a
failing test, run the test binary directly.
