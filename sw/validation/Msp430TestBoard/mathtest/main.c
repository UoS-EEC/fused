/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <math.h>
#include <msp430fr5994.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <support.h>

/* ------ Macros ------------------------------------------------------------*/

/* ------ Extern functions --------------------------------------------------*/

/* ------ Function Prototypes -----------------------------------------------*/
void assert(bool c);

/* ------ Variable Declarations ---------------------------------------------*/

/* ------ Function Declarations ---------------------------------------------*/

int main(void) {
  WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

  while (1) {
    // 16-bit unsigned
    uint16_t a = 13, b = 10;
    uint16_t c = a * b;  // c = 130
    c = c / a;           // c = 10
    c -= 10;             // c = 0
    c += 0xffff;         // c = 0xffff
    assert(c == 0xffff);

    // 16-bit signed
    int16_t d = -13, e = 10;
    int16_t f = d * e;  // f = -130
    f /= d;             // f = 10
    assert(f == 10);
    f -= 110;  // f = -100
    assert(f == -100);
    f += 300;  // f = 200
    assert(f == 200);

    // 8-bit unsigned
    uint8_t x = 13, y = 10;
    uint8_t z = x * y;  // z = 130
    z /= x;             // z = 10
    assert(z == 10);
    z -= 10;  // z = 0
    assert(z == 0);
    z += 0xff;  // z = 0xff
    assert(z == 0xff);

    // 8-bit signed
    int16_t l = -13, m = 10;
    int16_t n = l * m;  // n = -130
    assert(n == -130);
    n /= l;  // n = 10
    assert(n == 10);
    l = 110;
    n -= l;  // n = -100
    assert(n == -100);
    n += 300;  // n = 200
    assert(n == 200);
    n >>= 1;  // n = 100
    assert(n == 100);
    n <<= 1;  // n = 200
    assert(n == 200);

    // 32-bit math
    uint32_t big = 0xffffffff;
    uint32_t r = big + 2;  // big = 1
    assert(r == 1);

    r = (r + 10) * 12345678;
    assert(r == 11 * 12345678);

    r = 8192;
    r >>= 1;
    assert(r == 4096);

    // Bitwise ops
    l = 0x0f;
    m = 0xf0;
    n = l & m;
    assert(n == 0);

    l = 0x1;
    m = 0x3;
    n = l ^ m;
    assert(n == 2);

    l = 0x0f;
    m = 0x3;
    n = l | m;
    assert(n == 0xf);

    n &= 0x3;
    assert(n == 3);

    // 8-bit unsigned overflow
    uint8_t u8o_a = 0xff, u8o_b = 1, u8o_res;
    u8o_res = u8o_a + u8o_b;
    assert(u8o_res == 0);

    // 8-bit unsigned underflow
    uint8_t u8u_a = 0, u8u_b = 1, u8u_res;
    u8u_res = u8u_a - u8u_b;
    assert(u8u_res == 0xff);

    // 8-bit signed overflow
    int8_t i8o_a = 0xff, i8o_b = 1, i8o_res;
    i8o_res = i8o_a + i8o_b;
    assert(i8o_res == 0);

    // 8-bit signed underflow
    int8_t i8u_a = -128, i8u_b = 1, i8u_res;
    i8u_res = i8u_a - i8u_b;
    assert(i8u_res == 127);

    // Floats
    // Add
    float f_add_a = 1.3f, f_add_b = 1.2, f_res;
    f_res = f_add_a + f_add_b;
    assert((f_res <= 1.1f * (1.3f + 1.2f)) && (f_res >= 0.9f * (1.3f + 1.2f)));

    float f_a = 1.3f, f_b = 1.2345f;
    f_res = f_a * f_b;
    assert((f_res <= 1.1f * 1.3f * 1.2345f) &&
           (f_res >= 0.9f * 1.3f * 1.2345f));
    end_experiment();
  }
}

void assert(bool c) {
  if (!c) {
    indicate_test_fail();
    while (1)
      ;  // stall
  }
}
