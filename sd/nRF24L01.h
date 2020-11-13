/*
  Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
  Portions Copyright (C) 2011 Greg Copeland

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#define _BV(a) (1 << a)

/* Masks */
#define RW_COMMAND_MASK 0b11100000
#define RW_ADDRESS_MASK 0b00011111
#define WAP_COMMAND_MASK 0b11111000
#define WAP_PIPE_MASK 0b00000111

/* Registers */
#define NRF_CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define NRF_STATUS 0x07
#define OBSERVE_TX 0x08
#define RPD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

/* Bit Mnemonics */
#define MASK_RX_DR _BV(6)
#define MASK_TX_DS _BV(5)
#define MASK_MAX_RT _BV(4)
#define EN_CRC _BV(3)
#define CRCO _BV(2)
#define PWR_UP _BV(1)
#define PRIM_RX _BV(0)
#define ENAA_P5 _BV(5)
#define ENAA_P4 _BV(4)
#define ENAA_P3 _BV(3)
#define ENAA_P2 _BV(2)
#define ENAA_P1 _BV(1)
#define ENAA_P0 _BV(0)
#define ERX_P5 _BV(5)
#define ERX_P4 _BV(4)
#define ERX_P3 _BV(3)
#define ERX_P2 _BV(2)
#define ERX_P1 _BV(1)
#define ERX_P0 _BV(0)
#define AW_1 _BV(1)
#define AW_0 _BV(0)
#define ARD_3 _BV(7)
#define ARD_2 _BV(6)
#define ARD_1 _BV(5)
#define ARD_0 _BV(4)
#define ARC_3 _BV(3)
#define ARC_2 _BV(2)
#define ARC_1 _BV(1)
#define ARC_0 _BV(0)
#define CONT_WAVE _BV(7)
#define RF_DR_LOW _BV(5)
#define PLL_LOCK _BV(4)
#define RF_DR_HIGH _BV(3)
#define RF_PWR_1 _BV(2)
#define RF_PWR_0 _BV(1)
#define RX_DR _BV(6)
#define TX_DS _BV(5)
#define MAX_RT _BV(4)
#define RX_P_NO_2 _BV(3)
#define RX_P_NO_1 _BV(2)
#define RX_P_NO_0 _BV(1)
#define TX_FULL _BV(0)
#define PLOS_CNT _BV(4)
#define ARC_CNT _BV(0)
#define TX_REUSE _BV(6)
#define FIFO_FULL _BV(5)
#define TX_EMPTY _BV(4)
#define RX_FULL _BV(1)
#define RX_EMPTY _BV(0)
#define DPL_P5 _BV(5)
#define DPL_P4 _BV(4)
#define DPL_P3 _BV(3)
#define DPL_P2 _BV(2)
#define DPL_P1 _BV(1)
#define DPL_P0 _BV(0)
#define EN_DPL _BV(2)
#define EN_ACK_PAY _BV(1)
#define EN_DYN_ACK _BV(0)

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define RF24_NOP 0xFF

/* Non-P omissions */
#define LNA_HCURR 0

/* P model memory Map */
#define RPD 0x09
#define W_TX_PAYLOAD_NO_ACK 0xB0
