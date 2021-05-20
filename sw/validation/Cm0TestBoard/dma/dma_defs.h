//*****************************************************************************
//
// Copyright (C) 2012 - 2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the
//  distribution.
//
//  Neither the name of Texas Instruments Incorporated nor the names of
//  its contributors may be used to endorse or promote products derived
//  from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************

/********************************************************************
 *
 * Standard register and bit definitions for the Texas Instruments
 * MSP430 microcontroller.
 *
 * This file supports assembler and C development for
 * MSP430FR5994 devices.
 *
 ********************************************************************/

/*
 * Modified by Sivert Sliper to only include DMA bit fields
 */

/* DMA0CTL Control Bits */
#define DMAREQ (0x0001) /* DMA request */
#define DMAREQ_L (0x0001)
#define DMAREQ_0 (0x0000) /* No DMA start */
#define DMAREQ_1 (0x0001) /* Start DMA */
#define DMAREQ_1_L (0x0001)
#define DMAABORT (0x0002) /* DMA abort */
#define DMAABORT_L (0x0002)
#define DMAABORT_0 (0x0000) /* DMA transfer not interrupted */
#define DMAABORT_1 (0x0002) /* DMA transfer interrupted by NMI */
#define DMAABORT_1_L (0x0002)
#define DMAIE (0x0004) /* DMA interrupt enable */
#define DMAIE_L (0x0004)
#define DMAIE_0 (0x0000) /* Disabled */
#define DMAIE_1 (0x0004) /* Enabled */
#define DMAIE_1_L (0x0004)
#define DMAIE__DISABLE (0x0000) /* Disabled */
#define DMAIE__ENABLE (0x0004)  /* Enabled */
#define DMAIE__ENABLE_L (0x0004)
#define DMAIFG (0x0008) /* DMA interrupt flag */
#define DMAIFG_L (0x0008)
#define DMAIFG_0 (0x0000) /* No interrupt pending */
#define DMAIFG_1 (0x0008) /* Interrupt pending */
#define DMAIFG_1_L (0x0008)
#define DMAEN (0x0010) /* DMA enable */
#define DMAEN_L (0x0010)
#define DMAEN_0 (0x0000) /* Disabled */
#define DMAEN_1 (0x0010) /* Enabled */
#define DMAEN_1_L (0x0010)
#define DMAEN__DISABLE (0x0000) /* Disabled */
#define DMAEN__ENABLE (0x0010)  /* Enabled */
#define DMAEN__ENABLE_L (0x0010)
#define DMALEVEL (0x0020) /* DMA level */
#define DMALEVEL_L (0x0020)
#define DMALEVEL_0 (0x0000) /* Edge sensitive (rising edge) */
#define DMALEVEL_1 (0x0020) /* Level sensitive (high level) */
#define DMALEVEL_1_L (0x0020)
#define DMALEVEL__EDGE (0x0000)  /* Edge sensitive (rising edge) */
#define DMALEVEL__LEVEL (0x0020) /* Level sensitive (high level) */
#define DMALEVEL__LEVEL_L (0x0020)
#define DMASRCBYTE (0x0040) /* DMA source byte */
#define DMASRCBYTE_L (0x0040)
#define DMASRCBYTE_0 (0x0000) /* Word */
#define DMASRCBYTE_1 (0x0040) /* Byte */
#define DMASRCBYTE_1_L (0x0040)
#define DMASRCBYTE__WORD (0x0000) /* Word */
#define DMASRCBYTE__BYTE (0x0040) /* Byte */
#define DMASRCBYTE__BYTE_L (0x0040)
#define DMADSTBYTE (0x0080) /* DMA destination byte */
#define DMADSTBYTE_L (0x0080)
#define DMADSTBYTE_0 (0x0000) /* Word */
#define DMADSTBYTE_1 (0x0080) /* Byte */
#define DMADSTBYTE_1_L (0x0080)
#define DMADSTBYTE__WORD (0x0000) /* Word */
#define DMADSTBYTE__BYTE (0x0080) /* Byte */
#define DMADSTBYTE__BYTE_L (0x0080)
#define DMASRCINCR (0x0300) /* DMA source increment */
#define DMASRCINCR_H (0x0003)
#define DMASRCINCR0 (0x0100) /* DMA source increment */
#define DMASRCINCR0_H (0x0001)
#define DMASRCINCR1 (0x0200) /* DMA source increment */
#define DMASRCINCR1_H (0x0002)
#define DMASRCINCR_0 (0x0000) /* Source address is unchanged */
#define DMASRCINCR_1 (0x0100) /* Source address is unchanged */
#define DMASRCINCR_1_H (0x0001)
#define DMASRCINCR_2 (0x0200) /* Source address is decremented */
#define DMASRCINCR_2_H (0x0002)
#define DMASRCINCR_3 (0x0300) /* Source address is incremented */
#define DMASRCINCR_3_H (0x0003)
#define DMADSTINCR (0x0c00) /* DMA destination increment */
#define DMADSTINCR_H (0x000c)
#define DMADSTINCR0 (0x0400) /* DMA destination increment */
#define DMADSTINCR0_H (0x0004)
#define DMADSTINCR1 (0x0800) /* DMA destination increment */
#define DMADSTINCR1_H (0x0008)
#define DMADSTINCR_0 (0x0000) /* Destination address is unchanged */
#define DMADSTINCR_1 (0x0400) /* Destination address is unchanged */
#define DMADSTINCR_1_H (0x0004)
#define DMADSTINCR_2 (0x0800) /* Destination address is decremented */
#define DMADSTINCR_2_H (0x0008)
#define DMADSTINCR_3 (0x0c00) /* Destination address is incremented */
#define DMADSTINCR_3_H (0x000c)
#define DMADT (0x7000) /* DMA transfer mode */
#define DMADT_H (0x0070)
#define DMADT0 (0x1000) /* DMA transfer mode */
#define DMADT0_H (0x0010)
#define DMADT1 (0x2000) /* DMA transfer mode */
#define DMADT1_H (0x0020)
#define DMADT2 (0x4000) /* DMA transfer mode */
#define DMADT2_H (0x0040)
#define DMADT_0 (0x0000) /* Single transfer */
#define DMADT_1 (0x1000) /* Block transfer */
#define DMADT_1_H (0x0010)
#define DMADT_2 (0x2000) /* Burst-block transfer */
#define DMADT_2_H (0x0020)
#define DMADT_3 (0x3000) /* Burst-block transfer */
#define DMADT_3_H (0x0030)
#define DMADT_4 (0x4000) /* Repeated single transfer */
#define DMADT_4_H (0x0040)
#define DMADT_5 (0x5000) /* Repeated block transfer */
#define DMADT_5_H (0x0050)
#define DMADT_6 (0x6000) /* Repeated burst-block transfer */
#define DMADT_6_H (0x0060)
#define DMADT_7 (0x7000) /* Repeated burst-block transfer */
#define DMADT_7_H (0x0070)
