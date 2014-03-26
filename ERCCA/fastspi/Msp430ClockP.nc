//$Id: Msp430ClockP.nc,v 1.2 2009/08/31 23:29:06 mossmoss Exp $

/* "Copyright (c) 2000-2003 The Regents of the University of California.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement
 * is hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
 * OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 */

/**
 * @author Cory Sharp <cssharp@eecs.berkeley.edu>
 * @author Vlado Handziski <handzisk@tkn.tu-berlind.de>
 */

#include <Msp430DcoSpec.h>

#include "Msp430Timer.h"

module Msp430ClockP
{
  provides interface Init;
  provides interface Msp430ClockInit;
}
implementation
{
  MSP430REG_NORACE(IE1);
  MSP430REG_NORACE(TACTL);
  MSP430REG_NORACE(TAIV);
  MSP430REG_NORACE(TBCTL);
  MSP430REG_NORACE(TBIV);

  enum
  {
    ACLK_CALIB_PERIOD = 8,
    TARGET_DCO_DELTA = (TARGET_DCO_KHZ / ACLK_KHZ) * ACLK_CALIB_PERIOD,
  };


  command void Msp430ClockInit.defaultSetupDcoCalibrate()
  {
  
#ifdef HAVE_ROSC_P2_5
#warning "Enabling ROSC on P2.5"
#define DCOR_BIT DCOR
    // Enable P2.5 ROSC
    CLR_FLAG( P2IE,  1 << 5);
    CLR_FLAG( P2IFG, 1 << 5);
#ifdef __MSP430_2618__
    CLR_FLAG( P2REN, 1 << 5);
#endif
    SET_FLAG( P2OUT, 1 << 5);
    CLR_FLAG( P2DIR, 1 << 5);
    SET_FLAG( P2SEL, 1 << 5);
#else
#define DCOR_BIT 0
#endif

    // --- setup ---

    TACTL = TASSEL1 | MC1; // source SMCLK, continuous mode, everything else 0
    TBCTL = TBSSEL0 | MC1;

    // BCSCTL1:
    //   No HF XTAL
    //   LFXT1 has LF XTAL
    //   ACLK divisor is 1
    //   RSEL at default (mid-range)
    BCSCTL1 = XT2OFF | RSEL2;

    // BCSCTL2:
    //   MCLK from DCO
    //   MCLK divisor is 1
    //   SMCLK from DCO
    //   SMCLK divisor is 1
    //   Ext DCO resistor on P2.5
    BCSCTL2 = DCOR_BIT;

    TBCCTL0 = CM0;
   }
    
  command void Msp430ClockInit.defaultInitClocks()
  {
    // IE1.OFIE = 0; no interrupt for oscillator fault
    CLR_FLAG( IE1, OFIE );

    // BCSCTL1
    // .XT2OFF = 1; disable the external oscillator for SCLK and MCLK
    // .XTS = 0; set low frequency mode for LXFT1
    // .DIVA = 0; set the divisor on ACLK to 1
    // .RSEL, pre-calibrated, do not modify
    BCSCTL1 = XT2OFF | (BCSCTL1 & (RSEL2|RSEL1|RSEL0));

    // BCSCTL2
    // .SELM = 0; select DCOCLK as source for MCLK
    // .DIVM = 0; set the divisor of MCLK to 1
    // .SELS = 0; select DCOCLK as source for SCLK
    // .DIVS = 2; set the divisor of SCLK to 4
    // .DCOR = 0; select internal resistor for DCO
#ifdef MHZ_8
// MHZ_8 is defined in MSP430DcoSpec.h if SPI DMA isn't being used
#warning "Using MCLK/8 for SMCLK"
#ifdef FAST_SMCLK
#warning "Using FAST 4 MHz SMCLK"
    BCSCTL2 = DIVS0 | DCOR_BIT;
#else
#warning "Using DEFAULT 1 MHz SMCLK"
    BCSCTL2 = DIVS1 | DIVS0 | DCOR_BIT;
#endif

#else

// !MHZ_8
#warning "Using MCLK/4 for SMCLK"
#ifdef FAST_SMCLK
#warning "Using FAST 4 MHz SMCLK"
    BCSCTL2 = DCOR_BIT;
#else
#warning "Using DEFAULT 1 MHz SMCLK"
    BCSCTL2 = DIVS1 | DCOR_BIT;
#endif
#endif
  }

  command void Msp430ClockInit.defaultInitTimerA()
  {
    TAR = 0;

    // TACTL
    // .TACLGRP = 0; each TACL group latched independently
    // .CNTL = 0; 16-bit counter
    // .TASSEL = 2; source SMCLK as DCO/4
    // .ID = 0; input divisor of 1
    // .MC = 0; initially disabled
    // .TACLR = 0; reset timer A
    // .TAIE = 1; enable timer A interrupts
#ifdef FAST_SMCLK
    TACTL = TASSEL1 | ID1 | TAIE;
#else
    TACTL = TASSEL1 | TAIE;
#endif
  }

  command void Msp430ClockInit.defaultInitTimerB()
  {
    TBR = 0;

    // TBCTL
    // .TBCLGRP = 0; each TBCL group latched independently
    // .CNTL = 0; 16-bit counter
    // .TBSSEL = 1; source ACLK
    // .ID = 0; input divisor of 1
    // .MC = 0; initially disabled
    // .TBCLR = 0; reset timer B
    // .TBIE = 1; enable timer B interrupts
    TBCTL = TBSSEL0 | TBIE;
  }

  default event void Msp430ClockInit.setupDcoCalibrate()
  {
    call Msp430ClockInit.defaultSetupDcoCalibrate();
  }
  
  default event void Msp430ClockInit.initClocks()
  {
    call Msp430ClockInit.defaultInitClocks();
  }

  default event void Msp430ClockInit.initTimerA()
  {
    call Msp430ClockInit.defaultInitTimerA();
  }

  default event void Msp430ClockInit.initTimerB()
  {
    call Msp430ClockInit.defaultInitTimerB();
  }


  void startTimerA()
  {
    // TACTL.MC = 2; continuous mode
    TACTL = MC1 | (TACTL & ~(MC1|MC0));
  }

  void stopTimerA()
  {
    //TACTL.MC = 0; stop timer B
    TACTL = TACTL & ~(MC1|MC0);
  }

  void startTimerB()
  {
    // TBCTL.MC = 2; continuous mode
    TBCTL = MC1 | (TBCTL & ~(MC1|MC0));
  }

  void stopTimerB()
  {
    //TBCTL.MC = 0; stop timer B
    TBCTL = TBCTL & ~(MC1|MC0);
  }

  void set_dco_calib( int calib )
  {
    BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
    DCOCTL = calib & 0xff;
  }

  uint16_t test_calib_busywait_delta( int calib )
  {
    int8_t aclk_count = 2;
    uint16_t dco_prev = 0;
    uint16_t dco_curr = 0;

    set_dco_calib( calib );

    while( aclk_count-- > 0 )
    {
      TBCCR0 = TBR + ACLK_CALIB_PERIOD; // set next interrupt
      TBCCTL0 &= ~CCIFG; // clear pending interrupt
      while( (TBCCTL0 & CCIFG) == 0 ); // busy wait
      dco_prev = dco_curr;
      dco_curr = TAR;
    }

    return dco_curr - dco_prev;
  }

  // busyCalibrateDCO
  // Should take about 9ms if ACLK_CALIB_PERIOD=8.
  // DCOCTL and BCSCTL1 are calibrated when done.
  void busyCalibrateDco()
  {
    // --- variables ---
    int calib;
    int step;

    // --- calibrate ---

    // Binary search for RSEL,DCO,DCOMOD.
    // It's okay that RSEL isn't monotonic.

    for( calib=0,step=0x800; step!=0; step>>=1 )
    {
      // if the step is not past the target, commit it
      if( test_calib_busywait_delta(calib|step) <= TARGET_DCO_DELTA )
        calib |= step;
    }

    // if DCOx is 7 (0x0e0 in calib), then the 5-bit MODx is not useable, set it to 0
    if( (calib & 0x0e0) == 0x0e0 )
      calib &= ~0x01f;

    set_dco_calib( calib );
  }

  command error_t Init.init()
  {
    // Reset timers and clear interrupt vectors
    TACTL = TACLR;
    TAIV = 0;
    TBCTL = TBCLR;
    TBIV = 0;

    atomic
    {
#if 1
      signal Msp430ClockInit.setupDcoCalibrate();
      busyCalibrateDco();
#else
#warning "Skipping DCO cal"
#endif
      signal Msp430ClockInit.initClocks();
      signal Msp430ClockInit.initTimerA();
      signal Msp430ClockInit.initTimerB();
      startTimerA();
      startTimerB();
    }

    return SUCCESS;
  }
}

