/*
 * Copyright (c) 2018, Bruno Kessler Foundation, Trento, Italy and
 * ETH, IIS, Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * \file
 *         SX1276 LoRa Radio Driver (Modified)
 * \author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 */
#include "contiki.h"
#include <msp430.h>
#include "sx1276.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"
#include "spi.h"
#include "isr_compat.h"
#include "sys/clock.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif
/*---------------------------------------------------------------------------*/

static RadioEvents_t *RadioEvents;
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];
sx1276_t sx1276;

/*---------------------------------------------------------------------------*/
#define DIO0  ((P1IN & BIT3) != 0)
/*
 * Enable/Disable DIO interrupt
 */
#define ENABLE_DIO0_IT\
  do {\
    sx1276_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);\
    P1IFG &= ~BIT3;\
    P1IE |= BIT3;\
  } while(0)
#define DISABLE_DIO0_IT (P1IE &= ~BIT3)
/*---------------------------------------------------------------------------*/

typedef struct {
    radio_modem_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
} radio_registers_t;

const radio_registers_t radio_registers[] = {
    { MODEM_FSK , REG_LNA                , 0x23 },
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },
    { MODEM_FSK , REG_AFCFEI             , 0x01 },
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },
    { MODEM_FSK , REG_OSC                , 0x07 },
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },
};

typedef struct {
    uint32_t bandwidth;
    uint8_t  RegValue;
} fsk_bandwidth_t;

const fsk_bandwidth_t fsk_bandwidths[] = {
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};

// static radio_events_t *RadioEvents;

#define RF_MID_BAND_THRESH                          525000000

static uint8_t sx1276_get_paselect(uint32_t channel) {
  if(channel < RF_MID_BAND_THRESH ) 
  {
    return RF_PACONFIG_PASELECT_PABOOST;
  } 
  else 
  {
    return RF_PACONFIG_PASELECT_PABOOST;
  }
}

static uint8_t sx1276_get_fsk_bandwidthregvalue(uint32_t bandwidth) {
  uint8_t i;

  for(i = 0; i < (sizeof(fsk_bandwidths) / sizeof(fsk_bandwidth_t)) - 1; i++) 
  {
    if((bandwidth >= fsk_bandwidths[i].bandwidth) && (bandwidth < fsk_bandwidths[i + 1].bandwidth)) 
    {
      return fsk_bandwidths[i].RegValue;
    }
  }
  // ERROR: Value not found
  while( 1 );
}

void sx1276_init(RadioEvents_t *events) {

  RadioEvents = events;

  // Initialize driver timeout timers
  //TimerInit( &TxTimeoutTimer, SX1276OnTimeoutIrq );
  //TimerInit( &RxTimeoutTimer, SX1276OnTimeoutIrq );
  //TimerInit( &RxTimeoutSyncWord, SX1276OnTimeoutIrq );

    // Setting the NRESET pin up defined in spi.c
  P1DIR |= RESET;
  P1OUT |= RESET;
  // Waiting 10ms (datasheet)
  delay_ms(10);

  // Init irq interrupts DIO_0 on Port1 Bit 3
  dio0irq_init();

  sx1276_reset();

  sx1276_rxchain_calibration();

  sx1276_set_opmode(RF_OPMODE_SLEEP);

  eint(); // __enable_interrupt();
  
  uint8_t i; 

  for(i = 0; i < sizeof(radio_registers) / sizeof(radio_registers_t); i++) 
  {
    sx1276_set_modem(radio_registers[i].Modem);
    sx1276_write(radio_registers[i].Addr, radio_registers[i].Value);
  }

  sx1276_set_modem(MODEM_FSK);

  sx1276.Settings.State = RF_IDLE;
}

void sx1276_reset() {

  P1OUT &= ~RESET;
  delay_ms(1); // Wait 1 ms
  P1OUT |= RESET;
  delay_ms(6); // Wait 6 ms
}


void sx1276_rxchain_calibration() {
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = sx1276_read(REG_PACONFIG);
    initialFreq = (double)(   ((uint32_t) sx1276_read(REG_FRFMSB) << 16) |
                              ((uint32_t) sx1276_read(REG_FRFMID) << 8 ) |
                              ((uint32_t) sx1276_read(REG_FRFLSB))) * (double) FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    sx1276_write(REG_PACONFIG, 0x00);

    // Launch Rx chain calibration for LF band
    sx1276_write(REG_IMAGECAL, (sx1276_read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while((sx1276_read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) 
    {
    }

    // Sets a Frequency in HF band
    sx1276_set_channel(868000000);

    // Launch Rx chain calibration for HF band
    sx1276_write(REG_IMAGECAL, (sx1276_read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while((sx1276_read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) 
    {
    }

    // Restore context
    sx1276_write(REG_PACONFIG, regPaConfigInitVal);
    sx1276_set_channel(initialFreq);
}

void sx1276_set_rxconfig(radio_modem_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous) 
{
  sx1276_set_modem(modem);

  switch( modem ) 
  {
    case MODEM_FSK: 
        {
          sx1276.Settings.Fsk.Bandwidth =     bandwidth;
          sx1276.Settings.Fsk.Datarate =      datarate;
          sx1276.Settings.Fsk.BandwidthAfc =  bandwidthAfc;
          sx1276.Settings.Fsk.FixLen =        fixLen;
          sx1276.Settings.Fsk.PayloadLen =    payloadLen;
          sx1276.Settings.Fsk.CrcOn =         crcOn;
          sx1276.Settings.Fsk.IqInverted =    iqInverted;
          sx1276.Settings.Fsk.RxContinuous =  rxContinuous;
          sx1276.Settings.Fsk.PreambleLen =   preambleLen;

          datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
          sx1276_write(REG_BITRATEMSB, (uint8_t) (datarate >> 8));
          sx1276_write(REG_BITRATELSB, (uint8_t) (datarate & 0xFF));

          sx1276_write(REG_RXBW,  sx1276_get_fsk_bandwidthregvalue(bandwidth));
          sx1276_write(REG_AFCBW, sx1276_get_fsk_bandwidthregvalue(bandwidthAfc));

          sx1276_write(REG_PREAMBLEMSB, (uint8_t) ((preambleLen >> 8) & 0xFF));
          sx1276_write(REG_PREAMBLELSB, (uint8_t) (preambleLen & 0xFF));

          if(fixLen == 1) {
              sx1276_write(REG_PAYLOADLENGTH, payloadLen);
          } else {
              sx1276_write(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
          }

          sx1276_write(REG_PACKETCONFIG1,
                         (sx1276_read(REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                         ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                         (crcOn << 4));
        }
        break;

      case MODEM_OOK:
        {
          sx1276.Settings.Ook.Bandwidth =     bandwidth;
          sx1276.Settings.Ook.Datarate =      datarate;
          sx1276.Settings.Ook.BandwidthAfc =  bandwidthAfc;
          sx1276.Settings.Ook.FixLen =        fixLen;
          sx1276.Settings.Ook.PayloadLen =    payloadLen;
          sx1276.Settings.Ook.CrcOn =         crcOn;
          sx1276.Settings.Ook.IqInverted =    iqInverted;
          sx1276.Settings.Ook.RxContinuous =  rxContinuous;
          sx1276.Settings.Ook.PreambleLen =   preambleLen;

          datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
          sx1276_write(REG_BITRATEMSB, (uint8_t) (datarate >> 8));
          sx1276_write(REG_BITRATELSB, (uint8_t) (datarate & 0xFF));

          sx1276_write(REG_RXBW,  sx1276_get_fsk_bandwidthregvalue(bandwidth));
          sx1276_write(REG_AFCBW, sx1276_get_fsk_bandwidthregvalue(bandwidthAfc));

          sx1276_write(REG_PREAMBLEMSB, (uint8_t) ((preambleLen >> 8) & 0xFF));
          sx1276_write(REG_PREAMBLELSB, (uint8_t) (preambleLen & 0xFF));

          if(fixLen == 1) {
              sx1276_write(REG_PAYLOADLENGTH, payloadLen);
          } else {
              sx1276_write(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
          }

          sx1276_write(REG_PACKETCONFIG1,
                         (sx1276_read(REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                         ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                         (crcOn << 4));
        }
        break;

    case MODEM_LORA: 
        {
          if( bandwidth > 2 ) 
          {
              // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
              while( 1 );
          }
          bandwidth += 7;

          sx1276.Settings.LoRa.Bandwidth =     bandwidth;
          sx1276.Settings.LoRa.Datarate =      datarate;
          sx1276.Settings.LoRa.Coderate =      coderate;
          sx1276.Settings.LoRa.PreambleLen =   preambleLen;
          sx1276.Settings.LoRa.FixLen =        fixLen;
          sx1276.Settings.LoRa.PayloadLen =    payloadLen;
          sx1276.Settings.LoRa.CrcOn =         crcOn;
          sx1276.Settings.LoRa.FreqHopOn =     freqHopOn;
          sx1276.Settings.LoRa.HopPeriod =     hopPeriod;
          sx1276.Settings.LoRa.IqInverted =    iqInverted;
          sx1276.Settings.LoRa.RxContinuous =  rxContinuous;

          if(datarate > 12) 
          {
            datarate = 12;
          }     
          else if(datarate < 6) 
          {
              datarate = 6;
          }

          if(((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) || ((bandwidth == 8) && (datarate == 12))) 
          {
              sx1276.Settings.LoRa.LowDatarateOptimize = 0x01;
          } 
          else 
          {
              sx1276.Settings.LoRa.LowDatarateOptimize = 0x00;
          }

          sx1276_write(REG_LR_MODEMCONFIG1,
                         (sx1276_read(REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                         (bandwidth << 4) | (coderate << 1) |
                         fixLen);

          sx1276_write(REG_LR_MODEMCONFIG2,
                         (sx1276_read(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
                         (datarate << 4) | (crcOn << 2) |
                         ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

          sx1276_write(REG_LR_MODEMCONFIG3,
                         (sx1276_read(REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                         (sx1276.Settings.LoRa.LowDatarateOptimize << 3));

          sx1276_write(REG_LR_SYMBTIMEOUTLSB, (uint8_t) (symbTimeout & 0xFF));

          sx1276_write(REG_LR_PREAMBLEMSB, (uint8_t) ((preambleLen >> 8 ) & 0xFF));
          sx1276_write(REG_LR_PREAMBLELSB, (uint8_t) ((preambleLen      ) & 0xFF));

          if(fixLen == 1) 
            {
              sx1276_write(REG_LR_PAYLOADLENGTH, payloadLen);
            }


          if(sx1276.Settings.LoRa.FreqHopOn == true ) 
          {
              sx1276_write(REG_LR_PLLHOP, (sx1276_read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
              sx1276_write(REG_LR_HOPPERIOD, sx1276.Settings.LoRa.HopPeriod);
          }

          if((bandwidth == 9) && (sx1276.Settings.Channel > RF_MID_BAND_THRESH)) 
          {
              // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
              sx1276_write(REG_LR_TEST36, 0x02);
              sx1276_write(REG_LR_TEST3A, 0x64);
          } 
          else if(bandwidth == 9) 
          {
              // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
              sx1276_write(REG_LR_TEST36, 0x02);
              sx1276_write(REG_LR_TEST3A, 0x7F);
          } 
          else 
          {
              // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
              sx1276_write(REG_LR_TEST36, 0x03);
          }

          if( datarate == 6 ) 
          {
              sx1276_write(REG_LR_DETECTOPTIMIZE,
                             (sx1276_read(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                             RFLR_DETECTIONOPTIMIZE_SF6);
              sx1276_write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
          } 
          else 
          {
              sx1276_write(REG_LR_DETECTOPTIMIZE,
                           (sx1276_read(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                           RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
              sx1276_write( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
          }
        }
        break;
  }
}

void sx1276_set_txconfig(radio_modem_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout) 
{
  //setting RF TX Power
  uint8_t paConfig = 0;
  uint8_t paDac = 0;

  sx1276_set_modem(modem);

  paConfig = sx1276_read(REG_PACONFIG);
  paDac =    sx1276_read(REG_PADAC);

  paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK)  | sx1276_get_paselect(sx1276.Settings.Channel);
  paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK) | 0x70;

  if((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST) 
  {
    if(power > 17) 
      { 
        paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
      }
    else
      {           
        paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
      }

    if((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) 
    {
      if(power < 5)  
        {
          power = 5;
        }
      if(power > 20) 
        {
          power = 20;
        }
      paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t) ((uint16_t) (power - 5) & 0x0F);
    } 
    else 
    {
      if( power < 2 ) 
        {
          power = 2;
        }
      if( power > 17 ) 
        {
          power = 17;
        }
      paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t) ((uint16_t) (power - 2) & 0x0F);
    }
  } 
  else 
  {
    if( power < -1 ) 
      {
        power = -1;
      }
    if( power > 14 ) 
      {
        power = 14;
      }
    paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t) ((uint16_t) (power + 1) & 0x0F);
  }

  sx1276_write(REG_PACONFIG, paConfig);
  sx1276_write(REG_PADAC, paDac);

  switch(modem) 
  {
    case MODEM_FSK: 
        {
          sx1276.Settings.Fsk.Power =        power;
          sx1276.Settings.Fsk.Fdev =         fdev;
          sx1276.Settings.Fsk.Bandwidth =    bandwidth;
          sx1276.Settings.Fsk.Datarate =     datarate;
          sx1276.Settings.Fsk.PreambleLen =  preambleLen;
          sx1276.Settings.Fsk.FixLen =       fixLen;
          sx1276.Settings.Fsk.CrcOn =        crcOn;
          sx1276.Settings.Fsk.IqInverted =   iqInverted;
          sx1276.Settings.Fsk.TxTimeout =    timeout;

          fdev = (uint16_t) ((double) fdev / (double) FREQ_STEP);
          sx1276_write(REG_FDEVMSB, (uint8_t) (fdev >> 8));
          sx1276_write(REG_FDEVLSB, (uint8_t) (fdev & 0xFF));

          datarate = (uint16_t) ((double) XTAL_FREQ / (double) datarate);
          sx1276_write(REG_BITRATEMSB, (uint8_t) (datarate >> 8));
          sx1276_write(REG_BITRATELSB, (uint8_t) (datarate & 0xFF));

          sx1276_write(REG_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
          sx1276_write(REG_PREAMBLELSB, (preambleLen     ) & 0xFF);

          sx1276_write(REG_PACKETCONFIG1,
                         (sx1276_read(REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                         ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                         (crcOn << 4));

        }
        break;

    case MODEM_OOK: 
        {
          sx1276.Settings.Ook.Power =        power;
          sx1276.Settings.Ook.Fdev =         fdev;
          sx1276.Settings.Ook.Bandwidth =    bandwidth;
          sx1276.Settings.Ook.Datarate =     datarate;
          sx1276.Settings.Ook.PreambleLen =  preambleLen;
          sx1276.Settings.Ook.FixLen =       fixLen;
          sx1276.Settings.Ook.CrcOn =        crcOn;
          sx1276.Settings.Ook.IqInverted =   iqInverted;
          sx1276.Settings.Ook.TxTimeout =    timeout;
          //setting OOK modulation mode for transmitting wake-up signal
          uint8_t d = RF_OPMODE_LONGRANGEMODE_OFF
              + RF_OPMODE_SLEEP
              + ((modem == MODEM_FSK) ? RF_OPMODE_MODULATIONTYPE_FSK : RF_OPMODE_MODULATIONTYPE_OOK)
              + RFLR_OPMODE_FREQMODE_ACCESS_HF;
          sx1276_write(REG_OPMODE, d);


          datarate = (uint16_t) ((double) XTAL_FREQ / (double) datarate);
          sx1276_write(REG_BITRATEMSB, (uint8_t) (datarate >> 8));
          sx1276_write(REG_BITRATELSB, (uint8_t) (datarate & 0xFF));

          sx1276_write(REG_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
          sx1276_write(REG_PREAMBLELSB, (preambleLen     ) & 0xFF);

          sx1276_write(REG_PACKETCONFIG1,
                         (sx1276_read(REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                         ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                         (crcOn << 4));

        }
        break;

    case MODEM_LORA: 
        {
          sx1276.Settings.LoRa.Power = power;

          if( bandwidth > 2 ) 
          {
            // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            while( 1 );
          }
          bandwidth += 7;

          sx1276.Settings.LoRa.Bandwidth =    bandwidth;
          sx1276.Settings.LoRa.Datarate =     datarate;
          sx1276.Settings.LoRa.Coderate =     coderate;
          sx1276.Settings.LoRa.PreambleLen =  preambleLen;
          sx1276.Settings.LoRa.FixLen =       fixLen;
          sx1276.Settings.LoRa.FreqHopOn =    freqHopOn;
          sx1276.Settings.LoRa.HopPeriod =    hopPeriod;
          sx1276.Settings.LoRa.CrcOn =        crcOn;
          sx1276.Settings.LoRa.IqInverted =   iqInverted;
          sx1276.Settings.LoRa.TxTimeout =    timeout;

          if(datarate > 12)
          {
            datarate = 12;
          }
          else if(datarate < 6)
          { 
            datarate = 6;
          }

          if(((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) || ((bandwidth == 8) && (datarate == 12))) 
          {
            sx1276.Settings.LoRa.LowDatarateOptimize = 0x01;
          } 
          else 
          {
            sx1276.Settings.LoRa.LowDatarateOptimize = 0x00;
          }

          if(sx1276.Settings.LoRa.FreqHopOn == true) 
          {
            sx1276_write(REG_LR_PLLHOP, (sx1276_read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            sx1276_write(REG_LR_HOPPERIOD, sx1276.Settings.LoRa.HopPeriod);
          }

          sx1276_write(REG_LR_MODEMCONFIG1,
                         (sx1276_read(REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK &  RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                         (bandwidth << 4) | (coderate << 1) |
                         fixLen);

          sx1276_write(REG_LR_MODEMCONFIG2,
                         (sx1276_read(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) |
                         (datarate << 4) | (crcOn << 2));

          sx1276_write(REG_LR_MODEMCONFIG3,
                         (sx1276_read( REG_LR_MODEMCONFIG3 ) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                         (sx1276.Settings.LoRa.LowDatarateOptimize << 3));

          sx1276_write(REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
          sx1276_write(REG_LR_PREAMBLELSB, (preambleLen     ) & 0xFF);

          if(datarate == 6) 
          {
            sx1276_write(REG_LR_DETECTOPTIMIZE,
                           (sx1276_read(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                           RFLR_DETECTIONOPTIMIZE_SF6);
            sx1276_write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
          } 
          else 
          {
            sx1276_write(REG_LR_DETECTOPTIMIZE,
                         (sx1276_read(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                         RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            sx1276_write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
          }
        }
        break;
  }
}

uint32_t sx1276_get_timeonair(radio_modem_t modem, uint8_t pktLen) {
  uint32_t airTime = 0;
  switch( modem ) 
  {
    case MODEM_FSK: 
    case MODEM_OOK:
        {
        // airTime = round((8 * (sx1276.Settings.Fsk.PreambleLen +
        //                          ((sx1276_read(REG_SYNCCONFIG) & ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1) +
        //                          ((sx1276.Settings.Fsk.FixLen == 0x01) ? 0.0 : 1.0) +
        //                          (((sx1276_read( REG_PACKETCONFIG1) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK) != 0x00) ? 1.0 : 0) +
        //                          pktLen +
        //                          ((sx1276.Settings.Fsk.CrcOn == 0x01) ? 2.0 : 0)) / sx1276.Settings.Fsk.Datarate) * 1e3);
        }
        break;
    case MODEM_LORA: 
        {
          double bw = 0.0;
          // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
          switch(sx1276.Settings.LoRa.Bandwidth) {
            case 7: // 125 kHz
              bw = 125e3;
              break;
            case 8: // 250 kHz
              bw = 250e3;
              break;
            case 9: // 500 kHz
              bw = 500e3;
              break;
          }

          // Symbol rate : time for one symbol (secs)
          double rs = bw / (1 << sx1276.Settings.LoRa.Datarate);
          double ts = 1 / rs;
          // time of preamble
          double tPreamble = (sx1276.Settings.LoRa.PreambleLen + 4.25) * ts;
          // Symbol length of payload and time
          double tmp = ceil((8 * pktLen - 4 * sx1276.Settings.LoRa.Datarate +
                               28 + 16 * sx1276.Settings.LoRa.CrcOn -
                               ( sx1276.Settings.LoRa.FixLen ? 20 : 0 ) ) /
                               ( double )( 4 * sx1276.Settings.LoRa.Datarate -
                               ( ( sx1276.Settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) *
                               ( sx1276.Settings.LoRa.Coderate + 4 );
          double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
          double tPayload = nPayload * ts;
          // Time on air
          double tOnAir = tPreamble + tPayload;
          // return us secs
          airTime = floor( tOnAir * 1e3 + 0.999 );
        }
        break;
  }
  return airTime;
}

void sx1276_send(uint8_t *buffer, uint8_t size) {
  uint32_t txTimeout = 0;

  DISABLE_DIO0_IT;

  switch( sx1276.Settings.Modem ) 
  {
    case MODEM_FSK:
        {
          sx1276.Settings.FskPacketHandler.NbBytes = 0;
          sx1276.Settings.FskPacketHandler.Size = size;

          if(sx1276.Settings.Fsk.FixLen == false) 
          {
            sx1276_write_fifo((uint8_t*) &size, 1);
          } 
          else 
          {
            sx1276_write(REG_PAYLOADLENGTH, size);
          }

          if((size > 0) && (size <= 64)) 
          {
            sx1276.Settings.FskPacketHandler.ChunkSize = size;
          } 
          else 
          {
            memcpy(RxTxBuffer, buffer, size);
            sx1276.Settings.FskPacketHandler.ChunkSize = 32;
          }

          // Write payload buffer
          sx1276_write_fifo(buffer, sx1276.Settings.FskPacketHandler.ChunkSize);
          sx1276.Settings.FskPacketHandler.NbBytes += sx1276.Settings.FskPacketHandler.ChunkSize;
          txTimeout = sx1276.Settings.Fsk.TxTimeout;
        }
        break;

    case MODEM_OOK: 
        {

          sx1276.Settings.OokPacketHandler.Size = size;

          sx1276_set_opmode(RF_OPMODE_STANDBY);

          delay_ms(1);

          if(sx1276.Settings.Ook.FixLen == true) 
          {

            sx1276_write(REG_PAYLOADLENGTH, size);
          }
          else
          {
            sx1276_write(REG_FIFO, size);
          }
          // for (i = 0; i < size; i++)
          // {
          //   sx1276_write(REG_FIFO, buffer[i]);
          // }

          sx1276_write_fifo( buffer, size );

          sx1276_set_opmode(RF_OPMODE_TRANSMITTER);
          txTimeout = sx1276.Settings.Ook.TxTimeout;

        }
        break;

    case MODEM_LORA: 
        {
          if( sx1276.Settings.LoRa.IqInverted == true ) 
          {
            sx1276_write(REG_LR_INVERTIQ, ((sx1276_read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
            sx1276_write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
          } 
          else 
          {
            sx1276_write(REG_LR_INVERTIQ, ((sx1276_read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
            sx1276_write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
          }

          sx1276.Settings.LoRaPacketHandler.Size = size;

          // Initializes the payload size
          sx1276_write(REG_LR_PAYLOADLENGTH, size);

          // Full buffer used for Tx
          sx1276_write(REG_LR_FIFOTXBASEADDR, 0);
          sx1276_write(REG_LR_FIFOADDRPTR, 0);

          // FIFO operations can not take place in Sleep mode
          if((sx1276_read(REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP) 
          {
            sx1276_set_opmode(RF_OPMODE_STANDBY);
            sx1276.Settings.State = RF_IDLE;

            delay_ms(1);

          }
          // Write payload buffer
          sx1276_write_fifo( buffer, size );
          txTimeout = sx1276.Settings.LoRa.TxTimeout;
        }
        break;
  }

  sx1276_set_tx( txTimeout );
}

void sx1276_set_sleep(void)
{
    // TimerStop( &RxTimeoutTimer );
    // TimerStop( &TxTimeoutTimer );
  PRINTF("Sx1276 entering sleep mode\n");
  DISABLE_DIO0_IT;
  sx1276_set_opmode(RF_OPMODE_SLEEP);
  sx1276.Settings.State = RF_IDLE;
  delay_ms(1);
}

void sx1276_disable_sync_word(void)
{
  uint8_t r = sx1276_read(REG_SYNCCONFIG);
  r &= ~RF_SYNCCONFIG_SYNC_ON;
  sx1276_write(REG_SYNCCONFIG, r);
}

void sx1276_set_rx(uint32_t timeout) {

  bool rxContinuous = false;

  switch( sx1276.Settings.Modem ) 
  {
    case MODEM_FSK: 
    case MODEM_OOK:
        {
          rxContinuous = sx1276.Settings.Fsk.RxContinuous;

          // DIO0=PayloadReady
          // DIO1=FifoLevel
          // DIO2=SyncAddr
          // DIO3=FifoEmpty
          // DIO4=Preamble
          // DIO5=ModeReady
          sx1276_write(REG_DIOMAPPING1, (sx1276_read(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK &
                                                                         RF_DIOMAPPING1_DIO1_MASK &
                                                                         RF_DIOMAPPING1_DIO2_MASK) |
                                                                         RF_DIOMAPPING1_DIO0_00 |
                                                                         RF_DIOMAPPING1_DIO1_00 |
                                                                         RF_DIOMAPPING1_DIO2_11);

          sx1276_write(REG_DIOMAPPING2, (sx1276_read(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK &
                                                                         RF_DIOMAPPING2_MAP_MASK) |
                                                                         RF_DIOMAPPING2_DIO4_11 |
                                                                         RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

          sx1276.Settings.FskPacketHandler.FifoThresh = sx1276_read(REG_FIFOTHRESH) & 0x3F;

          sx1276_write(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

          sx1276.Settings.FskPacketHandler.PreambleDetected = false;
          sx1276.Settings.FskPacketHandler.SyncWordDetected = false;
          sx1276.Settings.FskPacketHandler.NbBytes = 0;
          sx1276.Settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA: 
        {
          if(sx1276.Settings.LoRa.IqInverted == true) 
          {
              sx1276_write(REG_LR_INVERTIQ, ((sx1276_read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
              sx1276_write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
          } 
          else 
          {
              sx1276_write(REG_LR_INVERTIQ, ((sx1276_read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
              sx1276_write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
          }

          // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
          if(sx1276.Settings.LoRa.Bandwidth < 9) 
          {
              sx1276_write(REG_LR_DETECTOPTIMIZE, sx1276_read(REG_LR_DETECTOPTIMIZE) & 0x7F );
              sx1276_write(REG_LR_TEST30, 0x00);
              switch(sx1276.Settings.LoRa.Bandwidth) 
              {
                case 0: // 7.8 kHz
                    sx1276_write( REG_LR_TEST2F, 0x48 );
                    sx1276_set_channel(sx1276.Settings.Channel + 7810 );
                    break;
                case 1: // 10.4 kHz
                    sx1276_write( REG_LR_TEST2F, 0x44 );
                    sx1276_set_channel(sx1276.Settings.Channel + 10420 );
                    break;
                case 2: // 15.6 kHz
                    sx1276_write( REG_LR_TEST2F, 0x44 );
                    sx1276_set_channel(sx1276.Settings.Channel + 15620 );
                    break;
                case 3: // 20.8 kHz
                    sx1276_write( REG_LR_TEST2F, 0x44 );
                    sx1276_set_channel(sx1276.Settings.Channel + 20830 );
                    break;
                case 4: // 31.2 kHz
                    sx1276_write( REG_LR_TEST2F, 0x44 );
                    sx1276_set_channel(sx1276.Settings.Channel + 31250 );
                    break;
                case 5: // 41.4 kHz
                    sx1276_write( REG_LR_TEST2F, 0x44 );
                    sx1276_set_channel(sx1276.Settings.Channel + 41670 );
                    break;
                case 6: // 62.5 kHz
                    sx1276_write( REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    sx1276_write( REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    sx1276_write( REG_LR_TEST2F, 0x40 );
                    break;
              }
          } 
          else 
          {
              sx1276_write(REG_LR_DETECTOPTIMIZE, sx1276_read(REG_LR_DETECTOPTIMIZE) | 0x80);
          }

          rxContinuous = sx1276.Settings.LoRa.RxContinuous;

          /************************************************/
          // sx1276_on_dio0irq();
          PRINTF("DIO0 interrupt called\n\r");
          /************************************************/

          if(sx1276.Settings.LoRa.FreqHopOn == true) 
          {
              sx1276_write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                //RFLR_IRQFLAGS_RXDONE |
                                                //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                RFLR_IRQFLAGS_VALIDHEADER |
                                                RFLR_IRQFLAGS_TXDONE |
                                                RFLR_IRQFLAGS_CADDONE |
                                                //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                RFLR_IRQFLAGS_CADDETECTED );

              // DIO0=RxDone, DIO2=FhssChangeChannel
              sx1276_write(REG_DIOMAPPING1, (sx1276_read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
          } 
          else 
          {
              sx1276_write(REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                               //RFLR_IRQFLAGS_RXDONE |
                                               //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                               RFLR_IRQFLAGS_VALIDHEADER |
                                               RFLR_IRQFLAGS_TXDONE |
                                               RFLR_IRQFLAGS_CADDONE |
                                               RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                               RFLR_IRQFLAGS_CADDETECTED);

              // DIO0=RxDone
              sx1276_write(REG_DIOMAPPING1, (sx1276_read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);
          }
          sx1276_write(REG_LR_FIFORXBASEADDR, 0);
          sx1276_write(REG_LR_FIFOADDRPTR, 0);
        }
        break;
  }

  memset(RxTxBuffer, 0, (size_t) RX_BUFFER_SIZE );

  sx1276.Settings.State = RF_RX_RUNNING;
  if(timeout != 0) 
  {
    //TimerSetValue( &RxTimeoutTimer, timeout );
    //TimerStart( &RxTimeoutTimer );
  }

  if(sx1276.Settings.Modem == MODEM_FSK) 
  {
    sx1276_set_opmode(RF_OPMODE_RECEIVER);

    if(rxContinuous == false) 
    {
      //TimerSetValue( &RxTimeoutSyncWord, ceil( ( 8.0 * ( sx1276.Settings.Fsk.PreambleLen +
      //                                                 ( ( SX1276Read( REG_SYNCCONFIG ) &
      //                                                    ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
      //                                                    1.0 ) + 10.0 ) /
      //                                                 ( double )sx1276.Settings.Fsk.Datarate ) * 1e3 ) + 4 );
      //TimerStart( &RxTimeoutSyncWord );
    }
  } 
  else 
  {
    if(rxContinuous == true) 
    {
      ENABLE_DIO0_IT;
      sx1276_set_opmode(RFLR_OPMODE_RECEIVER);
    } 
    else 
    {
      ENABLE_DIO0_IT;
      sx1276_set_opmode(RFLR_OPMODE_RECEIVER_SINGLE);
    }
  }
}

void sx1276_set_tx(uint32_t timeout) {
  //TimerSetValue( &TxTimeoutTimer, timeout );

  switch( sx1276.Settings.Modem ) 
  {
    case MODEM_FSK: 
        {
          // DIO0=PacketSent
          // DIO1=FifoEmpty
          // DIO2=FifoFull
          // DIO3=FifoEmpty
          // DIO4=LowBat
          // DIO5=ModeReady
          sx1276_write(REG_DIOMAPPING1, (sx1276_read(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) | RF_DIOMAPPING1_DIO1_01);
          sx1276_write(REG_DIOMAPPING2, (sx1276_read(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK));
          sx1276.Settings.FskPacketHandler.FifoThresh = sx1276_read( REG_FIFOTHRESH ) & 0x3F;
        }
        break;

    case MODEM_OOK: 
        {
          // DIO0=PacketSent
          // DIO1=FifoEmpty
          // DIO2=FifoFull
          // DIO3=FifoEmpty
          // DIO4=LowBat
          // DIO5=ModeReady
          sx1276_write(REG_DIOMAPPING1, (sx1276_read(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) | RF_DIOMAPPING1_DIO1_01);
          sx1276_write(REG_DIOMAPPING2, (sx1276_read(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK));
          sx1276.Settings.OokPacketHandler.FifoThresh = sx1276_read( REG_FIFOTHRESH ) & 0x3F;
        }
        break;

    case MODEM_LORA: 
        {
          if(sx1276.Settings.LoRa.FreqHopOn == true) 
          {
            sx1276_write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                             RFLR_IRQFLAGS_RXDONE |
                                             RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                             RFLR_IRQFLAGS_VALIDHEADER |
                                             //RFLR_IRQFLAGS_TXDONE |
                                             RFLR_IRQFLAGS_CADDONE |
                                             //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                             RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=TxDone, DIO2=FhssChangeChannel
            sx1276_write(REG_DIOMAPPING1, (sx1276_read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00);
          } 
          else 
          {
            sx1276_write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                              RFLR_IRQFLAGS_RXDONE |
                                              RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                              RFLR_IRQFLAGS_VALIDHEADER |
                                              //RFLR_IRQFLAGS_TXDONE |
                                              RFLR_IRQFLAGS_CADDONE |
                                              RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                              RFLR_IRQFLAGS_CADDETECTED );

            // DIO0=TxDone
            sx1276_write(REG_DIOMAPPING1, (sx1276_read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
          }
        }
        break;
  }

  sx1276.Settings.State = RF_TX_RUNNING;
  //TimerStart( &TxTimeoutTimer );
  sx1276_set_opmode(RF_OPMODE_TRANSMITTER);
}


void sx1276_set_channel(uint32_t freq) {
    sx1276.Settings.Channel = freq;
    freq = (uint32_t) ((double) freq / (double) FREQ_STEP);
    sx1276_write(REG_FRFMSB, (uint8_t) ((freq >> 16) & 0xFF));
    sx1276_write(REG_FRFMID, (uint8_t) ((freq >> 8 ) & 0xFF));
    sx1276_write(REG_FRFLSB, (uint8_t) ((freq      ) & 0xFF));
}

void sx1276_set_modem(radio_modem_t modem) {
  if(sx1276.Settings.Modem == modem) 
    {
      return;
    }

  sx1276.Settings.Modem = modem;

  switch(sx1276.Settings.Modem) 
  {
    default:
    case MODEM_FSK:
    case MODEM_OOK:
        sx1276_set_sleep();
        // sx1276_set_opmode(RF_OPMODE_SLEEP );
        sx1276_write(REG_OPMODE, (sx1276_read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);

        sx1276_write(REG_DIOMAPPING1, 0x00);
        sx1276_write(REG_DIOMAPPING2, 0x30); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        sx1276_set_sleep();
        // sx1276_set_opmode(RF_OPMODE_SLEEP );
        sx1276_write(REG_OPMODE, (sx1276_read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);

        sx1276_write(REG_DIOMAPPING1, 0x00);
        sx1276_write(REG_DIOMAPPING2, 0x00);
        break;
  }
}

void sx1276_set_opmode(uint8_t opmode) {
  sx1276_write(REG_OPMODE, (sx1276_read(REG_OPMODE) & RF_OPMODE_MASK) | opmode);
}

void sx1276_write(uint8_t addr, uint8_t data) {
  spi_chipEnable();
  // __delay_cycles(20);
  // clock_delay(5);

  spi_transfer(addr | 0x80);
  spi_transfer(data);

  spi_chipDisable();
}

void sx1276_write_buffer(uint8_t addr, uint8_t* data, uint8_t len) {
  if (len > 0) 
  {
    spi_chipEnable();
    // __delay_cycles(20);
    spi_send(addr | 0x80);

    while (len--) 
      {
        spi_send(*data++);
      }

    spi_txready();

    spi_chipDisable();
  }
}

void sx1276_write_fifo(uint8_t *data, uint8_t len) {
  sx1276_write_buffer(0, data, len);
}

uint8_t sx1276_read(uint8_t addr) {
  spi_chipEnable();
  // __delay_cycles(20);
  spi_transfer(addr & 0x7f);
  spi_transfer(0x00);

  uint8_t result = spi_buf;

  spi_chipDisable();

  return result;
}

void sx1276_read_buffer(uint8_t addr, uint8_t* data, uint8_t len) {
  if (len > 0) 
  {
    spi_chipEnable();
    // __delay_cycles(20);
    spi_send(addr & 0x7f);

    while (len--) 
    {
      spi_transfer(0x00);
      *data++  = spi_buf;
    }

    spi_chipDisable();
  }
}

void sx1276_read_fifo(uint8_t *data, uint8_t len) {
  sx1276_read_buffer(0, data, len);
}

void sx1276_on_dio0irq() {
  volatile uint8_t irqFlags = 0;

  switch(sx1276.Settings.State) 
  {
    case RF_RX_RUNNING:
      // // TimerStop( &RxTimeoutTimer );
      switch(sx1276.Settings.Modem) 
      {
        case MODEM_FSK: 
        case MODEM_OOK:
            {
              if(sx1276.Settings.Fsk.CrcOn == true) 
              {
                irqFlags = sx1276_read( REG_IRQFLAGS2 );
                if((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK) 
                {
                  // Clear Irqs
                  sx1276_write(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | RF_IRQFLAGS1_PREAMBLEDETECT | RF_IRQFLAGS1_SYNCADDRESSMATCH);
                  sx1276_write(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

                  //TimerStop( &RxTimeoutTimer );

                  if(sx1276.Settings.Fsk.RxContinuous == false) 
                  {
                    //TimerStop( &RxTimeoutSyncWord );
                    sx1276.Settings.State = RF_IDLE;
                  } 
                  else 
                  {
                    // Continuous mode restart Rx chain
                    sx1276_write( REG_RXCONFIG, sx1276_read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                    //TimerStart( &RxTimeoutSyncWord );
                  }

                  if((RadioEvents != 0) && (RadioEvents->RxError != 0) ) 
                  {
                      RadioEvents->RxError();
                  }

                  sx1276.Settings.FskPacketHandler.PreambleDetected = false;
                  sx1276.Settings.FskPacketHandler.SyncWordDetected = false;
                  sx1276.Settings.FskPacketHandler.NbBytes = 0;
                  sx1276.Settings.FskPacketHandler.Size = 0;
                  break;
                }
              }

              // Read received packet size
              if((sx1276.Settings.FskPacketHandler.Size == 0) && (sx1276.Settings.FskPacketHandler.NbBytes == 0)) 
              {
                if(sx1276.Settings.Fsk.FixLen == false) 
                {
                    sx1276_read_fifo((uint8_t*) &sx1276.Settings.FskPacketHandler.Size, 1);
                } 
                else 
                {
                    sx1276.Settings.FskPacketHandler.Size = sx1276_read(REG_PAYLOADLENGTH);
                }
                sx1276_read_fifo(RxTxBuffer + sx1276.Settings.FskPacketHandler.NbBytes, sx1276.Settings.FskPacketHandler.Size - sx1276.Settings.FskPacketHandler.NbBytes);
                sx1276.Settings.FskPacketHandler.NbBytes += (sx1276.Settings.FskPacketHandler.Size - sx1276.Settings.FskPacketHandler.NbBytes);
              } 
              else 
              {
                sx1276_read_fifo(RxTxBuffer + sx1276.Settings.FskPacketHandler.NbBytes, sx1276.Settings.FskPacketHandler.Size - sx1276.Settings.FskPacketHandler.NbBytes);
                sx1276.Settings.FskPacketHandler.NbBytes += (sx1276.Settings.FskPacketHandler.Size - sx1276.Settings.FskPacketHandler.NbBytes);
              }

              if(sx1276.Settings.Fsk.RxContinuous == false) 
              {
                sx1276.Settings.State = RF_IDLE;
                //TimerStart( &RxTimeoutSyncWord );
              } 
              else 
              {
                // Continuous mode restart Rx chain
                sx1276_write(REG_RXCONFIG, sx1276_read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
              }
              //TimerStop( &RxTimeoutTimer );

              if((RadioEvents != 0) && (RadioEvents->RxDone != 0))   
              {
                  RadioEvents->RxDone(RxTxBuffer, sx1276.Settings.FskPacketHandler.Size, sx1276.Settings.FskPacketHandler.RssiValue, 0);
              }

              sx1276.Settings.FskPacketHandler.PreambleDetected = false;
              sx1276.Settings.FskPacketHandler.SyncWordDetected = false;
              sx1276.Settings.FskPacketHandler.NbBytes = 0;
              sx1276.Settings.FskPacketHandler.Size = 0;
            }
            break;
        case MODEM_LORA: 
            {
              int8_t snr = 0;

              // Clear Irq
              sx1276_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

              irqFlags = sx1276_read(REG_LR_IRQFLAGS);
              if((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR) 
              {
                // Clear Irq
                sx1276_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

                if(sx1276.Settings.LoRa.RxContinuous == false) 
                {
                  sx1276.Settings.State = RF_IDLE;
                }
                //TimerStop( &RxTimeoutTimer );

                if((RadioEvents != 0) && (RadioEvents->RxError != 0)) 
                {
                    RadioEvents->RxError();
                }

                break;
              }

              sx1276.Settings.LoRaPacketHandler.SnrValue = sx1276_read(REG_LR_PKTSNRVALUE);
              if(sx1276.Settings.LoRaPacketHandler.SnrValue & 0x80) 
              { // The SNR sign bit is 1
                // Invert and divide by 4
                snr = ( ( ~sx1276.Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                snr = -snr;
              } 
              else 
              {
                // Divide by 4
                snr = ( sx1276.Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
              }

              int16_t rssi = sx1276_read(REG_LR_PKTRSSIVALUE);
              if(snr < 0) 
              {
                if(sx1276.Settings.Channel > RF_MID_BAND_THRESH) 
                {
                  sx1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4) + snr;
                } 
                else 
                {
                  sx1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4) + snr;
                }
              } 
              else 
              {
                if(sx1276.Settings.Channel > RF_MID_BAND_THRESH) 
                {
                  sx1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4);
                } else 
                {
                  sx1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4);
                }
              }

              sx1276.Settings.LoRaPacketHandler.Size = sx1276_read(REG_LR_RXNBBYTES);
              sx1276_write( REG_LR_FIFOADDRPTR, sx1276_read( REG_LR_FIFORXCURRENTADDR ) ); //added 
              sx1276_read_fifo(RxTxBuffer, sx1276.Settings.LoRaPacketHandler.Size);

              if(sx1276.Settings.LoRa.RxContinuous == false) 
              {
                sx1276.Settings.State = RF_IDLE;
              }
              //TimerStop( &RxTimeoutTimer );

              if((RadioEvents != 0) && (RadioEvents->RxDone != 0)) 
              {
                RadioEvents->RxDone(RxTxBuffer, sx1276.Settings.LoRaPacketHandler.Size, sx1276.Settings.LoRaPacketHandler.RssiValue, sx1276.Settings.LoRaPacketHandler.SnrValue);
                PRINTF("RxDone\n");
                PRINTF("Radio Layer Pkt Size: %d\n", sx1276.Settings.LoRaPacketHandler.Size);
                PRINTF("Radio Layer RSSI: %d\n", sx1276.Settings.LoRaPacketHandler.RssiValue);
              }
            }
            break;
        default:
        break;
      }
    break;
    case RF_TX_RUNNING:
          //TimerStop( &TxTimeoutTimer );
          switch(sx1276.Settings.Modem) 
          {
            case MODEM_LORA:
              // Clear Irq
              sx1276_write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
            case MODEM_FSK:
            case MODEM_OOK:
            default:
              sx1276.Settings.State = RF_IDLE;
              if((RadioEvents != 0) && (RadioEvents->TxDone != 0)) 
              {
                RadioEvents->TxDone( );
              }
              break;
          }
        break;
    default:
    break;
  }
}


#pragma vector=PORT1_VECTOR
__interrupt void port1_interrupt_handler(void)
{
    if (P1IFG & BIT3)
    {
        sx1276_on_dio0irq();

    }
    // Reset the interrupt flag
    P1IFG &= ~BIT3;
}

