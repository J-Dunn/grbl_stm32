/*
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */



#ifndef _GRBL_6474_H
#define _GRBL_6474_H

#include <stdint.h>  // for uintXX_t defs
#ifndef CPU_MAP_L6474
#define CPU_MAP_L6474  // ensure anything using this lib turns on L6474 sections
#endif


#define NUMBER_OF_SHIELDS  (3)
#define ALL_SHIELDS (NUMBER_OF_SHIELDS)
#define L6474_TXBUFF_LEN           (4)   // L6474_CMD_MAX_ARG_BYTES  

// pull in reqd #defines if not using SPI class
#ifndef SPI_MODE3
#define SPI_MODE3 0x0C
#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV2 0x04
#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR
#endif


/// Define to print debug logs via the UART 
#ifndef _DEBUG_GRBL6474
#define _DEBUG_GRBL6474
#endif


#ifdef _DEBUG_L6474
/// Size of the log buffer
#define DEBUG_BUFFER_SIZE    (75)
/// Log buffer
extern char l6474StrOut[DEBUG_BUFFER_SIZE];
#endif

#define L6474_Reset_Pin  (8)  // = PB0  ### unused?



/// L6474 command + argument bytes number for GET_STATUS command
#define L6474_GET_PARAM_ARG_BYTES         (1)
#define L6474_STATUS_RESP_BYTES           (2)

#define L6474_2byte_result (L6474_TXBUFF_LEN - L6474_STATUS_RESP_BYTES)   
// define pointer for accessing results in spiRxBuffer, must match send order in L6474_GetStatusParamAll() etc


/// TOFF_FAST & FAST_STEP : (nibble+1)*2 us

// datasheet default times 1us/5us are WRONG  ### do not match Table 13 values for 0x19 defaults

/// L6474 fast decay time option (TOFF_FAST values for T_FAST register, high nibble )
typedef enum {
  L6474_TOFF_FAST_2us  = ((uint8_t) 0x00 << 4),  
  L6474_TOFF_FAST_4us  = ((uint8_t) 0x01 << 4), // def from hi nibble of 0x19
  L6474_TOFF_FAST_6us  = ((uint8_t) 0x02 << 4),
  L6474_TOFF_FAST_8us  = ((uint8_t) 0x03 << 4),
  L6474_TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
  L6474_TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
  L6474_TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
  L6474_TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
  L6474_TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
  L6474_TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
  L6474_TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
  L6474_TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
  L6474_TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
  L6474_TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
  L6474_TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
  L6474_TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
} L6474_TOFF_FAST_t;


/// L6474 fall step time options (FAST_STEP values for T_FAST register )
typedef enum {
  L6474_FAST_STEP_2us  = ((uint8_t) 0x00),
  L6474_FAST_STEP_4us  = ((uint8_t) 0x01),
  L6474_FAST_STEP_6us  = ((uint8_t) 0x02),
  L6474_FAST_STEP_8us  = ((uint8_t) 0x03),
  L6474_FAST_STEP_10us = ((uint8_t) 0x04),
  L6474_FAST_STEP_12us = ((uint8_t) 0x05),
  L6474_FAST_STEP_14us = ((uint8_t) 0x06),
  L6474_FAST_STEP_16us = ((uint8_t) 0x07),
  L6474_FAST_STEP_18us = ((uint8_t) 0x08),
  L6474_FAST_STEP_20us = ((uint8_t) 0x09), // def from lo nibble of 0x19
  L6474_FAST_STEP_22us = ((uint8_t) 0x0A),
  L6474_FAST_STEP_24us = ((uint8_t) 0x0B),
  L6474_FAST_STEP_26us = ((uint8_t) 0x0C),
  L6474_FAST_STEP_28us = ((uint8_t) 0x0D),
  L6474_FAST_STEP_30us = ((uint8_t) 0x0E),
  L6474_FAST_STEP_32us = ((uint8_t) 0x0F)
} L6474_FAST_STEP_t;

/// L6474_TON_MIN , L6474_TOFF_MIN same range values
/// def =0x29=d41 at start-up. T= (byte+1)/2 us def =21us

/// L6474 values for min on time (TON_MIN register)
typedef enum {
  L6474_TON_MIN_0_5us  = ((uint8_t) 0x00),
  L6474_TON_MIN_1us    = ((uint8_t) 0x01),
  L6474_TON_MIN_2us    = ((uint8_t) 0x03),
  L6474_TON_MIN_4us    = ((uint8_t) 0x07),
  L6474_TON_MIN_16us   = ((uint8_t) 0x1F),
  L6474_TON_MIN_17us   = ((uint8_t) 0x21),
  L6474_TON_MIN_18us   = ((uint8_t) 0x23),
  L6474_TON_MIN_20us   = ((uint8_t) 0x27),
  L6474_TON_MIN_21us   = ((uint8_t) 0x29)  // default 0x29=d41 : 21us 
} L6474_TON_MIN_t;
/// L6474 overcurrent threshold options (OCD_TH register)


/// L6474 CONFIG off time options ( cf TOFF_MIN register)
typedef enum {
  L6474_CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
  L6474_CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
  L6474_CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
  L6474_CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
  L6474_CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
  L6474_CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
  L6474_CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
  L6474_CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
  L6474_CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
  L6474_CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),  // default
  L6474_CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
  L6474_CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
  L6474_CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
  L6474_CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
  L6474_CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
  L6474_CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
  L6474_CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
  L6474_CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
  L6474_CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
  L6474_CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
  L6474_CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
  L6474_CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
  L6474_CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
  L6474_CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
  L6474_CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
  L6474_CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
  L6474_CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
  L6474_CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
  L6474_CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
  L6474_CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
  L6474_CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
} L6474_CONFIG_TOFF_t;


typedef enum {
  L6474_OCD_TH_375mA  = ((uint8_t) 0x00),
  L6474_OCD_TH_750mA  = ((uint8_t) 0x01),
  L6474_OCD_TH_1125mA = ((uint8_t) 0x02),
  L6474_OCD_TH_1500mA = ((uint8_t) 0x03),
  L6474_OCD_TH_1875mA = ((uint8_t) 0x04),
  L6474_OCD_TH_2250mA = ((uint8_t) 0x05),
  L6474_OCD_TH_2625mA = ((uint8_t) 0x06),
  L6474_OCD_TH_3000mA = ((uint8_t) 0x07),
  L6474_OCD_TH_3375mA = ((uint8_t) 0x08),  // default 
  L6474_OCD_TH_3750mA = ((uint8_t) 0x09),
  L6474_OCD_TH_4125mA = ((uint8_t) 0x0A),
  L6474_OCD_TH_4500mA = ((uint8_t) 0x0B),
  L6474_OCD_TH_4875mA = ((uint8_t) 0x0C),
  L6474_OCD_TH_5250mA = ((uint8_t) 0x0D),
  L6474_OCD_TH_5625mA = ((uint8_t) 0x0E),
  L6474_OCD_TH_6000mA = ((uint8_t) 0x0F)
} L6474_OCD_TH_t;

/// L6474 STEP_MODE register masks
typedef enum {
  L6474_STEP_MODE_STEP_SEL = ((uint8_t) 0x07),
  L6474_STEP_MODE_SYNC_SEL = ((uint8_t) 0x70)
} L6474_STEP_MODE_Masks_t;

/// L6474 STEP_SEL options for STEP_MODE register 
typedef enum {
  L6474_STEP_SEL_1    = ((uint8_t) 0x08),  //full step
  L6474_STEP_SEL_1_2  = ((uint8_t) 0x09),  //half step
  L6474_STEP_SEL_1_4  = ((uint8_t) 0x0A),  //1/4 microstep
  L6474_STEP_SEL_1_8  = ((uint8_t) 0x0B),  //1/8 microstep
  L6474_STEP_SEL_1_16 = ((uint8_t) 0x0C)   //1/16 microstep
} L6474_STEP_SEL_t;

/// L6474 SYNC_SEL options for STEP_MODE register 
typedef enum {
  L6474_SYNC_SEL_1_2    = ((uint8_t) 0x80),
  L6474_SYNC_SEL_1      = ((uint8_t) 0x90),
  L6474_SYNC_SEL_2      = ((uint8_t) 0xA0),
  L6474_SYNC_SEL_4      = ((uint8_t) 0xB0),
  L6474_SYNC_SEL_8      = ((uint8_t) 0xC0),
  L6474_SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
} L6474_SYNC_SEL_t;

/// L6474 ALARM_EN register options
typedef enum {
  L6474_ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
  L6474_ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
  L6474_ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
  L6474_ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
  L6474_ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
  L6474_ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
} L6474_ALARM_EN_t;


/// L6474 CONFIG register masks
typedef enum {
  L6474_CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
  L6474_CONFIG_EN_TQREG = ((uint16_t) 0x0020),
  L6474_CONFIG_OC_SD    = ((uint16_t) 0x0080),
  L6474_CONFIG_VOLT_SR  = ((uint16_t) 0x0300),
  L6474_CONFIG_TOFF_MASK = ((uint16_t) 0x001F<<10)  // =L6474_CONFIG_TOFF_124us 
} L6474_CONFIG_Masks_t;


/// L6474 clock source options for CONFIG register
typedef enum {
  L6474_CONFIG_INT_16MHZ               = ((uint16_t) 0x0000),
  L6474_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),  // default
  L6474_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
  L6474_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
  L6474_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
  L6474_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
  L6474_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
  L6474_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
  L6474_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
  L6474_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
  L6474_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
  L6474_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
} L6474_CONFIG_OSC_MGMT_t;


/// L6474 external torque regulation options for CONFIG register
typedef enum {
  L6474_CONFIG_EN_TQREG_TVAL_USED = ((uint16_t) 0x0000),
  L6474_CONFIG_EN_TQREG_ADC_OUT = ((uint16_t) 0x0020)
} L6474_CONFIG_EN_TQREG_t;

/// L6474 over current shutdown options for CONFIG register
typedef enum {
  L6474_CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
  L6474_CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)   // default
} L6474_CONFIG_OC_SD_t;

/// L6474 power bridge output (falling) slew_rates options (POW_SR values for CONFIG register)
// datasheet table 5, rising slew-rates vary somewhat and fastest setting changes for -ve current
typedef enum {
  L6474_CONFIG_SR_320V_us    =((uint16_t)0x0000),
  L6474_CONFIG_SR_075V_us    =((uint16_t)0x0100),
  L6474_CONFIG_SR_110V_us    =((uint16_t)0x0200),
  L6474_CONFIG_SR_260V_us    =((uint16_t)0x0300)    // default
} L6474_CONFIG_POW_SR_t;

///  L6474 STATUS register bit masks 
typedef enum {
  L6474_STATUS_HIZ         = (((uint16_t) 0x0001)),
  L6474_STATUS_BIT1        = (((uint16_t) 0x0002)),  // def=1
//  L6474_STATUS_BIT2       = (((uint16_t) 0x0004)),  // def=0
//  L6474_STATUS_BIT3       = (((uint16_t) 0x0008)),  // def=0
  L6474_STATUS_DIR         = (((uint16_t) 0x0010)),
//  L6474_STATUS_BIT5       = (((uint16_t) 0x002)),  // def=0
//  L6474_STATUS_BIT6       = (((uint16_t) 0x004)),  // def=0
  L6474_STATUS_NOTPERF_CMD = (((uint16_t) 0x0080)),  // active hi
  
  L6474_STATUS_WRONG_CMD   = (((uint16_t) 0x0100)),  // active hi, rest of hi byte active LO !
  L6474_NOT_STATUS_UVLO        = (((uint16_t) 0x0200)),
  L6474_NOT_STATUS_TH_WRN      = (((uint16_t) 0x0400)),
  L6474_NOT_STATUS_TH_SD       = (((uint16_t) 0x0800)),
  L6474_NOT_STATUS_OCD         = (((uint16_t) 0x1000)),
//  L6474_STATUS_BIT13       = (((uint16_t) 0x2000)),  // def=1
//  L6474_STATUS_BIT14       = (((uint16_t) 0x4000)),  // def=1
//  L6474_STATUS_BIT15       = (((uint16_t) 0x8000)),  // def=1
} L6474_STATUS_Masks_t;


// define short masks to resolve error fast in INT0_vect ISR
#define  L6474_CRIT_FLAGS   ((L6474_NOT_STATUS_UVLO | L6474_NOT_STATUS_TH_SD | L6474_NOT_STATUS_OCD)>>8)
#define  L6474_LOW_ACTIVE_FLAGS   ( L6474_CRIT_FLAGS | ( L6474_NOT_STATUS_TH_WRN  >>8 ) )
#define  L6474_DEF_HI_BITS  ( (1<<15) | (1<<14)  | (1<<13) | (1<<1))
#define  L6474_DEF_LO_BITS  ( (1<<6) | (1<<5)  | (1<<3) | (1<<2))

#define HiZ (uint32_t)0x00000001

/// L6474 STATUS register options
typedef enum {
  L6474_STATUS_DIR_FORWARD = (((uint16_t) 0x0010) ),
  L6474_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) )
} L6474_STATUS_DIR_t;



/// L6474 internal register addresses
typedef enum {
  L6474_ABS_POS        = ((uint8_t) 0x01),
  L6474_EL_POS         = ((uint8_t) 0x02),
  L6474_MARK           = ((uint8_t) 0x03),
  L6474_RESERVED_REG01 = ((uint8_t) 0x04),
  L6474_RESERVED_REG02 = ((uint8_t) 0x05),
  L6474_RESERVED_REG03 = ((uint8_t) 0x06),
  L6474_RESERVED_REG04 = ((uint8_t) 0x07),
  L6474_RESERVED_REG05 = ((uint8_t) 0x08),
  L6474_TVAL           = ((uint8_t) 0x09),
  L6474_RESERVED_REG07 = ((uint8_t) 0x0A),
  L6474_RESERVED_REG08 = ((uint8_t) 0x0B),
  L6474_RESERVED_REG09 = ((uint8_t) 0x0C),
  L6474_RESERVED_REG10 = ((uint8_t) 0x0D),
  L6474_T_FAST         = ((uint8_t) 0x0E),
  L6474_TON_MIN        = ((uint8_t) 0x0F),
  L6474_TOFF_MIN       = ((uint8_t) 0x10),
  L6474_RESERVED_REG11 = ((uint8_t) 0x11),
  L6474_ADC_OUT        = ((uint8_t) 0x12),
  L6474_OCD_TH         = ((uint8_t) 0x13),  // def 8 = 3.375A
  L6474_RESERVED_REG12 = ((uint8_t) 0x14),
  L6474_RESERVED_REG06 = ((uint8_t) 0x15),
  L6474_STEP_MODE      = ((uint8_t) 0x16),
  L6474_ALARM_EN       = ((uint8_t) 0x17),
  L6474_CONFIG         = ((uint8_t) 0x18),
  L6474_STATUS         = ((uint8_t) 0x19),
  L6474_RESERVED_REG13 = ((uint8_t) 0x1A),
  L6474_RESERVED_REG14 = ((uint8_t) 0x1B),
  L6474_INEXISTENT_REG = ((uint8_t) 0x1F)
} L6474_Registers_t;

// re.  L6474_STEP_MODE 
// BUSY/SYNC line tied up on on L6474 shield but can vary internally
// b7 and  b3 (reserved) =1 ; b4..6 SYNC: (unused =1) ie 0xF8
#define NO_SYNC ( (uint8_t)0xF8 )
#define   L6474_STEP_MODE_MASK ((uint8_t) 0b111)  // MODE is coded in 3 LSBs ie ~0xF8=d248

typedef enum {
L6474_STEP_FUll  = ((uint8_t)  0x000 | NO_SYNC),
L6474_STEP_HALF  = ((uint8_t)  0b001 | NO_SYNC),
L6474_STEP_BY4   = ((uint8_t)  0b010 | NO_SYNC),
L6474_STEP_BY8   = ((uint8_t)  0b011 | NO_SYNC),
L6474_STEP_BY16  = ((uint8_t)  0b100 | NO_SYNC)  // last two bits ignored only b2 signif.
} L6474_STEP_MODE_t;  
 
      
/// L6474 command set
typedef enum {
  L6474_NOP           = ((uint8_t) 0x00),
  L6474_SET_PARAM     = ((uint8_t) 0x00), // OR with param register byte 0x01 .. 0x0F
  L6474_GET_PARAM     = ((uint8_t) 0x20), // idem.
  L6474_ENABLE        = ((uint8_t) 0xB8), // d184
  L6474_DISABLE       = ((uint8_t) 0xA8), // d168
  L6474_GET_STATUS    = ((uint8_t) 0xD0), // resets warming flags but does not remove HiZ state.
  L6474_RESERVED_CMD1 = ((uint8_t) 0xEB),
  L6474_RESERVED_CMD2 = ((uint8_t) 0xF8)
} L6474_Commands_t;

//volatile void (*L6474::flagInterruptCallback)(void);
//volatile uint8_t numberOfShields;

void L6474_init(); 
void SPI_ChainSend_Byte(uint8_t *pTxByte, uint8_t *pRxByte);
void L6474_SendByteCommand(uint8_t shieldId, uint8_t command);
uint16_t L6474_GetStatus_ClearFlags(uint8_t shieldId);
uint32_t L6474_GetParam(uint8_t shieldId, L6474_Registers_t param);
void L6474_GetStatusParam_All();
void L6474_SetParam(uint8_t shieldId,L6474_Registers_t param,uint32_t value);
void Wait_us(uint16_t usDelay);
void L6474_FLAG_INT_vect (void) ;  // this is no longer an INT, so make it accessible to limitpin_check() in limits.c

extern uint8_t (*spiRxBuffer) [3] ;  // ### once in report : restructure ??? 
extern volatile uint8_t inFlagInt;

#endif /* #ifndef _GRBL_6474_H */
