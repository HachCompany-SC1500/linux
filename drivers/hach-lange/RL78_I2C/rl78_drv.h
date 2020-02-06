/*******************************************************************************
 *
 * (C) Copyright 2014 Hach-Lange
 *
 * Project: Fusion
 * Purpose: Contains definitions and for accessing RL78 interrupt and
 *          request lines.
 *
 * File:  RL78_drv.h
 * Desc:  Definitions for RL78 interrupt access.
 *
 * Revision History:
 * 2014-07-17 created.
 *
 ********************************************************************************/

#ifndef _RL78_DRV_H
#define _RL78_DRV_H


#define RL78_READ_REQ_LINES  701
#define RL78_READ_INT        702


#define I2C_BAUDRATE_CONFIG 1
#ifdef I2C_BAUDRATE_CONFIG
#define I2C_BAUDRATE 0x709  /* configure I2C baudrate */
#endif

/*
 * defines for RL78 request line handling (REQ0...REQ3),
 * connected via PTC2...PTC5
 */
#define REQ0 0
#define REQ1 1
#define REQ2 2
#define REQ3 3
#define REQALL 4

#define REQ0_MASK 0x04
#define REQ1_MASK 0x08
#define REQ2_MASK 0x10
#define REQ3_MASK 0x20
#define REQALL_MASK (REQ0_MASK | REQ1_MASK | REQ2_MASK | REQ3_MASK)

#define REQ0_SHIFT 2
#define REQ1_SHIFT 3
#define REQ2_SHIFT 4
#define REQ3_SHIFT 5
#define REQALL_SHIFT 2


#ifdef __KERNEL__
#define RL78_IRQ_MAJOR  245    /* RL78 irq device major number */
#define RL78_I2C_MAJOR  246    /* RL78 i2c device major number */
// #define RL78_I2C_MAJOR I2C_MAJOR
#endif




#endif
