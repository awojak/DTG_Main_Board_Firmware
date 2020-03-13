/*
 * tmc2209.h
 *
 *  Created on: 10.03.2020
 *      Author: Neo
 */

#ifndef TMC2209_H_
#define TMC2209_H_

/* GENERAL CONFIGURATION REGISTERS (0X00…0X0F) */
/* [RW] GCONF - Global configuration flags */
#define GCONF 0x00
	#define	GCONF_I_SCALE_ANALOG 0x00
	#define GCONF_INTERNAL_RSENSE 0x01
	#define GCONF_EN_SPREAD_CYCLE 0x02
	#define GCONF_SHAFT 0x04
	#define GCONF_INDEX_OTPW 0x08
	#define GCONF_INDEX_STEP 0x10
	#define GCONF_PDN_DISABLE 0x20
	#define GCONF_MSTEP_REG_SELECT 0x40
	#define GCONF_MULTISTEP_FILT 0x80
	#define GCONF_TEST_MODE 0x100

/* [R+WC] GSTAT - Global status flag */
#define GSTAT 0x01
	#define GSTAT_RESET 0x00
	#define GSTAT_DRV_ERR 0x01
	#define GSTAT_UV_CP 0x02

/* [R] IFCNT - Interface transmission counter */
#define IFCNT 0x02

/* [W] SLAVECONF - SENDDELAY for read access */
#define SLAVECONF 0x03

/* [W] OTP_PROG - OTP programming */
#define OTP_PROG 0x04

/* [R] OTP_READ - OTP read data */
#define OTP_READ 0x05

/* [R] IOIN - INPUT (Reads the state of all input pins available) */
#define IOIN 0x06

/* [RW] FACTORY_CONF */
#define FACTORY_CONF 0x07

/* VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10…0X1F) */
#define IHOLD_IRUN 0x10

#endif /* TMC2209_H_ */
