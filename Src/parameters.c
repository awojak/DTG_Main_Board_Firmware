/*
 * parameters.c
 *
 *  Created on: 15.03.2020
 *      Author: Adrian
 */

#include "parameters.h"
#include "settings.h"
#include "common/utils.h"
#include "drivers/motion_controller.h"
#include "scheduler/tasks.h"

/* Lookup Tables */

const char * const table_off_on[] = {
	"OFF",
	"ON",
};

const char * const table_motion_y_state[] = {
	"STOP",
	"ACCEL",
	"DECEL",
	"RUN"
};


/* Parameters definition */
/* Y motor parameters */
static setting_t motion_state_y = {
		.name = "motion_state_y",
		.type = MODE_LOOKUP | MASTER_VALUE,
		.parameterPointer = &MotionY.ramp_data.run_state,
		.lookupTable.values = table_motion_y_state,
		.lookupTable.valueCount =ARRAYLEN(table_motion_y_state)
};

static setting_t print_speed_y = {
		.name = "print_speed_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_SPEED_Y,
		.config.min = PARAMETER_PRINT_SPEED_Y_MIN,
		.config.max = PARAMETER_PRINT_SPEED_Y_MAX,
		.parameterPointer = &MotionY.rampPrint.speed
};

static setting_t print_accel_y = {
		.name = "print_accel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_ACCEL_Y,
		.config.min = PARAMETER_PRINT_ACCEL_Y_MIN,
		.config.max = PARAMETER_PRINT_ACCEL_Y_MAX,
		.parameterPointer = &MotionY.rampPrint.accel
};

static setting_t print_decel_y = {
		.name = "print_decel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_DECEL_Y,
		.config.min = PARAMETER_PRINT_DECEL_Y_MIN,
		.config.max = PARAMETER_PRINT_DECEL_Y_MAX,
		.parameterPointer = &MotionY.rampPrint.decel
};

/* JOG */
static setting_t jog_speed_y = {
		.name = "jog_speed_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_JOG_SPEED_Y,
		.config.min = PARAMETER_JOG_SPEED_Y_MIN,
		.config.max = PARAMETER_JOG_SPEED_Y_MAX,
		.parameterPointer = &MotionY.rampJog.speed
};

static setting_t jog_accel_y = {
		.name = "jog_accel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_JOG_ACCEL_Y,
		.config.min = PARAMETER_JOG_ACCEL_Y_MIN,
		.config.max = PARAMETER_JOG_ACCEL_Y_MAX,
		.parameterPointer = &MotionY.rampJog.accel
};

static setting_t jog_decel_y = {
		.name = "jog_decel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_JOG_DECEL_Y,
		.config.min = PARAMETER_JOG_DECEL_Y_MIN,
		.config.max = PARAMETER_JOG_DECEL_Y_MAX,
		.parameterPointer = &MotionY.rampJog.decel
};

/* Table contain all parameters */

const setting_t *parametersTable[] = {
		&motion_state_y,
		&print_speed_y,
		&print_accel_y,
		&print_decel_y,
		&jog_speed_y,
		&jog_accel_y,
		&jog_decel_y
} ;

const uint8_t paramsTableLen = ARRAYLEN(parametersTable);

