/*
 * parameters.c
 *
 *  Created on: 15.03.2020
 *      Author: Adrian
 */

#include "parameters.h"
#include "settings.h"

/* Lookup Tables */

static const char * const table_off_on[] = {
	"OFF",
	"ON",
};

static const char * const table_motion_y_state[] = {
	"STOP",
	"START",
};

/* Parameters definition */

static setting_t print_speed_y = {
		.name = "print_speed_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_SPEED_Y,
		.config.min = PARAMETER_PRINT_SPEED_Y_MIN,
		.config.max = PARAMETER_PRINT_SPEED_Y_MAX
};

static setting_t print_accel_y = {
		.name = "print_accel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_ACCEL_Y,
		.config.min = PARAMETER_PRINT_ACCEL_Y_MIN,
		.config.max = PARAMETER_PRINT_ACCEL_Y_MAX
};

static setting_t print_decel_y = {
		.name = "print_decel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_DECEL_Y,
		.config.min = PARAMETER_PRINT_DECEL_Y_MIN,
		.config.max = PARAMETER_PRINT_DECEL_Y_MAX
};

/* Table contain all parameters */

static const setting_t *parametersTable[] = {
		&print_speed_y,
		&print_accel_y,
		&print_decel_y
} ;

