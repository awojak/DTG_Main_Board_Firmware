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
#include "printer.h"

extern tPrinter Printer;
extern MotionController MotionY;
extern MotionController MotionZ;
/* Lookup Tables */
/*
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


// Parameters definition
// Y motor parameters
static setting_t motion_state_y = {
		.name = "motion_state_y",
		.type = MODE_LOOKUP | MASTER_VALUE,
		.parameterPointer = &MotionY.ramp_data.run_state,
		.lookupTable.values = table_motion_y_state,
		.lookupTable.valueCount =ARRAYLEN(table_motion_y_state)
};
*/
// Parameters definition
/* Printers parameters */
static setting_t stepper_factor = {
		.name = "stepper_factor",
		.type = VAR_FLOAT | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_STEPPER_FACTOR,
		.config.min = PARAMETER_PRINT_STEPPER_FACTOR_MIN,
		.config.max = PARAMETER_PRINT_STEPPER_FACTOR_MAX,
		.parameterPointer = &Printer.stepper_factor
};

static setting_t print_start_position = {
		.name = "print_start_position",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_START_POS,
		.config.min = PARAMETER_PRINT_START_POS_MIN,
		.config.max = PARAMETER_PRINT_START_POS_MAX,
		.parameterPointer = &Printer.print_start_position
};

static setting_t eject_trigger_position = {
		.name = "eject_trigger_position",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_EJECT_TRIGGER_POS,
		.config.min = PARAMETER_PRINT_EJECT_TRIGGER_POS_MIN,
		.config.max = PARAMETER_PRINT_EJECT_TRIGGER_POS_MAX,
		.parameterPointer = &Printer.eject_trigger_position
};

static setting_t eject_position = {
		.name = "eject_position",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_EJECT_POS,
		.config.min = PARAMETER_PRINT_EJECT_POS_MIN,
		.config.max = PARAMETER_PRINT_EJECT_POS_MAX,
		.parameterPointer = &Printer.eject_position
};

static setting_t timeout_motion_z = {
		.name = "timeout_motion_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_TIMEOUT_Z,
		.config.min = PARAMETER_PRINT_TIMEOUT_Z_MIN,
		.config.max = PARAMETER_PRINT_TIMEOUT_Z_MAX,
		.parameterPointer = &Printer.timeout_motion_z
};

static setting_t timeout_motion_y = {
		.name = "timeout_motion_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_TIMEOUT_Y,
		.config.min = PARAMETER_PRINT_TIMEOUT_Y_MIN,
		.config.max = PARAMETER_PRINT_TIMEOUT_Y_MAX,
		.parameterPointer = &Printer.timeout_motion_y
};

static setting_t pe_lower_limit = {
		.name = "pe_lower_limit",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_PE_LOWER_LIMIT,
		.config.min = PARAMETER_PRINT_PE_LOWER_LIMIT_MIN,
		.config.max = PARAMETER_PRINT_PE_LOWER_LIMIT_MAX,
		.parameterPointer = &Printer.pe_lower_limit
};

static setting_t pe_upper_limit = {
		.name = "pe_upper_limit",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_PE_UPPER_LIMIT,
		.config.min = PARAMETER_PRINT_PE_UPPER_LIMIT_MIN,
		.config.max = PARAMETER_PRINT_PE_UPPER_LIMIT_MAX,
		.parameterPointer = &Printer.pe_lower_limit
};

// Y motor parameters
/* PRINT */
static setting_t home_speed_y = {
		.name = "home_speed_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_HOME_SPEED_Y,
		.config.min = PARAMETER_HOME_SPEED_Y_MIN,
		.config.max = PARAMETER_HOME_SPEED_Y_MAX,
		.parameterPointer = &MotionY.rampHome.speed
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

/* MOVE */

static setting_t move_speed_y = {
		.name = "move_speed_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_MOVE_SPEED_Y,
		.config.min = PARAMETER_MOVE_SPEED_Y_MIN,
		.config.max = PARAMETER_MOVE_SPEED_Y_MAX,
		.parameterPointer = &MotionY.rampMove.speed
};


static setting_t move_accel_y = {
		.name = "move_accel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_MOVE_ACCEL_Y,
		.config.min = PARAMETER_MOVE_ACCEL_Y_MIN,
		.config.max = PARAMETER_MOVE_ACCEL_Y_MAX,
		.parameterPointer = &MotionY.rampMove.accel
};

static setting_t move_decel_y = {
		.name = "move_decel_y",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_MOVE_DECEL_Y,
		.config.min = PARAMETER_MOVE_DECEL_Y_MIN,
		.config.max = PARAMETER_MOVE_DECEL_Y_MAX,
		.parameterPointer = &MotionY.rampMove.decel
};

// Parameters definition
// Z motor parameters
/* PRINT */
static setting_t home_speed_z = {
		.name = "home_speed_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_HOME_SPEED_Z,
		.config.min = PARAMETER_HOME_SPEED_Z_MIN,
		.config.max = PARAMETER_HOME_SPEED_Z_MAX,
		.parameterPointer = &MotionZ.rampHome.speed
};

static setting_t print_speed_z = {
		.name = "print_speed_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_SPEED_Z,
		.config.min = PARAMETER_PRINT_SPEED_Z_MIN,
		.config.max = PARAMETER_PRINT_SPEED_Z_MAX,
		.parameterPointer = &MotionZ.rampPrint.speed
};

static setting_t print_accel_z = {
		.name = "print_accel_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_ACCEL_Z,
		.config.min = PARAMETER_PRINT_ACCEL_Z_MIN,
		.config.max = PARAMETER_PRINT_ACCEL_Z_MAX,
		.parameterPointer = &MotionZ.rampPrint.accel
};

static setting_t print_decel_z = {
		.name = "print_decel_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_PRINT_DECEL_Z,
		.config.min = PARAMETER_PRINT_DECEL_Z_MIN,
		.config.max = PARAMETER_PRINT_DECEL_Z_MAX,
		.parameterPointer = &MotionZ.rampPrint.decel
};

/* JOG */
static setting_t jog_speed_z = {
		.name = "jog_speed_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_JOG_SPEED_Z,
		.config.min = PARAMETER_JOG_SPEED_Z_MIN,
		.config.max = PARAMETER_JOG_SPEED_Z_MAX,
		.parameterPointer = &MotionZ.rampJog.speed
};

static setting_t jog_accel_z = {
		.name = "jog_accel_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_JOG_ACCEL_Z,
		.config.min = PARAMETER_JOG_ACCEL_Z_MIN,
		.config.max = PARAMETER_JOG_ACCEL_Z_MAX,
		.parameterPointer = &MotionZ.rampJog.accel
};

static setting_t jog_decel_z = {
		.name = "jog_decel_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_JOG_DECEL_Z,
		.config.min = PARAMETER_JOG_DECEL_Z_MIN,
		.config.max = PARAMETER_JOG_DECEL_Z_MAX,
		.parameterPointer = &MotionZ.rampJog.decel
};

/* MOVE */

static setting_t move_speed_z = {
		.name = "move_speed_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_MOVE_SPEED_Z,
		.config.min = PARAMETER_MOVE_SPEED_Z_MIN,
		.config.max = PARAMETER_MOVE_SPEED_Z_MAX,
		.parameterPointer = &MotionZ.rampMove.speed
};


static setting_t move_accel_z = {
		.name = "move_accel_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_MOVE_ACCEL_Z,
		.config.min = PARAMETER_MOVE_ACCEL_Z_MIN,
		.config.max = PARAMETER_MOVE_ACCEL_Z_MAX,
		.parameterPointer = &MotionZ.rampMove.accel
};

static setting_t move_decel_z = {
		.name = "move_decel_z",
		.type = VAR_UINT32 | MASTER_VALUE,
		.config.def = PARAMETER_MOVE_DECEL_Z,
		.config.min = PARAMETER_MOVE_DECEL_Z_MIN,
		.config.max = PARAMETER_MOVE_DECEL_Z_MAX,
		.parameterPointer = &MotionZ.rampMove.decel
};

/* Table contain all parameters */

const setting_t *parametersTable[] = {
		&stepper_factor,
		&print_start_position,
		&eject_trigger_position,
		&eject_position,
		&pe_lower_limit,
		&pe_upper_limit,
		&timeout_motion_y,
		&home_speed_y,
		&print_speed_y,
		&print_accel_y,
		&print_decel_y,
		&move_speed_y,
		&move_accel_y,
		&move_decel_y,
		&jog_speed_y,
		&jog_accel_y,
		&jog_decel_y,
		&timeout_motion_z,
		&home_speed_z,
		&print_speed_z,
		&print_accel_z,
		&print_decel_z,
		&move_speed_z,
		&move_accel_z,
		&move_decel_z,
		&jog_speed_z,
		&jog_accel_z,
		&jog_decel_z,
} ;

const uint16_t paramsTableLen = ARRAYLEN(parametersTable);

