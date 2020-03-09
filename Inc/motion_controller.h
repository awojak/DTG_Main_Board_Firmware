/*
 * motion_controller.h
 *
 *  Created on: 09.12.2019
 *      Author: Neo
 */

#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_

#include "main.h"

#define TRUE 1
#define FALSE 0

/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct {
  //! What part of the speed ramp we are in.
  unsigned char run_state : 3;
  //! Direction stepper motor should move.
  unsigned char dir : 1;
  //! Peroid of next timer delay. At start this value set the accelration rate.
  unsigned int step_delay;
  //! What step_pos to start decelaration
  unsigned int decel_start;
  //! Sets deceleration rate.
  signed int decel_val;
  //! Minimum time delay (max speed)
  signed int min_delay;
  //! Counter used when accelerateing/decelerateing to calculate step_delay.
  signed int accel_count;
} speedRampData;

/*! \brief Holding configuration of the motion controller
 *
 *  Contains configuration data used by motion controller functions
 *  Data is written to it by MotionInitialize() and is used for any motion functions.
 */
typedef struct{
	//! Back/Down limit switch GPIO port and pin, should be configured as input
	GPIO_TypeDef *back_down_limit_gpio_port;
	uint16_t back_down_limit_pin;

	//! Front/Up limit switch GPIO port and pin, should be configured as input
	GPIO_TypeDef *front_up_limit_gpio_port;
	uint16_t front_up_limit_pin;

	//! DIR GPIO port and pin to generate DIR signal, should be configured as output
	GPIO_TypeDef *dir_gpio_port;
	uint16_t dir_pin;

	//! STEP GPIO port and pin to generate STEP signal, should be configured as output
	GPIO_TypeDef *step_gpio_port;
	uint16_t step_pin;

	//! Handler for timer instance, use to generate interrupt, should be stopped and configured as up counting with interrupt
	TIM_HandleTypeDef *timer;

	//! Handler for UART instance to communicate with TMC2209
	UART_HandleTypeDef *uart;

	//! TMC driver UART address to communicate with
	uint8_t	tmc_addr;

	//! Contains data used by timer interrupt to calculate speed profile.
	speedRampData ramp_data;

	//! Actual position of the axis
	int position;

	//! True when stepper motor is running.
	uint8_t running:1;

	//! True if initialized successfully
	uint8_t initialized:1;


} MotionController;

// Direction of stepper motor movement
#define CW  0
#define CCW 1

/*! \Brief Frequency of timer1 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1-FREQ 460750)
#define T1_FREQ 500000

//! Number of (full)steps per round on stepper motor in use.
#define SPR 3200
#define FULLSTEPS

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000

// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

void MotionControllerInitialize(MotionController *m);
void MotionMoveSteps(MotionController *m, signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
void MotionMoveSpeed(MotionController *m, unsigned char dir, unsigned int accel, unsigned int speed);
void MotionMoveStop(MotionController *m, unsigned char mode, unsigned int decel);
void MotionUpdate(MotionController *m);

#endif /* MOTION_CONTROLLER_H_ */
