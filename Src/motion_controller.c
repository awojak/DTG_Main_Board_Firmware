/*
 * motion_controller.c
 *
 *  Created on: 09.12.2019
 *      Author: Neo
 */

#include "motion_controller.h"
#include "tools.h"
#include "tmc2209.h"

/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step  Number of steps to move (pos - CW, neg - CCW).
 *  \param accel  Accelration to use, in 0.01*rad/sec^2.
 *  \param decel  Decelration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 */
void MotionMoveSteps(MotionController *m, signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
  //! Number of steps before we hit max speed.
  unsigned int max_s_lim;
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  unsigned int accel_lim;

  // Set direction from sign on step value.
  if(step < 0){
    m->ramp_data.dir = CCW;
    step = -step;
    HAL_GPIO_WritePin(m->dir_gpio_port, m->dir_pin, GPIO_PIN_RESET);
  }
  else{
    m->ramp_data.dir = CW;
    HAL_GPIO_WritePin(m->dir_gpio_port, m->dir_pin, GPIO_PIN_SET);
  }

  // If moving only 1 step.
  if(step == 1){
    // Move one step...
    m->ramp_data.accel_count = -1;
    // ...in DECEL state.
    m->ramp_data.run_state = DECEL;
    // Just a short delay so main() can act on 'running'.
    m->ramp_data.step_delay = 1000;
    m->running = TRUE;

    m->timer->Instance->ARR = 10;
    // Run Timer
    HAL_TIM_Base_Start_IT(m->timer);
  }
  // Only move if number of steps to move is not zero.
  else if(step != 0){
    // Refer to documentation for detailed information about these calculations.

    // Set max speed limit, by calc min_delay to use in timer.
    // min_delay = (alpha / tt)/ w
    m->ramp_data.min_delay = (int)(A_T_x100 / speed);

    // Set accelration by calc the first (c0) step delay .
    // step_delay = 1/tt * Sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * Sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    m->ramp_data.step_delay = (unsigned int)((T1_FREQ_148 * Sqrt(A_SQ / accel))/100);

    // Find out after how many steps does the speed hit the max speed limit.
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (unsigned int)((long)speed*speed/(long)(((long)A_x20000*accel)/100));
    // If we hit max speed limit before 0,5 step it will round to 0.
    // But in practice we need to move atleast 1 step to get any speed at all.
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = (unsigned int)(((long)step*decel) / (accel+decel));
    // We must accelrate at least 1 step before we can start deceleration.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Use the limit we hit first to calc decel.
    if(accel_lim <= max_s_lim){
      m->ramp_data.decel_val = (int)(accel_lim - step);
    }
    else{
      m->ramp_data.decel_val = -(int)(((long)max_s_lim*accel)/decel);
    }
    // We must decelrate at least 1 step to stop.
    if(m->ramp_data.decel_val == 0){
      m->ramp_data.decel_val = -1;
    }

    // Find step to start decleration.
    m->ramp_data.decel_start = (unsigned int)(step + m->ramp_data.decel_val);

    // If the maximum speed is so low that we dont need to go via accelration state.
    if(m->ramp_data.step_delay <= m->ramp_data.min_delay){
      m->ramp_data.step_delay = m->ramp_data.min_delay;
      m->ramp_data.run_state = RUN;
    }
    else{
      m->ramp_data.run_state = ACCEL;
    }

    // If the minimum speed is to low
    if(m->ramp_data.step_delay >= 65536){
      m->ramp_data.step_delay = 65535;
    }

    // Reset counter.
    m->ramp_data.accel_count = 0;
    m->running = TRUE;

    m->timer->Instance->ARR = 10;
    // Run Timer
    HAL_TIM_Base_Start_IT(m->timer);
  }
}

/*! \brief Read Register
 *
 *
 *  reg - register
 *  data -
 */
HAL_StatusTypeDef MotionReadRegister(MotionController *m, uint8_t reg, uint32_t *data)
{
	HAL_StatusTypeDef status;
	m->uart_tx[2] = reg; //register
	//calculate CRC and write to last byte
	calcCRC(m->uart_tx, 4);
	//send by UART
	HAL_UART_Transmit(m->uart, m->uart_tx, 4, 10);

	//TODO how much to wait?
	//Read 8 bytes
	status = HAL_UART_Receive(m->uart, m->uart_rx, 8, 10);

	if(status == HAL_OK)
	{
			*data = (uint32_t)((m->uart_rx[3] << 24) |  (m->uart_rx[4] << 16) | (m->uart_rx[5] << 8) | m->uart_rx[6]);
	}

	//TODO What to do if status not OK?
	return status;
}

/*! \brief Write Register
 *
 *
 */

HAL_StatusTypeDef MotionWriteRegister(MotionController *m, uint8_t reg, uint32_t data)
{
	HAL_StatusTypeDef status;
	m->uart_tx[2] = reg | 0x80; //register + write bit
	m->uart_tx[3] = (uint8_t)((data >> 24) & 0xFF);
	m->uart_tx[4] = (uint8_t)((data >> 16) & 0xFF);
	m->uart_tx[5] = (uint8_t)((data >> 8) & 0xFF);
	m->uart_tx[6] = (uint8_t)((data) & 0xFF);
	//calculate CRC and write to last byte
	calcCRC(m->uart_tx, 8);
	//send by UART
	status = HAL_UART_Transmit(m->uart, m->uart_tx, 8, 10);
	//TODO What to do if status not OK?
	return status;
}

/*! \brief Init of Motion Controller
 *
 *  Initialize Motion Controller variables to correct states
 */
void MotionControllerInitialize(MotionController *m)
{
	// Tells what part of speed ramp we are in
	m->ramp_data.run_state = STOP;
	m->running = FALSE;

	//UART communication fill first byte
	//sync + reserved
	m->uart_tx[0] = 0x65;
	m->uart_tx[1] = m->tmc_addr;

	//Configure TMC by UART
	uint32_t data;
	data = (GCONF_PDN_DISABLE) | (GCONF_MSTEP_REG_SELECT);
	MotionWriteRegister(m, GCONF, data);
	data = 0;
	//Chech if write function work
	MotionReadRegister(m, IFCNT, &data);
	data = 0;
	MotionReadRegister(m, GCONF, &data);
	data = (1 << 5);
	MotionWriteRegister(m, IHOLD_IRUN, data);
	data = 0;
	MotionReadRegister(m, IFCNT, &data);

	//TODO Configuracja dziala ale jest jeden problem z odbiorem danych, zawsze pierwszy bajt zawiera jakieœ smieci.

	//VREF current set
	//Can be disabled by GCONF.i_scale_analog
	/*
	 * UART interface IHOLD_IRUN
	TPOWERDOWN
	OTP
	IRUN, IHOLD:
	1/32 to 32/32 of full
	scale current.
	- Fine programming of run and
	hold (stand still) current
	- Change IRUN for situation
	specific motor current
	- Set OTP options
	 */
}



/*! \brief Motion Update execute in timer interrupt
 *
 *  Increments/decrements the position of the stepper motor
 *  exept after last position, when it stops.
 *  The \ref step_delay defines the period of this interrupt
 *  and controls the speed of the stepper motor.
 *  A new step delay is calculated to follow wanted speed profile
 *  on basis of accel/decel parameters.
 */
void MotionUpdate(MotionController *m)
{
  // Holds next delay period.
  unsigned int new_step_delay=0;
  // Remember the last step delay used when accelerating.
  //static int last_accel_delay;
  // Counting steps when moving.
  static unsigned int step_count = 0;
  // Keep track of remainder from new_step-delay calculation to incrase accurancy
  static unsigned int rest = 0;

  m->timer->Instance->ARR = m->ramp_data.step_delay;

  switch(m->ramp_data.run_state) {
    case STOP:
      step_count = 0;
      rest = 0;
      // Stop Timer/Counter 1.
      HAL_TIM_Base_Stop_IT(m->timer);
      m->running = FALSE;
      break;

    case ACCEL:
    	HAL_GPIO_TogglePin(m->step_gpio_port, m->step_pin);
      //sm_driver_StepCounter(m->ramp_data.dir);
      step_count++;
      m->ramp_data.accel_count++;
      new_step_delay = (unsigned int)(m->ramp_data.step_delay - (((2 * (long)m->ramp_data.step_delay) + rest)/(4 * m->ramp_data.accel_count + 1)));
      rest = (unsigned int)(((2 * (long)m->ramp_data.step_delay)+rest)%(4 * m->ramp_data.accel_count + 1));
      // Chech if we should start decelration.
      if(step_count >= m->ramp_data.decel_start) {
        m->ramp_data.accel_count = m->ramp_data.decel_val;
        rest = 0;
        m->ramp_data.run_state = DECEL;
      }
      // Chech if we hitted max speed.
      else if(new_step_delay <= m->ramp_data.min_delay) {
        //last_accel_delay = new_step_delay;
        new_step_delay = m->ramp_data.min_delay;
        rest = 0;
        m->ramp_data.run_state = RUN;
      }
      break;

    case RUN:
    	HAL_GPIO_TogglePin(m->step_gpio_port, m->step_pin);
      //sm_driver_StepCounter(m->ramp_data.dir);
      step_count++;
      new_step_delay = m->ramp_data.min_delay;
      // Chech if we should start decelration.
      if(step_count >= m->ramp_data.decel_start) {
        m->ramp_data.accel_count = m->ramp_data.decel_val;
        // Start decelration with same delay as accel ended with.
        //new_step_delay = last_accel_delay;
        m->ramp_data.run_state = DECEL;
      }
      break;

    case DECEL:
    	HAL_GPIO_TogglePin(m->step_gpio_port, m->step_pin);
      //sm_driver_StepCounter(m->ramp_data.dir);
      step_count++;
      m->ramp_data.accel_count++;
      new_step_delay = (unsigned int)( m->ramp_data.step_delay - (int)(((2 * (long)m->ramp_data.step_delay) + (long)rest)/(4 * m->ramp_data.accel_count + 1)));
      rest = (unsigned int)(((2 * (long)m->ramp_data.step_delay)+(long)rest)%(4 * m->ramp_data.accel_count + 1));
      // Check if we at last step
      if(m->ramp_data.accel_count >= 0){
        m->ramp_data.run_state = STOP;
      }
      break;
  }
  m->ramp_data.step_delay = new_step_delay;
}

/*! \brief Motion Move Speed
 *
 *  Move motor at constant speed, use acceleration at the beginning
 *  To stop motor use MotionMoveStop() function
 */
void MotionMoveSpeed(MotionController *m, unsigned char dir, unsigned int accel, unsigned int speed)
{

}

/*! \brief Motion Move Stop
 *
 *  Stop motor, use deceleration or stop immediately
 *  Mode determines the behavior
 */
void MotionMoveStop(MotionController *m, unsigned char mode, unsigned int decel)
{

}

