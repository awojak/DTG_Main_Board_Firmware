/*
 * motion_controller.c
 *
 *  Created on: 09.12.2019
 *      Author: Neo
 */

#include "motion_controller.h"

#include "tmc2209.h"

#include "../scheduler/task_scheduler.h"

#include "../common/tools.h"

uint8_t isHomed(MotionController *m)
{
	return m->homed;
}
uint8_t checkErrors(MotionController *m)
{
	//TODO return errors
	return 0;
}


static void SetCCW_DIR(MotionController *m)
{
    m->ramp_data.dir = CCW;
    HAL_GPIO_WritePin(m->dir_gpio_port, m->dir_pin, GPIO_PIN_RESET);
}

static void SetCW_DIR(MotionController *m)
{
    m->ramp_data.dir = CW;
    HAL_GPIO_WritePin(m->dir_gpio_port, m->dir_pin, GPIO_PIN_SET);
}

/**
 * Require to execute every 50ms for better performance
 */
void MotionProcess(MotionController *m)
{
	//Check Errors, Problem, State, Position etc
	//Check TMC2209 state

	//Home part
	if(SemCheck(&m->sem_home))
	{
		if(m->ramp_data.run_state == STOP)
		{
			//Check if Limit Switch is active
			if((m->back_down_limit_gpio_port->IDR & m->back_down_limit_pin) == (uint32_t)m->back_down_limit_active_level)
			{
				//Set current position to 0
				m->position = 0;
				m->homed  = 1;
				//Already at limit switch,
				SemDeactivate(&m->sem_home);
				//Go to standby position
				MotionMovePos(m, m->standby_position);

			} else {
				//Error
				SemDeactivate(&m->sem_home);
			}
		} else {
			if(GetTicks() - SemGetTime(&m->sem_home) >= m->home_timeout)
			{
				//TODO Error timeout
				SemDeactivate(&m->sem_home);
				MotionMoveStop(m, 1, 100);
			}
		}
	}
}

/**
 *	Motion Home, scheduler version
 */

void MotionHome(MotionController *m)
{
	uint8_t dir;
	// Check if ready to Home
	if(!m->initialized)
		return;

	if(m->forward_dir==CW)
		dir=CCW;
	else
		dir=CW;
	if(!isMotionEnable(m))
		MotionEnable(m);
	//Move in opposite direction to forward direction at constant speed
	MotionMove(m, dir,m->rampHome.accel, m->rampHome.speed);

	SemActivate(&m->sem_home, 0);
	SemSetTime(&m->sem_home, GetTicks());

	//Rest code will be executed in MotionProcess
}

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
void MotionMovePrinting(MotionController *m, signed int position)
{
	//TODO If already in motion, update just position
  //! Number of steps before we hit max speed.
  unsigned int max_s_lim;
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  unsigned int accel_lim;

  int step;

  step = position - m->position;

  // Set direction from sign on step value.
  if(step < 0){
    step = -step;
    SetCCW_DIR(m);
  }
  else{
    SetCW_DIR(m);
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
    m->ramp_data.min_delay = (int)(A_T_x100 / m->rampPrint.speed);

    // Set accelration by calc the first (c0) step delay .
    // step_delay = 1/tt * Sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * Sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    m->ramp_data.step_delay = (unsigned int)((T1_FREQ_148 * Sqrt(A_SQ / m->rampPrint.accel))/100);

    // Find out after how many steps does the speed hit the max speed limit.
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (unsigned int)((long)(m->rampPrint.speed)*(m->rampPrint.speed)/(long)(((long)A_x20000*(m->rampPrint.accel))/100));
    // If we hit max speed limit before 0,5 step it will round to 0.
    // But in practice we need to move atleast 1 step to get any speed at all.
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = (unsigned int)(((long)step*(m->rampPrint.decel)) / (m->rampPrint.accel+m->rampPrint.decel));
    // We must accelrate at least 1 step before we can start deceleration.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Use the limit we hit first to calc decel.
    if(accel_lim <= max_s_lim){
      m->ramp_data.decel_val = (int)(accel_lim - step);
    }
    else{
      m->ramp_data.decel_val = -(int)(((long)max_s_lim*(m->rampPrint.accel))/(m->rampPrint.decel));
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

  //TODO Check status
	if(m->initialized == 0)
		return ;

	if(m->homed == 0)
		return ;

  // Set direction from sign on step value.
  if(step < 0){
    step = -step;
    SetCCW_DIR(m);
  }
  else{
    SetCW_DIR(m);
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
	//clear buffer read one byte
	status = HAL_UART_Receive(m->uart, m->uart_rx, 1, 10);

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
	m->homed = FALSE;

	//Initialize semaphore
	SemDeactivate(&m->sem_home);

	MotionDisable(m);

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
	//Set IHOLD 8, IRUN 32
	data = (1<<3)|(1 << 12);
	MotionWriteRegister(m, IHOLD_IRUN, data);
	data = 0;
	MotionReadRegister(m, IFCNT, &data);
	data = 0;
	MotionReadRegister(m, CHOPCONF, &data);
	//Reset microstepping
	//data &= ~(1<<28 | 1<<27 | 1<<26 | 1<<25 | 1<<24);
	//Set 16 microsteps, dedge
	//data |= (1<<26 | 1<<29);
	data = 0x24010053;
	MotionWriteRegister(m, CHOPCONF, data);
	data = 0;
	MotionReadRegister(m, IFCNT, &data);
	data = 0;
	MotionReadRegister(m, CHOPCONF, &data);

	//Read CHOPCONF
	//1000 0000000010000000001010011
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

	/* If everything go well */
	m->initialized = TRUE;
}

uint8_t MotionSetIHOLD(MotionController *m, uint32_t ihold)
{
	uint32_t ifcnt = 0;
	uint32_t tmp = 0;
	uint32_t data = 0;

	MotionReadRegister(m, IFCNT, &ifcnt);
	MotionReadRegister(m, IHOLD_IRUN, &data);
	data &= ~(0x1F);
	data |= (ihold & 0x1F) ;
	MotionWriteRegister(m, IHOLD_IRUN, data);
	MotionReadRegister(m, IFCNT, &tmp);

	if(ifcnt!=tmp)
		return 0;

	return 1;
}

uint8_t MotionSetIRUN(MotionController *m, uint32_t irun)
{
	uint32_t ifcnt = 0;
	uint32_t tmp = 0;
	uint32_t data = 0;

	MotionReadRegister(m, IFCNT, &ifcnt);
	MotionReadRegister(m, IHOLD_IRUN, &data);
	data &= ~(0x1F00);
	data |= ((irun<<8) & 0x1F00) ;
	MotionWriteRegister(m, IHOLD_IRUN, data);
	MotionReadRegister(m, IFCNT, &tmp);

	if(ifcnt!=tmp)
		return 0;

	return 1;
}

//Check limit switches and emergency
static inline void limit(MotionController *m)
{
	//Don't check emergency in Motion Controller, just disable motion if emergency active
	//if((EMERGENCY_GPIO_Port->IDR & EMERGENCY_Pin) == (uint32_t) GPIO_PIN_SET)
	//{
	//	m->ramp_data.run_state = STOP;
	//	m->homed = FALSE;
	//}


	// If not safe mode active, stop Move in not safe direction
	if(m->not_safe)
	{
		if(m->safe_dir)
		{
			//Forward is safe
			if(m->ramp_data.dir != m->forward_dir)
				m->ramp_data.run_state = STOP;
		} else
		{
			//Backward is safe
			if(m->ramp_data.dir == m->forward_dir)
				m->ramp_data.run_state = STOP;
		}
		if(m->ramp_data.dir != (m->forward_dir == m->safe_dir))
			m->ramp_data.run_state = STOP;
	}

	if(HAL_GPIO_ReadPin(m->back_down_limit_gpio_port, m->back_down_limit_pin) == (uint32_t)m->back_down_limit_active_level)
	{
		if(m->ramp_data.dir != m->forward_dir)
			m->ramp_data.run_state = STOP;
	}

	if(HAL_GPIO_ReadPin(m->front_up_limit_gpio_port, m->front_up_limit_pin) == (uint32_t)m->front_up_limit_active_level)
	{
		if(m->ramp_data.dir == m->forward_dir)
			m->ramp_data.run_state = STOP;
	}
}

static inline void step(MotionController *m)
{
	//TODO remove hal library to increase speed
	HAL_GPIO_TogglePin(m->step_gpio_port, m->step_pin);

	//Increase step
	m->ramp_data.step_count++;

	if(m->forward_dir==m->ramp_data.dir)
		m->position++;
	else
		m->position--;
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

  //check limit switch state and emergency state
  limit(m);

  m->timer->Instance->ARR = m->ramp_data.step_delay;

  switch(m->ramp_data.run_state) {
    case STOP:
      m->ramp_data.step_count = 0;
      m->ramp_data.rest = 0;
      // Stop Timer/Counter 1.
      HAL_TIM_Base_Stop_IT(m->timer);
      m->running = FALSE;
      break;

    case ACCEL:
      step(m);
      //HAL_GPIO_TogglePin(m->step_gpio_port, m->step_pin);
      //sm_driver_StepCounter(m->ramp_data.dir);
      //step_count++;
      m->ramp_data.accel_count++;
      new_step_delay = (unsigned int)(m->ramp_data.step_delay - (((2 * (long)m->ramp_data.step_delay) + (m->ramp_data.rest))/(4 * m->ramp_data.accel_count + 1)));
      m->ramp_data.rest = (unsigned int)(((2 * (long)m->ramp_data.step_delay)+(m->ramp_data.rest))%(4 * m->ramp_data.accel_count + 1));
      // Chech if we should start decelration.
      if(m->ramp_data.step_count >= m->ramp_data.decel_start) {
        m->ramp_data.accel_count = m->ramp_data.decel_val;
        m->ramp_data.rest = 0;
        m->ramp_data.run_state = DECEL;
      }
      // Chech if we hitted max speed.
      else if(new_step_delay <= m->ramp_data.min_delay) {
        //last_accel_delay = new_step_delay;
        new_step_delay = m->ramp_data.min_delay;
        m->ramp_data.rest = 0;
        m->ramp_data.run_state = RUN;
      }
      break;

    case RUN:
      step(m);
      //HAL_GPIO_TogglePin(m->step_gpio_port, m->step_pin);
      //sm_driver_StepCounter(m->ramp_data.dir);
      //step_count++;
      new_step_delay = m->ramp_data.min_delay;
      // Chech if we should start decelration.
      if(m->ramp_data.step_count >= m->ramp_data.decel_start) {
        m->ramp_data.accel_count = m->ramp_data.decel_val;
        // Start decelration with same delay as accel ended with.
        //new_step_delay = last_accel_delay;
        m->ramp_data.run_state = DECEL;
      }
      break;

    case DECEL:
      step(m);
      //HAL_GPIO_TogglePin(m->step_gpio_port, m->step_pin);
      //sm_driver_StepCounter(m->ramp_data.dir);
      //step_count++;
      m->ramp_data.accel_count++;
      new_step_delay = (unsigned int)( m->ramp_data.step_delay - (int)(((2 * (long)m->ramp_data.step_delay) + (long)(m->ramp_data.rest))/(4 * m->ramp_data.accel_count + 1)));
      m->ramp_data.rest = (unsigned int)(((2 * (long)m->ramp_data.step_delay)+(long)(m->ramp_data.rest))%(4 * m->ramp_data.accel_count + 1));
      // Check if we at last step
      if(m->ramp_data.accel_count >= 0){
        m->ramp_data.run_state = STOP;
      }
      break;
  }
  m->ramp_data.step_delay = new_step_delay;
}

/*! \brief Motion Move Stop
 *
 *  Stop motor, use deceleration 0 or stop immediately 1
 *  Mode determines the behavior
 */
void MotionMoveStop(MotionController *m, unsigned char mode, unsigned int decel)
{
	if(mode)
	{
		m->ramp_data.run_state = STOP;
	} else {
		//TODO configure DECEL before go to DECEL state
	    m->ramp_data.run_state = DECEL;
	}
}

void MotionJogSteps(MotionController *m, signed int steps)
{
	MotionMoveSteps(m, steps, m->rampJog.accel, m->rampJog.decel, m->rampJog.speed);
}

void MotionMove(MotionController *m, uint8_t dir, unsigned int accel, unsigned int speed)
{
	if(m->initialized == 0)
		return ;

	//! Number of steps before we hit max speed.
	unsigned int max_s_lim;

	if(dir == CW)
		SetCW_DIR(m);
	else
		SetCCW_DIR(m);

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

	//Just for compatibility
	m->ramp_data.decel_val = -1;

	//No deceleration state
	m->ramp_data.decel_start = 0xFFFFFFFF;

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

/*! \brief Motion Move Speed
 *
 *  Move motor at constant speed, use acceleration at the beginning
 *  To stop motor use MotionMoveStop() function
 */
void MotionMoveSpeed(MotionController *m, unsigned char dir)
{
	MotionMove(m, dir, m->rampMove.accel, m->rampMove.speed);
}

void MotionMovePos(MotionController *m, int pos)
{
	MotionMoveSteps(m, pos - m->position, m->rampMove.accel, m->rampMove.decel, m->rampMove.speed);
}

void MotionDisable(MotionController *m)
{
	GPIO_PinState state;
	if(m->enable_active_level)
		state = GPIO_PIN_RESET;
	else
		state = GPIO_PIN_SET;

	HAL_GPIO_WritePin(m->enable_gpio_port, m->enable_pin, state);

	//m->not_safe = 1;
	m->enable_state = 0;
	m->homed = 0;

	//TODO co dalej?
}

void MotionEnable(MotionController *m)
{
	GPIO_PinState state;

	if(m->enable_active_level)
		state = GPIO_PIN_SET;
	else
		state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(m->enable_gpio_port, m->enable_pin, state);

	m->enable_state = 1;
	//TODO co dalej?
}

uint8_t isMotionEnable(MotionController *m)
{
	return m->enable_state;
}
