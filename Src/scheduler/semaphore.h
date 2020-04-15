/*
 * semaphore.h
 *
 *  Created on: 15.04.2020
 *      Author: Adrian
 */

#ifndef SCHEDULER_SEMAPHORE_H_
#define SCHEDULER_SEMAPHORE_H_

#include "main.h"

typedef struct sSemaphore {
	uint8_t active;
	uint8_t process;
	uint32_t start_time; //for timeout purpose
} Semaphore;

uint8_t SemCheck(Semaphore *s);
uint8_t SemGetProcess(Semaphore *s);
void SemSetProcess(Semaphore *s, uint8_t process);
void SemSetTime(Semaphore *s, uint32_t time);
uint32_t SemGetTime(Semaphore *s);
//void SemActivate(Semaphore *s);
void SemActivate(Semaphore *s, uint8_t process);
void SemDeactivate(Semaphore *s);
#endif /* SCHEDULER_SEMAPHORE_H_ */
