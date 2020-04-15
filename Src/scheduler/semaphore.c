/*
 * semaphore.c
 *
 *  Created on: 15.04.2020
 *      Author: Adrian
 */
#include "semaphore.h"

void SemActivate(Semaphore *s, uint8_t process)
{
	s->active = 1;
	s->process = process;
}

//void SemActivate(Semaphore *s)
//{
//	s->active = 1;
//}

void SemDeactivate(Semaphore *s)
{
	s->active = 0;
}

uint8_t SemCheck(Semaphore *s)
{
	return s->active;
}

uint8_t SemGetProcess(Semaphore *s)
{
	return s->process;
}
void SemSetProcess(Semaphore *s, uint8_t process)
{
	s->process = process;
}

void SemSetTime(Semaphore *s, uint32_t time)
{
	s->start_time = time;
}

uint32_t SemGetTime(Semaphore *s)
{
	return s->start_time;
}
