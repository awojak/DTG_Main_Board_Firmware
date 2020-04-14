/*
 * task_scheduler.c
 *
 *  Created on: 22.10.2019
 *      Author: Neo
 */

#include "task_scheduler.h"

//Tick variable
volatile unsigned int tick = 0;

/**
 * Initialize scheduler before add tasks
 */
void SchedulerInit(SchedulerTasks* sts)
{
	sts->tasks_count = 0;
}

void TaskTick()
{
	//increment scheduler tick
	tick++;
}

/**
 * Add task to scheduler queue with priority sort
 */
int AddToQueue(SchedulerTasks* sts, Task *t)
{
	int i;
	Task *tmp, *tmp2;

	for(i=0; i < sts->tasks_count; i++)
	{
		if(sts->tasks[i] == t)
		{
			//Task exist, exit function
			return 1;
		}
	}

	if(sts->tasks_count < MAX_TASKS)
	{
		if(sts->tasks_count == 0)
		{
			sts->tasks[0] = t;
			sts->tasks_count++;
		} else
		{
			for(i=0; i < sts->tasks_count; i++)
			{
				//Looking for place with lower priority
				if(sts->tasks[i]->priority > t->priority)
				{
					tmp = sts->tasks[i];
					sts->tasks[i] = t;
					sts->tasks_count++;

					//Move tasks
					for(++i;i < sts->tasks_count; i++)
					{
						tmp2 = sts->tasks[i];
						sts->tasks[i] = tmp;
						tmp = tmp2;
					}
					return 0;
				}
			}
			if(sts->tasks_count == i)
			{
				//Add task at the end of the queue
				sts->tasks[sts->tasks_count] = t;
				sts->tasks_count++;
			}
		}
		return 0;
	} else
	{
		return -1;
	}
}
/**
 * Remove task from queue
 */
int RemoveFromQueue(SchedulerTasks* sts, Task *t)
{
	int i;
	for(i=0; i < sts->tasks_count; i++)
	{
		//Looking for task index
		if(sts->tasks[i] == t)
		{
			sts->tasks_count--;
			//Move tasks
			for(; i < sts->tasks_count; i++)
			{
				sts->tasks[i] = sts->tasks[i+1];
			}
			return 0;
		}
	}
	return -1; //don't found task
}

TaskRemoveState TaskRemove(SchedulerTasks* sts, Task *t)
{
	if(RemoveFromQueue(sts, t) == 0)
	{
		return RemovedTaskSuccessfully;
	} else
	{
		//If task not exist in queue
		return RemovedTaskUnsuccessfully;
	}
}

int TaskChangePriority(SchedulerTasks* sts, Task *t, unsigned char priority)
{
	t->priority = priority;

	int i;
	//Looking if task exist in sts
	for(i=0; i<sts->tasks_count; i++)
	{
		if(sts->tasks[i] == t)
		{
			if(RemoveFromQueue(sts, t) == 0)
			{
				if(AddToQueue(sts, t) != 0)
					return -1;
			} else
				return -1;
		}
	}
	return 0;
}

TaskCreateState TaskCreate(SchedulerTasks* sts, Task *t, void *fun_ptr, unsigned char priority)
{
	unsigned char ret;
	t->active = TaskInactive;
	t->do_task = 0;
	t->fun_ptr = fun_ptr;
	t->period = -1;
	t->priority = priority;
	t->next_exe = 0;
	ret = AddToQueue(sts, t);

	if(ret == 0)
	{
		//Added to queue
		return CreatedTaskSuccessfully;
	} else if(ret == 1)
	{
		//Already exist in queue
		return CreatedTaskExist;
	} else
	{
		//No more space for new tasks
		return CreatedTaskUnsuccessfully;
	}
}

unsigned int GetTicks()
{
	return tick;
}

/**
 * Start task once after delay
 */
void TaskStartOnceDelay(Task* t, unsigned int delay)
{
	t->active = TaskActive;
	t->repeatedly = 0;
	t->next_exe = tick + delay;
	t->period = 0;
}

/**
 * Start task once normal with priority
 */
void TaskStartOnce(Task* t)
{
	t->active = TaskActive;
	t->repeatedly = 0;
	t->next_exe = 0;
	t->period = 0;
}

/**
 * Start task repeatedly
 */
void TaskStartRepeatedly(Task* t, int period)
{
	//TODO check if task exist and is added to scheduler
	t->active = TaskActive;
	t->period = period;
	t->repeatedly = 1;
	t->next_exe = 0;
}

/**
 * Start event task, not periodically
 */
void TaskEventStart(Task* t)
{
	//TODO check if task exist and is added to scheduler
	t->active = TaskActive;
	t->period = -1;
}

/**
 * Do immediately task
 */
void TaskEventTrigger(Task* t)
{
	//TODO check if task exist and is added to scheduler
	t->do_task = 1;
}
/**
 * Stop task
 */
void TaskStop(Task* t)
{
	//TODO check if task exist and is added to scheduler
	t->active = TaskInactive;
}
/**
 * Scheduler for tasks
 */
void Scheduler(SchedulerTasks* sts)
{
	int i,j;
	for(i=0; i<sts->tasks_count; i++)
	{
		//Inside loop for task with higher priority
		for(j=0; j < sts->tasks_count; j++)
		{
			if(sts->tasks[j]->priority < sts->tasks[i]->priority)
			{
				//found task with higher priority then actual task
				if(sts->tasks[j]->active == TaskActive)
				{
					if(sts->tasks[j]->do_task)
					{
						//Do task immediately
						sts->tasks[j]->fun_ptr();
						//Reset do task to wait for next event
						sts->tasks[j]->do_task = 0;
						continue;
					}

					if((tick >= sts->tasks[j]->next_exe) && (sts->tasks[j]->period>=0))
					{
						if(sts->tasks[j]->repeatedly)
						{
							//Save next execution
							sts->tasks[j]->next_exe = tick + sts->tasks[j]->period;
						} else {
							sts->tasks[j]->active = TaskInactive;
						}
						//Do task
						sts->tasks[j]->fun_ptr();
					}
				}
			} else
			{
				//Task with higher priority should be on the beginning of the queue
				break;
			}
		}

		// Others tasks
		if(sts->tasks[i]->active == TaskActive)
		{
			if(sts->tasks[i]->do_task)
			{
				//Do task immediately
				sts->tasks[i]->fun_ptr();
				//Reset do task to wait for next event
				sts->tasks[i]->do_task = 0;
				continue;
			}

			if((tick >= sts->tasks[i]->next_exe) && (sts->tasks[i]->period>=0))
			{
				if(sts->tasks[i]->repeatedly)
				{
					//Save next execution
					sts->tasks[i]->next_exe = tick + sts->tasks[i]->period;
				} else {
					sts->tasks[i]->active = TaskInactive;
				}
				//Do task
				sts->tasks[i]->fun_ptr();
			}
		}
	}
}
