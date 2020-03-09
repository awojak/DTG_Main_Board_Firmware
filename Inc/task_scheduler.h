/**
  ******************************************************************************
  * @file           : task_scheduler.h
  * @brief          : Header for task_scheduler.c file.
  *                   This file contains the common defines of the ???
  ******************************************************************************

  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TASKS 10

//simple task structure
typedef enum eTaskActiveState {TaskInactive, TaskActive} TaskActiveState;
typedef enum eTaskCreateState {CreatedTaskUnsuccessfully = -1, CreatedTaskSuccessfully = 0, CreatedTaskExist} TaskCreateState;
typedef enum eTaskRemoveState {RemovedTaskUnsuccessfully = -1, RemovedTaskSuccessfully = 0} TaskRemoveState;

typedef struct sTask {
	int id;			//32 bit task id
	TaskActiveState active;	//0 - inactive, others - active
	unsigned char priority; // 0 - highest, 254 - lowest
	int period; // -1 - don't repeat, 0 - repeat every tick, >0 - repeat every value in ms.
	unsigned int next_exe;	// next execution time
	volatile char do_task;	//0 - wait, others - execute immediately
	void (*fun_ptr)(void); //function for task
	//TODO: Implement timeout function to check if everything ok with task
	//TODO: Add scheduler task pointer to now where task is added
} Task;

typedef struct sScheduler {
	Task *tasks[MAX_TASKS];
	unsigned char tasks_count;
} SchedulerTasks;

void SchedulerInit(SchedulerTasks* sts);
void TaskTick();
TaskCreateState TaskCreate(SchedulerTasks* sts, Task *t, void *fun_ptr, unsigned char priority);
TaskRemoveState TaskRemove(SchedulerTasks* sts, Task *t);
int TaskChangePriority(SchedulerTasks* sts, Task *t, unsigned char priority);
void TaskStart(Task* t, int period);
void TaskEventStart(Task* t);
void TaskEventTrigger(Task* t);
void TaskStop(Task* t);
void Scheduler(SchedulerTasks* sts);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SCHEDULER_H */
