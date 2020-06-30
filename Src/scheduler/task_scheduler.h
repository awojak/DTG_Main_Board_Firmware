/**
  ******************************************************************************
  * @file           : task_scheduler.h
  * @brief          : Header for task_scheduler.c file.
  *                   This file contains the common defines of the ???
  ******************************************************************************

  *
  ******************************************************************************
  */

/* How to use it:
 * 1. Create SchedulerTasks instance
 * 2. Create Tasks variable and implementation
 * 3. Repetitively execute TaskTick() function, for example in SysTick interrupt. Default use 1kHz timer to obtain 1ms resolution.
 * 4. Initialize scheduler use SchedulerInit() function, argument is SchedulerTasks instance
 * 5. Use TaskCreate function to create task, arguments:
 * 		*sts 	- pointer to instance of scheduler, where task will be added
 * 		*t		- pointer to task instance
 * 		*fun_ptr - pointer to task function
 * 		priority - priority of task
 * 	6. Start Task, use TaskStart() to execute task repeatedly with delay, or
 * 		use TaskEventStart() to start task only with TaskEventTrigger() function,
 * 		one TaskEventTrigger, one Task Execution.
 * 	7. Put Scheduler() function in infinite loop e.g. (while(1) { }), scheduler don't have infinite loop
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TASKS 256

//simple task structure
typedef enum eTaskActiveState {TaskInactive, TaskActive} TaskActiveState;
typedef enum eTaskCreateState {CreatedTaskUnsuccessfully = -1, CreatedTaskSuccessfully = 0, CreatedTaskExist} TaskCreateState;
typedef enum eTaskRemoveState {RemovedTaskUnsuccessfully = -1, RemovedTaskSuccessfully = 0} TaskRemoveState;

typedef struct sTask {
	int id;			//32 bit task id
	TaskActiveState active;	//0 - inactive, others - active
	unsigned char priority; // 0 - highest, 255 - lowest
	int period; // -1 - don't repeat, 0 - repeat every tick, >0 - repeat every value in ms.
	unsigned char repeatedly; //0 - execute task once, 1 - execute task repeatedly
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
unsigned int Ticks();
TaskCreateState TaskCreate(SchedulerTasks* sts, Task *t, void *fun_ptr, unsigned char priority);
TaskRemoveState TaskRemove(SchedulerTasks* sts, Task *t);
int TaskChangePriority(SchedulerTasks* sts, Task *t, unsigned char priority);
void TaskStartRepeatedly(Task* t, int period);
void TaskStartOnceDelay(Task* t, unsigned int delay);
void TaskStartOnce(Task* t);
void TaskEventStart(Task* t);
void TaskEventTrigger(Task* t);
void TaskStop(Task* t);
void Scheduler(SchedulerTasks* sts);
unsigned int GetTicks();
#ifdef __cplusplus
}
#endif

#endif /* TASK_SCHEDULER_H */
