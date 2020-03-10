/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2019-2020 INAVFLIGHT OU
 */

/*
 * Parts of this code are based on Protothreads (coroutines)
 * written by Serge Zaitsev under MIT license below:
 *
 * Copyright (c) 2016 Serge Zaitsev
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#pragma once
#include "common/time.h"
#include "common/utils.h"
#include "drivers/time.h"
#include "scheduler/scheduler_impl.h"

#define SCHEDULER_PRIORITY_MODULATION

#define MAX_TASKS                   (16)
#define TASKID_SYSTEM               (0)
#define THREAD_TIMEOUT_INFINITE_US  ((timeUs_t)(-1L))

typedef enum {
    TASK_PRIORITY_HIGHEST   = 0,
    TASK_PRIORITY_HIGH      = 3,
    TASK_PRIORITY_MEDIUM    = 7,
    TASK_PRIORITY_LOW       = 17,
    TASK_PRIORITY_LOWEST    = 63,
    TASK_MAX_PRIORITIES     = 64
} schdTaskPriority_e;


typedef struct {
    volatile bool   state;
} schdSemaphore_t;

typedef struct {
    timeUs_t  periodUs;
    timeUs_t  lastScheduledEventUs;
    timeUs_t  lastActualEventUs;
    timeUs_t  lastEventInterval;
} schdTimer_t;



typedef enum {
    TASK_STATE_IDLE = 0,
    TASK_STATE_RUNNING,
    TASK_STATE_WAITING_SEMAPHORE,
    TASK_STATE_WAITING_TIMER,
    TASK_STATE_ZOMBIE
} taskState_e;

typedef struct {
    const char * taskName;
    taskState_e  taskState;
    uint8_t      taskPriority;
    timeUs_t     totalExecutionTime;
} taskInfo_t;

typedef enum {
    TASK_WAIT_REASON_UNKNOWN = 0,
    TASK_WAIT_REASON_EVENT,
    TASK_WAIT_REASON_TIMEOUT
} taskWaitUnblockReason_e;

typedef struct {
    void *                      objPtr;
    timeUs_t                    startTimeUs;
    timeUs_t                    timeoutUs;
    taskWaitUnblockReason_e     unblockReason;
} taskWaitCtx_t;

typedef struct {
    taskState_e     state;
    taskWaitCtx_t   wait;
    int             line;
} taskRuntimeContext_t;

typedef struct {
    timeUs_t        totalRunTimeUs;
} taskAccounting_t;

typedef void (*taskFunctionPtr_t)(taskRuntimeContext_t * ctx, timeUs_t currentTimeUs, void * param);
typedef void (*taskFunctionLegacyPtr_t)(timeUs_t currentTimeUs);

typedef struct {
    schdTimer_t             timer;
    timeUs_t                delayUs;
    taskFunctionLegacyPtr_t fn;
} taskLegacyTCB_t;

typedef struct {
    bool                    allocated;
    int                     index;
    taskRuntimeContext_t    ctx;
    taskAccounting_t        accnt;
    const char *            taskName;
    taskFunctionPtr_t       taskFn;
    void *                  taskParam;
    int                     taskPrio;
    int                     taskSchedulerPrio;
} taskTCB_t;

// Define a task (protothread) with given name
#define TASK(name) \
  void name(taskRuntimeContext_t * ctx, timeUs_t currentTimeUs, void * param)

// Must be the first line in each task/protothread
#define taskBegin()                                                         \
  bool taskYielded = true;                                                  \
  (void) taskYielded;                                                       \
  (void) currentTimeUs;                                                     \
  (void) param;                                                             \
  switch (ctx->line) {                                                      \
  case 0:

// Low-level API to create continuation, normally should not be used
#define taskLabel()                                                         \
  do {                                                                      \
    ctx->line = __LINE__;                                                   \
    FALLTHROUGH;                                                            \
    case __LINE__:;                                                         \
  } while (0)

// Suspends protothread until cond becomes true
// Condition is evaluated each call and shouldn't use any local variables
#define taskWait(condition)                                                 \
  do {                                                                      \
    taskLabel();                                                            \
    if (!(condition)) {                                                     \
      return;                                                               \
    }                                                                       \
  } while (0)

// Wait until semaphore using scheduler infrastructure
// threadYield() will continue execution only when semaphore is signalled
#define taskWaitSemaphore(semaphore)                                        \
  do {                                                                      \
    schdImpl_WaitUntilSemaphore(ctx, semaphore, THREAD_TIMEOUT_INFINITE_US);\
    taskYield();                                                            \
  } while (0)

#define taskWaitSemaphoreWithTimeoutUs(semaphore, timeoutUs, unblockReason) \
  do {                                                                      \
    schdImpl_WaitUntilSemaphore(ctx, semaphore, timeoutUs);                 \
    taskYield();                                                            \
    schdImpl_GetLastWaitUnblockReason(ctx, unblockReason);                  \
  } while (0)

// Suspends protothread until next timer event
#define taskWaitTimer(timer)                                                \
  do {                                                                      \
    schdImpl_WaitUntilTimer(ctx, timer);                                    \
    taskYield();                                                            \
  } while (0)

// Suspends photothread/task until scheduler calls it again
#define taskYield()                                                         \
  do {                                                                      \
    taskYielded = false;                                                    \
    taskLabel();                                                            \
    if (!taskYielded) {                                                     \
      return;                                                               \
    }                                                                       \
  } while (0)

// Suspends protothread for a given amount of time
#define taskDelayMs(delay)                                                  \
  do {                                                                      \
    schdImpl_WaitUntilSemaphore(ctx, NULL, MS2US(delay));                   \
    taskYield();                                                            \
  } while (0)

// Suspends protothread for a given amount of time
#define taskDelayUs(delay)                                                  \
  do {                                                                      \
    schdImpl_WaitUntilSemaphore(ctx, NULL, delay);                          \
    taskYield();                                                            \
  } while (0)

// terminates current protothread pt with the given status. Protothread won't continue
#define taskStop()                                                          \
  do {                                                                      \
    schdImpl_StopTask(ctx, 0);                                              \
    return;                                                                 \
  } while (0)                                                               \

// must be the last line in each protothread
#define taskEnd()                                                           \
  schdImpl_StopTask(ctx, 0);                                                \
  FALLTHROUGH;                                                              \
  default:                                                                  \
      return;                                                               \
  }                                                                         \

#define taskSemaphoreInit(semPtr)               schdImpl_SemaphoreInit(semPtr)
#define taskSemaphoreSignal(semPtr)             schdImpl_SemaphoreSignal(semPtr)

#define taskTimerInit(timerPtr, period)         schdImpl_TimerInit(timerPtr, period)
#define taskTimerTrigger(timerPtr)              schdImpl_TimerTrigger(timerPtr)
#define taskTimerGetLastInterval(timerPtr)      schdImpl_TimerGetLastInterval(timerPtr)

#define taskCreate(fn, name, prio, param)       schdImpl_CreateTask(fn, name, prio, param)

#define taskCreateLegacyHz(fnName, name, prio, periodUs)      \
  static taskLegacyTCB_t legacyTCB_##fnName = {               \
    .delayUs = periodUs,                                      \
    .fn = fnName                                              \
  };                                                          \
  taskCreate(schdImpl_LegcyTaskFn, name, prio, &legacyTCB_##fnName);

#define isSystemOverloaded()                    (schedulerGetAverageSystemLoadPercent() >= 90)

/***************************/
void schdImpl_LegcyTaskFn(taskRuntimeContext_t * ctx, timeUs_t currentTimeUs, void * param);

void schdImpl_SemaphoreInit(schdSemaphore_t * semPtr);
void schdImpl_SemaphoreSignal(schdSemaphore_t * semPtr);
void schdImpl_WaitUntilSemaphore(taskRuntimeContext_t * ctx, schdSemaphore_t * semPtr, timeUs_t timeout);
void schdImpl_GetLastWaitUnblockReason(taskRuntimeContext_t * ctx, taskWaitUnblockReason_e * isTimeoutUnblock);

void schdImpl_TimerInit(schdTimer_t * timerPtr, timeUs_t period);
void schdImpl_TimerTrigger(schdTimer_t * timerPtr);
timeUs_t schdImpl_TimerGetLastInterval(schdTimer_t * timerPtr);
void schdImpl_WaitUntilTimer(taskRuntimeContext_t * ctx, schdTimer_t * timPtr);


int  schdImpl_CreateTask(taskFunctionPtr_t fn, const char * name, int priority, void * param);
void schdImpl_StopTask(taskRuntimeContext_t * ctx, int returnCode);

/***************************/
void schedulerInit(void);
void scheduler(void);
int  schedulerGetAverageSystemLoadPercent(void);
const taskInfo_t * schedulerGetTaskInfo(int pid);
