/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2019-2020 INAVFLIGHT OU
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "scheduler.h"
#include "drivers/light_led.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "common/maths.h"
#include "common/utils.h"

static taskTCB_t tasks[MAX_TASKS];
static timeUs_t  totalTaskRunTimeUs;
static timeUs_t  totalSchedulerRunTimeUs;
static int       avgSchedulerSystemLoad;

TASK(taskSystem)
{
    taskBegin();

    while(1) {
        // < 10Hz
        taskDelayMs(200);

        // Calculate averate CPU load
        avgSchedulerSystemLoad = 100 * totalTaskRunTimeUs / totalSchedulerRunTimeUs;
        totalSchedulerRunTimeUs = 0;
        totalTaskRunTimeUs = 0;
    }

    taskEnd();
}

TASK(schdImpl_LegcyTaskFn)
{
    taskBegin();
    taskTimerInit(&((taskLegacyTCB_t *)param)->timer, ((taskLegacyTCB_t *)param)->delayUs);

    while(1) {
        taskWaitTimer(&((taskLegacyTCB_t *)param)->timer);
        ((taskLegacyTCB_t *)param)->fn(currentTimeUs);
    }

    taskEnd();
}

void schdImpl_SemaphoreInit(schdSemaphore_t * semPtr)
{
    semPtr->state = false;
}

void schdImpl_SemaphoreReset(schdSemaphore_t * semPtr)
{
    semPtr->state = false;
}

void schdImpl_SemaphoreSignal(schdSemaphore_t * semPtr)
{
    semPtr->state = true;
}

taskTCB_t * schdImpl_FindFreeTCB(void)
{
    for (int idx = 0; idx < MAX_TASKS; idx++) {
        if (!tasks[idx].allocated) {
            memset(&tasks[idx], 0, sizeof(tasks[idx]));
            tasks[idx].index = idx;

            return &tasks[idx];
        }
    }

    return NULL;
}

static void schdImpl_ResetSchedulingPriority(taskTCB_t * tcb)
{
    tcb->taskSchedulerPrio = tcb->taskPrio;
}

static void schdImpl_InitTCB(taskTCB_t * tcb, taskFunctionPtr_t fn, const char * name, int priority, void * param)
{
    // Initialize the TCB
    tcb->allocated = true;
    tcb->taskName = name;
    tcb->taskFn = fn;
    tcb->taskParam = param;
    tcb->taskPrio = priority;

    schdImpl_ResetSchedulingPriority(tcb);

    // Initialize the protothread state
    tcb->ctx.state = TASK_STATE_IDLE;
    tcb->ctx.line = 0;

    // Initialize task accounting
    tcb->accnt.totalRunTimeUs = 0;
}

int schdImpl_CreateTask(taskFunctionPtr_t fn, const char * name, int priority, void * param)
{
    taskTCB_t * tcb = schdImpl_FindFreeTCB();
    if (!tcb) {
        return -1;
    }

    // Init TCB
    schdImpl_InitTCB(tcb, fn, name, priority, param);

    // Schedule the task to run
    tcb->ctx.state = TASK_STATE_RUNNING;

    return tcb->index;
}

void schdImpl_WaitUntilSemaphore(taskRuntimeContext_t * ctx, schdSemaphore_t * semPtr, timeUs_t timeout)
{
    // Create waiting context
    ctx->wait.objPtr = semPtr;
    ctx->wait.startTimeUs = micros();
    ctx->wait.timeoutUs = timeout;
    ctx->wait.unblockReason = TASK_WAIT_REASON_UNKNOWN;

    // Move task to waiting/semaphore state
    ctx->state = TASK_STATE_WAITING_SEMAPHORE;
}

void schdImpl_GetLastWaitUnblockReason(taskRuntimeContext_t * ctx, taskWaitUnblockReason_e * isTimeoutUnblock)
{
    if (isTimeoutUnblock) {
        *isTimeoutUnblock = ctx->wait.unblockReason;
    }
}

void schdImpl_WaitUntilTimer(taskRuntimeContext_t * ctx, schdTimer_t * timPtr)
{
    // Create waiting context
    ctx->wait.objPtr = timPtr;
    ctx->wait.unblockReason = TASK_WAIT_REASON_UNKNOWN;

    // Move task to waiting/timer state
    ctx->state = TASK_STATE_WAITING_TIMER;
}

void schdImpl_StopTask(taskRuntimeContext_t * ctx, int returnCode)
{
    ctx->state = TASK_STATE_ZOMBIE;
}

static timeUs_t FAST_CODE NOINLINE schdImpl_RunTask(taskTCB_t * tcb)
{
    if (tcb->taskFn) {
        timeUs_t taskExecutionStart = micros();

        // Run task
        tcb->taskFn(&tcb->ctx, taskExecutionStart, tcb->taskParam);

        // Bookkeeping
        timeUs_t taskExecutionTime = micros() - taskExecutionStart;
        tcb->accnt.totalRunTimeUs += taskExecutionTime;

        // Re-schedule next execution
        schdImpl_ResetSchedulingPriority(tcb);

        return taskExecutionTime;
    }
    else {
        return 0;
    }
}

static bool FAST_CODE NOINLINE schdImpl_CheckWaitTimeout(taskWaitCtx_t * wait)
{
    // Check for timeout
    if ((micros() - wait->startTimeUs) >= wait->timeoutUs) {
        wait->unblockReason = TASK_WAIT_REASON_TIMEOUT;
        return true;
    }

    return false;
}

static bool FAST_CODE NOINLINE schdImpl_CheckWaitSemaphore(taskWaitCtx_t * wait)
{
    // Treat objPtr as a pointer to semaphore
    if (wait->objPtr) {
        schdSemaphore_t * semaphore = (schdSemaphore_t *)wait->objPtr;
        if (semaphore->state) {
            schdImpl_SemaphoreReset(semaphore);
            wait->unblockReason = TASK_WAIT_REASON_EVENT;
            return true;
        }
    }

    return false;
}

void schdImpl_TimerInit(schdTimer_t * timerPtr, timeUs_t period)
{
    timerPtr->lastScheduledEventUs = micros();
    timerPtr->lastActualEventUs = timerPtr->lastScheduledEventUs;
    timerPtr->lastEventInterval = 0;
    timerPtr->periodUs = period;
}

void schdImpl_TimerTrigger(schdTimer_t * timerPtr)
{
    timeUs_t currentTimeUs = micros();
    timerPtr->lastEventInterval = currentTimeUs - timerPtr->lastActualEventUs;
    timerPtr->lastActualEventUs = currentTimeUs;
}

timeUs_t schdImpl_TimerGetLastInterval(schdTimer_t * timerPtr)
{
    return timerPtr->lastEventInterval;
}

static bool FAST_CODE NOINLINE schdImpl_CheckWaitTimer(taskWaitCtx_t * wait)
{
    // Treat objPtr as a pointer to semaphore
    if (wait->objPtr) {
        schdTimer_t * timerPtr = (schdTimer_t *)wait->objPtr;
        timeUs_t currentTimeUs = micros();

        if ((currentTimeUs - timerPtr->lastScheduledEventUs) >= timerPtr->periodUs) {
            // Store last actual interval
            schdImpl_TimerTrigger(timerPtr);

            // Advance timer to make sure we don't fire again immediately if we missed one event
            while ((currentTimeUs - timerPtr->lastScheduledEventUs) >= timerPtr->periodUs) {
                timerPtr->lastScheduledEventUs += timerPtr->periodUs;
            }

            wait->unblockReason = TASK_WAIT_REASON_EVENT;
            return true;
        }
    }

    return false;
}

static timeUs_t FAST_CODE NOINLINE schdImpl_CheckRunTask(taskTCB_t * tcb)
{
    if (tcb->taskSchedulerPrio == 0) {
        return schdImpl_RunTask(tcb);
    }
    else {
        return 0;
    }
}

static timeUs_t FAST_CODE NOINLINE schdImpl_ProcessTaskState(taskTCB_t * tcb)
{
    // Account for priority
    if (tcb->taskSchedulerPrio > 0) {
        tcb->taskSchedulerPrio--;
    }

    // Act based on task state
    switch (tcb->ctx.state) {
        case TASK_STATE_IDLE:
            // Idle task is not running
            return 0;

        case TASK_STATE_RUNNING:
            return schdImpl_CheckRunTask(tcb);
            
        case TASK_STATE_WAITING_SEMAPHORE:
            if (schdImpl_CheckWaitTimeout(&tcb->ctx.wait) || schdImpl_CheckWaitSemaphore(&tcb->ctx.wait)) {
                tcb->ctx.state = TASK_STATE_RUNNING;
                return schdImpl_CheckRunTask(tcb);
            }
            break;

        case TASK_STATE_WAITING_TIMER:
            if (schdImpl_CheckWaitTimer(&tcb->ctx.wait)) {
                tcb->ctx.state = TASK_STATE_RUNNING;
                return schdImpl_CheckRunTask(tcb);
            }
            break;

        case TASK_STATE_ZOMBIE:
            // Zombie task is not running
            return 0;
    }

    return 0;
}

void FAST_CODE NOINLINE scheduler(void)
{
    timeUs_t schedulerCycleStart = micros();
    timeUs_t taskExecutionTimeUs = 0;

    for (int idx = 0; idx < MAX_TASKS; idx++) {
        if (tasks[idx].allocated) {
            taskExecutionTimeUs += schdImpl_ProcessTaskState(&tasks[idx]);
        }
    }

    timeUs_t schedulerCycleTime = micros() - schedulerCycleStart;

    // Accumulate statistics
    totalTaskRunTimeUs += taskExecutionTimeUs;
    totalSchedulerRunTimeUs += schedulerCycleTime;

    // Bill scheduler time to system task
    //tasks[TASKID_SYSTEM].accnt.totalRunTimeUs += (schedulerCycleTime - taskExecutionTimeUs);
}

void schedulerInit(void)
{
    memset(tasks, 0, sizeof(tasks));

    for (int idx = 0; idx < MAX_TASKS; idx++) {
        tasks[idx].allocated = false;
    }

    // Create a system task [0]
    taskTCB_t * tcb = &tasks[TASKID_SYSTEM];
    schdImpl_InitTCB(tcb, taskSystem, "SYSTEM", TASK_PRIORITY_HIGH, NULL);
    tcb->ctx.state = TASK_STATE_RUNNING;
}

int schedulerGetAverageSystemLoadPercent(void)
{
    return avgSchedulerSystemLoad;
}

const taskInfo_t * schedulerGetTaskInfo(int pid)
{
    static taskInfo_t taskInfo;

    if (tasks[pid].allocated) {
        taskInfo.taskName           = tasks[pid].taskName;
        taskInfo.taskState          = tasks[pid].ctx.state;
        taskInfo.taskPriority       = tasks[pid].taskPrio;
        taskInfo.totalExecutionTime = tasks[pid].accnt.totalRunTimeUs;
        return &taskInfo;
    }

    return NULL;
}
