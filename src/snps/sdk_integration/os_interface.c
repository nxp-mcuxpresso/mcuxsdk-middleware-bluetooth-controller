/* Copyright 2021-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"
#include "os_wrapper.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "board.h"
#include "fsl_os_abstraction.h"
#include "bsp.h"

#ifdef MEM_PROFILING
#include "fsl_os_abstraction_mem_profiling_free_rtos.h"
#endif /* MEM_PROFILING */

/* Link layer uses os abstraction layer for OS specific calls. All these calls need to be stubbed here.
   By default Link layer ll has FreeRTOS implementations of these functions, but since these BLE application use an
   abstraction layer on top of FreeRTOS, the FreeRTOS implementation in the Link layer is not compiled in.
   Instead all these OS specific calls need to be stubbed here
*/
void os_start(void)
{
	/* Not needed, already done by main in OS abstraction layer*/
	vTaskStartScheduler();
}

/**
 * @brief FreeRTOS synchronisation functions.
 *
 * The FreeRTOS semaphore and mutex use the same base,
 */
os_thread_id os_thread_create(os_pthread thread,char* name,os_priority pri,void* argu,uint32_t stack_size)
{
	BaseType_t xReturn;
	TaskHandle_t xtaskHandle = NULL;
	xReturn = xTaskCreate((TaskFunction_t)thread,name, stack_size, argu, (configMAX_PRIORITIES - 1) - pri, &xtaskHandle);
	
	if(xReturn == pdPASS)
	{
		#ifdef MEM_PROFILING
		OSA_TaskTracking((osa_task_handle_t) xtaskHandle, (char const *) name, (uint16_t) stack_size);
		#endif /* MEM_PROFILING */
		return (os_thread_id)xtaskHandle;  
	}

	return NULL;
}
int32_t os_thread_terminate(os_thread_id task)
{
	vTaskDelete((TaskHandle_t)task);
	return 0;
}

void os_disable_isr(void)
{
    disable_irq();
}

void os_enable_isr(void)
{
    enable_irq();
}

/*  ==== Use FreeRTOS Mutex Functions ====  */

os_mutex_id os_rcrsv_mutex_create(void)
{
	return (xSemaphoreCreateRecursiveMutex());
}

int32_t os_rcrsv_mutex_wait(os_mutex_id mutex_id, uint32_t millisec)
{	

	return xSemaphoreTakeRecursive(mutex_id, millisec);
}

int32_t os_rcrsv_mutex_release(os_mutex_id mutex_id)
{

	return xSemaphoreGiveRecursive(mutex_id);
}

int32_t os_rcrsv_mutex_delete(os_mutex_id mutex_id)
{
	vSemaphoreDelete(mutex_id);
	return (0);

}

/*  ==== Use FreeRTOS Semaphore Functions ====  */

os_semaphore_id os_semaphore_create(int32_t max_count,
	int32_t initial_count)
{
	return (xSemaphoreCreateCounting(max_count, initial_count));

}

int32_t os_semaphore_wait(os_semaphore_id semaphore_id, uint32_t millisec)
{

	return (xSemaphoreTake(semaphore_id, millisec));

}

int32_t os_semaphore_wait_isr(os_semaphore_id semaphore_id, uint32_t prio)
{

	return (xSemaphoreTakeFromISR(semaphore_id,(void *) prio));
}

int32_t os_semaphore_release(os_semaphore_id semaphore_id)
{

	return (xSemaphoreGive(semaphore_id));

}

int32_t os_semaphore_release_isr(os_semaphore_id semaphore_id)
{
	BaseType_t ret;
	BaseType_t woken = pdFALSE;
	ret = xSemaphoreGiveFromISR(semaphore_id, &woken);
	portYIELD_FROM_ISR(woken);
	return (ret);
}

int32_t os_semaphore_delete(os_semaphore_id semaphore_id)
{
	vSemaphoreDelete(semaphore_id);
	return (0);
}
