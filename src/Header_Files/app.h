/***************************************************************************//**
 * @file app.h
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include "cpu.h"
#include "cmu.h"
#include "em_chip.h"
#include "os.h"
#include "bsp_os.h"
#include "config_v3.h"
#include "os_trace.h"
#include "em_emu.h"
#include "queue.h"
#include "pwm.h"



/********************************************************************************
 * Macro Expressions
 ********************************************************************************/

/**
 * Digital high : +5V
 */
#define HIGH 1

/**
 * Digital low : -5V
 */
#define LOW 0

/**
 * Task Stack Size for
 */
#define TASK_STK_SIZE 256

/**
 * OS level priority of the boost task
 */
#define BOOST_TASK_PRIORITY 18

/**
 * OS level priority of the laser task
 */
#define LASER_TASK_PRIORITY 18

/**
 * OS level priority of the task dedicated to determining the slider state
 */
#define SLIDER_STATE_TASK_PRIORITY 18

/**
 * OS level priority to the task updating the Harkonnen Mass Physics
 */
#define HM_PHYSICS_PRIORITY 18


/**
 * OS level priority to the task updating the shield position, velocity and
 * acceleration
 */
#define SHIELD_PHYSICS_PRIORITY 18


/**
 * OS level priority of the task dedicated to the desired shield force
 */
#define DESIRED_SHIELD_FORCE_PRIORITY 18

/**
 * OS level priority of the shield force task
 */
#define SHIELD_FORCE_TASK_PRIORITY 20

#define LCD_DISPLAY_PRIORITY 20

/**
 * OS level priority of the idle task
 */
#define IDLETASK_PRIORITY 25





/******************************************************************************
 * Button Event Enumerations
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void);
/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void);


/******************************************************************************
 * Message Queues
 *****************************************************************************/

/******************************************************************************
 * Semaphores
 *****************************************************************************/
static OS_SEM slider_semaphore;
static OS_SEM LCD_semaphore;
static OS_SEM shield_physics_semaphore;
static OS_SEM hm_physics_semaphore;
static OS_SEM laser_semaphore;
static OS_SEM boost_deactivate_semaphore;



/**
 * @brief Semaphore used with timer and touch slider
 */

static OS_MUTEX HM_mux;

static OS_MUTEX shield_mux;



/******************************************************************************
 * Event Flag
 *****************************************************************************/

/******************************************************************************
 * OS Timer
 *****************************************************************************/
//static OS_TMR PWM0_timer;
//static OS_TMR PWM1_timer;
static OS_TMR slider_timer;
static OS_TMR LCD_timer;
static OS_TMR boost_timer;


/**
 * @brief Initialize OS objects
 */
void os_object_init (void);

//
//void GPIO_EVEN_IRQHandler(void);
//void GPIO_ODD_IRQHandler(void);
/***************************************************************************//**
 * Initialize application.
 *
 ******************************************************************************/

/**
 * Initialize all application specific software
 */
void app_init(void);

/**
 * Initialize all the application's tasks
 */
void task_init(void);


void lcd_timer_callback_function(OS_TMR* p_tmr, void* p_arg);
void slider_state_timer_callback_function(OS_TMR* p_tmr, void* p_arg);
void shield_physics_timer_callback_function(OS_TMR* p_tmr, void* p_arg);
void boost_timer_callback_function(OS_TMR* p_tmr, void* p_arg);
void hm_physics_timer_callback_function(OS_TMR* p_tmr, void* p_arg);

#endif  // APP_H
