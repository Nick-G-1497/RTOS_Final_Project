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


/*******************************************************************************************************************
 * Custom Data Types
 ******************************************************************************************************************/
/**
 * @brief Harkonnen Mass Position type
 */
typedef struct Harkonnen_Mass_Position_t
{
  int x;
  int y;
  float velocity_x;
  float velocity_y;
};


/**
 * @brief Vehicle Direction Structure
 */
typedef struct ShieldPosition_t
{
  int x;
  float velocity_x;
  float acceleration_x;
  bool isBoosted;
};




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
static OS_TMR PWM0_timer;
static OS_TMR PWM1_timer;


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


#endif  // APP_H
