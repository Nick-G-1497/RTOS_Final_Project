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
 * OS level priority of the task dedicated to setting the speed set-point
 */
#define SPEED_SETPOINT_PRIORITY 18

/**
 * OS level priority of the task dedicated to monitoring the vehicle
 */
#define VEHICLE_MONITOR_PRIORITY 18

/**
 * OS level priority of the task dedicated to determining the vehicle direction
 */
#define VEHICLE_DIRECTION_PRIORITY 18


/**
 * OS level priority of the LED output task
 */
#define LED_OUTPUT_PRIORITY 18

/**
 * OS level priority of the LCD Display
 */
#define LCD_DISPLAY_PRIORITY 20

/**
 * OS level priority of the idle task
 */
#define IDLETASK_PRIORITY 25


/*******************************************************************************************************************
 * Custom Data Types
 ******************************************************************************************************************/
/**
 * @brief Speed Set Point
 */
struct SpeedSetPoint_t
{
  int current_speed;
  unsigned int speed_increments;
  unsigned int speed_decrements;
};

/**
 * @brief Vehicle Direction Enumeration
 */
enum vehicle_direction
{
  LEFT,     //!< LEFT
  HARD_LEFT,//!< HARD_LEFT
  RIGHT,    //!< RIGHT
  HARD_RIGHT,//!< HARD_RIGHT
  STRAIGHT
};

/**
 * @brief Vehicle Direction Structure
 */
struct VehicleDirection_t
{
  enum vehicle_direction direction;
  unsigned int time_current_direction_held_constant_ms;
  unsigned int num_left_turns;
  unsigned int num_right_turns;
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
static OS_SEM button_semaphore;

static OS_MUTEX speed_set_point_mux;

static OS_MUTEX direction_mux;

static OS_SEM slider_semaphore;

static OS_SEM monitor_semaphore;

///**
// * @brief Button zero semaphore
// */
//static OS_SEM button0_semaphore;
//
//
///**
// * @brief Button One semaphore
// */
//static OS_SEM button1_semaphore;

/******************************************************************************
 * Event Flag
 *****************************************************************************/
///**
// * @brief Button Event Flag Group
// */
//extern static OS_FLAG_GRP button_event_flag_group;

//void post_button0_event();
//void post_button1_event();
/******************************************************************************
 * OS Timer
 *****************************************************************************/
static OS_TMR slider_timer;

static OS_TMR direction_timer;

static OS_TMR monitor_timer;


/**
 * @brief Slider timer callback function
 * @param p_tmr
 * @param p_arg
 */
void steering_timer_callback_function(OS_TMR* p_tmr, void* p_arg);

/**
 * @Monitor Timer Callback Function
 * @param p_tmr
 * @param p_arg
 */
void monitor_timer_callback_function(OS_TMR* p_tmr, void* p_arg);

/**
 * @brief Turn timer callback function
 * @param p_tmr timer pointer
 * @param p_arg pointer arguement
 */
void turn_timer_callback_function(OS_TMR* p_tmr, void* p_arg);


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
