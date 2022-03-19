/***************************************************************************//**
 * @file
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
#include "blink.h"
#include "app.h"
#include "gpio.h"
#include "os.h"
#include "capsense.h"
#include "queue.h"
#include "glib.h"
#include "dmd.h"
#include "os.h"
#include "pwm.h"


#define BALL_RADIUS 5


/**
 * @brief Flags for button 0 press
 *
 */
const uint8_t button_0_press = 0x00;

/**
 * @brief Flags for button 1 press
 *
 */
const uint8_t button_1_press = 0x01;


/**
 * @brief Queue defined to track button events
 */
struct node_t* fifo;



pwm_paramaters_t PWM0 = {
  .frequency_HZ = 50, // frequency
  .duty_cycle_percent = 25, // duty cycle
  // .timer = &PWM0_timer, // timer
  .timer_name = "PWM0 Timer", // name
  .port = gpioPortF, // port
  .pin = 4u, // pin
  .counter = 0 // counter
};



GLIB_Rectangle_t left_wall;
GLIB_Rectangle_t right_wall;

GLIB_Rectangle_t shield;


/*******************************************************************************
 * Task Control Blocks
 ******************************************************************************/

/**
 * LCD Display Task Control Block
 */
OS_TCB BoostTask_TCB;

/**
 * Speed Set-point Task Control Block
 */
OS_TCB LaserTask_TCB;

/**
 * LED Task Control Block
 */
OS_TCB SliderState_TCB;

/**
 * Vehicle Monitor Task Control Block
 */
OS_TCB DesiredShieldForce_TCB;

/**
 * Vehicle Direction Task Control Block
 */
OS_TCB ShieldForce_TCB;

/**
 * Idle Task Control Block
 */
OS_TCB IdleTask_TCB;

OS_TCB LCDDisplay_TCB;

/******************************************************************************
 * Task Stacks
 ******************************************************************************/

/**
 * Button Task Stack of size TASK_STK_SIZE
 */
CPU_STK  LCDDisplay_TaskStack [TASK_STK_SIZE];

CPU_STK  Boost_TaskStack [TASK_STK_SIZE];

/**
 * Button Task Stack of size TASK_STK_SIZE
 */
CPU_STK  Laser_TaskStack [TASK_STK_SIZE];


/**
 * LED Task Stack of size TASK_STK_SIZE
 */
CPU_STK  SliderState_TaskStack [TASK_STK_SIZE];

/**
 * Vehicle Monitor Task Stack
 */
CPU_STK DesiredShieldForce_TaskStack [TASK_STK_SIZE];

/**
 * Slider Task Stack of size TASK_STK_SIZE
 */
CPU_STK  ShieldForce_TaskStack [TASK_STK_SIZE];

/**
 * Idle Task Stack of size TASK_STK_SIZE
 */
CPU_STK Idle_TaskStack[TASK_STK_SIZE];

/*******************************************************************************
 * Application Specific Variables
 *******************************************************************************/


static GLIB_Context_t glibContext;


/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

/**
 * Task declaration function for task dedicated to setting up the boost button
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void BoostTask(void* random_arguement_parameter);

/**
 * Task declaration function for task dedicated for taking the laser button
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void LaserTask(void* random_arguement_parameter);


/**
 * Task declaration function for task dedicated to getting the state of the slider
 * for the shield
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void SliderStateTask(void* random_arguement_parameter);

/**
 * Task declaration function for task dedicated for updating the LCD
 * Display
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void LCD_Display(void* random_arguement_parameter);

/**
 * Task declaration function for task dedicated for predictive analytics for the
 * desired force needed for the shield to meet the mass
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void DesiredShieldForceTask(void* random_arguement_parameter);


/**
 * Task declaration function for lower priority task intended to save
 * energy by putting the micro-controller to sleep
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void Idle(void* random_arguement_parameter);



/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void){
  RTOS_ERR err;


  GPIO_IntClear(1 << BUTTON0_pin);

}

/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void){
//TODO///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  RTOS_ERR err;



  GPIO_IntClear(1 << BUTTON1_pin);

}



/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/

/**
 * Initialize all tasks
 */
void task_init ()
{

    RTOS_ERR err;

    OSTaskCreate(&LCDDisplay_TCB, /* Create the start task */
      "LCD Display",
      LCD_Display,
      DEF_NULL,
      LCD_DISPLAY_PRIORITY,
      &LCDDisplay_TaskStack[0],
      (TASK_STK_SIZE / 10),
      TASK_STK_SIZE,
      0u,
      0u,
      DEF_NULL,
      (OS_OPT_TASK_STK_CLR),
      &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSTaskCreate(&DesiredShieldForce_TCB, /* Create the start task */
          "Desired Force Task",
          DesiredShieldForceTask,
          DEF_NULL,
          DESIRED_SHIELD_FORCE_PRIORITY,
          &DesiredShieldForce_TaskStack[0],
          (TASK_STK_SIZE / 10),
          TASK_STK_SIZE,
          0u,
          0u,
          DEF_NULL,
          (OS_OPT_TASK_STK_CLR),
          &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}


void os_object_init (void)
{
  // Create OS Level Message Queue
  RTOS_ERR err;



  // Create the mutex for the speed setpoint
  OSMutexCreate (&HM_mux,
                       "Harkonnen Mass Mux",
                       &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  // Create the mutex for the direction variable
  OSMutexCreate (&shield_mux,
                         "Shield Mux",
                         &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


}


void app_init(void)
{
  gpio_open();
  // blink_init();
  // task_init();

}







static void LCD_Display(void* random_arguement_parameter)
{
  PP_UNUSED_PARAM(random_arguement_parameter);

    uint32_t status;
    RTOS_ERR err;
    /* Enable the memory lcd */
    status = sl_board_enable_display();
    EFM_ASSERT(status == SL_STATUS_OK);

    /* Initialize the DMD support for memory lcd display */
    status = DMD_init(0);
    EFM_ASSERT(status == DMD_OK);

    /* Initialize the glib context */
    status = GLIB_contextInit(&glibContext);
    EFM_ASSERT(status == GLIB_OK);

    glibContext.backgroundColor = White;
    glibContext.foregroundColor = Black;


    // Define the Left Wall
    left_wall.xMin = 1;
    left_wall.xMax = 5;
    left_wall.yMin = 0;
    left_wall.yMax = 127;

    // Define the Right Wall
    right_wall.xMin = 121;
    right_wall.xMax = 126;
    right_wall.yMin = 0;
    right_wall.yMax = 127;

    shield.xMin = 75;
    shield.xMax = 100;
    shield.yMin = 120;
    shield.yMax = 127;


    int32_t ball_x = 65;
    int32_t ball_y = 65;

    uint32_t numPoints = 6;
    int32_t ball_polyPoints[12] = {ball_x, ball_y + 6,
                                ball_x + 5, ball_y + 3,
                                ball_x + 5, ball_y - 3,
                                ball_x , ball_y - 6,
                                ball_x - 5, ball_y - 3,
                                ball_x - 5, ball_y + 3};

  while(1){

      ball_y = (ball_y + 1) % 127;


      // Update the polygon points of the
      ball_polyPoints[0] = ball_x;
      ball_polyPoints[1] = ball_y + 6;
      ball_polyPoints[2] = ball_x + 5;
      ball_polyPoints[3] = ball_y + 3;
      ball_polyPoints[4] = ball_x + 5;
      ball_polyPoints[5] = ball_y - 3;
      ball_polyPoints[6] = ball_x;
      ball_polyPoints[7] = ball_y - 6;
      ball_polyPoints[8] = ball_x - 5;
      ball_polyPoints[9] = ball_y - 3;
      ball_polyPoints[10] = ball_x - 5;
      ball_polyPoints[11] = ball_y + 3;

      /* Fill lcd with background color */
      GLIB_clear(&glibContext);


      GLIB_drawRectFilled   (   &glibContext,
          &left_wall
        );

      GLIB_drawRectFilled   (   &glibContext,
              &right_wall
            );


      GLIB_drawRectFilled   (   &glibContext,
                  &shield
                );


      GLIB_drawPolygonFilled  (   &glibContext,
          numPoints,
          ball_polyPoints
        );
      DMD_updateDisplay();


//      OSTimeDly(1, OS_OPT_TIME_DLY, &err);


  }
}

static void DesiredShieldForceTask(void* random_arguement_parameter)
{
  RTOS_ERR err;
  PP_UNUSED_PARAM(random_arguement_parameter);

  pwm_init (&PWM0_timer, &PWM0);
  pwm_start(&PWM0_timer, &PWM0);

  while (1)
    {

    }

}

//
static void Idle(void* random_arguement_parameter)
{
  RTOS_ERR err;

  PP_UNUSED_PARAM(random_arguement_parameter);

  while (1)
    {
      EMU_EnterEM1();
      OSTimeDly(1, OS_OPT_TIME_DLY, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
    }

}
