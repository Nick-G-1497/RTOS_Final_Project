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

/**
 * @brief Current Speed Set Point
 */
volatile struct SpeedSetPoint_t current_speed_set_point;

/**
 * @brief Current Vehicle Direction
 */
volatile struct VehicleDirection_t current_vehicle_direction;

/**
 * @brief Speed Update Flag
 */
const OS_FLAGS  SpeedUpdateFlag = 0x01;

/**
 * @brief Direction Update Flag
 */
const OS_FLAGS  DirectionUpdateFlag = 0x02;

const OS_FLAGS  BothUpdates = 0x03;

/**
 * @brief Flag indicating whether the current turn exceeds 5 s
 *
 */
const OS_FLAGS  TurnTooLongFlag = 0x04;

/**
 * @brief Speed Violation Event Flag
 */
const OS_FLAGS  SpeedViolation = 0x01;

/**
 * @brief Direction Violation Event Flag
 */
const OS_FLAGS  DirectionViolation = 0x02;


/**
 * @brief No Speed Violation Event
 */
const OS_FLAGS NoSpeedViolation = 0x04;


/**
 * @brief No Direction Violation Event
 */
const OS_FLAGS NoDirectionViolation = 0x08;

const OS_FLAGS All_LED_events = 0xf;



GLIB_Rectangle_t left_wall;
GLIB_Rectangle_t right_wall;

GLIB_Rectangle_t shield;


/*******************************************************************************
 * Task Control Blocks
 ******************************************************************************/

/**
 * LCD Display Task Control Block
 */
OS_TCB LCDDisplay_TCB;

/**
 * Speed Set-point Task Control Block
 */
OS_TCB SpeedSetpoint_TCB;

/**
 * LED Task Control Block
 */
OS_TCB LEDOutput_TCB;

/**
 * Vehicle Monitor Task Control Block
 */
OS_TCB VehicleMonitor_TCB;

/**
 * Vehicle Direction Task Control Block
 */
OS_TCB VehicleDirection_TCB;

/**
 * Idle Task Control Block
 */
OS_TCB IdleTask_TCB;

/******************************************************************************
 * Task Stacks
 ******************************************************************************/

/**
 * Button Task Stack of size TASK_STK_SIZE
 */
CPU_STK  LCDDisplay_TaskStack [TASK_STK_SIZE];

/**
 * Button Task Stack of size TASK_STK_SIZE
 */
CPU_STK  SpeedSetpoint_TaskStack [TASK_STK_SIZE];


/**
 * LED Task Stack of size TASK_STK_SIZE
 */
CPU_STK  LEDOutput_TaskStack [TASK_STK_SIZE];

/**
 * Vehicle Monitor Task Stack
 */
CPU_STK VehicleMonitor_TaskStack [TASK_STK_SIZE];

/**
 * Slider Task Stack of size TASK_STK_SIZE
 */
CPU_STK  VehicleDirection_TaskStack [TASK_STK_SIZE];

/**
 * Idle Task Stack of size TASK_STK_SIZE
 */
CPU_STK Idle_TaskStack[TASK_STK_SIZE];

/*******************************************************************************
 * Application Specific Variables
 *******************************************************************************/
static OS_FLAG_GRP LED_event_group;
static OS_FLAG_GRP VehicleMonitorEventGroup;

static GLIB_Context_t glibContext;
//static int currentLine = 0;


/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

/**
 * Task declaration function for task dedicated to setting the LED output
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void LED_Output(void* random_arguement_parameter);

/**
 * Task declaration function for task dedicated for taking input from
 * the user and determine vehicle direction
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void VehicleDirection(void* random_arguement_parameter);


/**
 * Task declaration function for task dedicated for polling the buttons
 * in order to determin a speed setpoint
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void SpeedSetpoint(void* random_arguement_parameter);

/**
 * Task declaration function for task dedicated for updating the LCD
 * Display
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void LCD_Display(void* random_arguement_parameter);

/**
 * Task declaration function for task dedicated for monitoring the
 * vehicle
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
static void VehicleMonitor(void* random_arguement_parameter);


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
  // Post to the Semaphore
  OSSemPost(&button_semaphore, OS_OPT_POST_ALL, &err);

  // Push a button event into the queue
  push(&fifo, button_0_press);

  GPIO_IntClear(1 << BUTTON0_pin);

}

/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void){
//TODO///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  RTOS_ERR err;

  // Post to the Semaphore
  OSSemPost(&button_semaphore, OS_OPT_POST_ALL, &err);

  // Push a button event into the queue
  push(&fifo, button_1_press);

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

//    // Create the LED Output Task
//    OSTaskCreate(&LEDOutput_TCB,
//                 "led task",
//                 LED_Output,
//                 DEF_NULL,
//                 LED_OUTPUT_PRIORITY,
//                 &LEDOutput_TaskStack[0],
//                 (TASK_STK_SIZE / 10u),
//                 TASK_STK_SIZE,
//                 0u,
//                 0u,
//                 DEF_NULL,
//                 (OS_OPT_TASK_STK_CLR),
//                 &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//
//    OSTaskCreate(&VehicleDirection_TCB,
//                 "Vehicle Direction",
//                 VehicleDirection,
//                 DEF_NULL,
//                 VEHICLE_DIRECTION_PRIORITY,
//                 &VehicleDirection_TaskStack[0],
//                 (TASK_STK_SIZE / 10u),
//                 TASK_STK_SIZE,
//                 0u,
//                 0u,
//                 DEF_NULL,
//                 (OS_OPT_TASK_STK_CLR),
//                 &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSTaskCreate(&SpeedSetpoint_TCB,
//                  "Speed Set-point",
//                  SpeedSetpoint,
//                  DEF_NULL,
//                  SPEED_SETPOINT_PRIORITY,
//                  &SpeedSetpoint_TaskStack[0],
//                  (TASK_STK_SIZE / 10u),
//                  TASK_STK_SIZE,
//                  0u,
//                  0u,
//                  DEF_NULL,
//                  (OS_OPT_TASK_STK_CLR),
//                  &err);
//     EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


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

//    OSTaskCreate(&VehicleMonitor_TCB, /* Create the start task */
//      "Vehicle Monitor",
//      VehicleMonitor,
//      DEF_NULL,
//      VEHICLE_MONITOR_PRIORITY,
//      &VehicleMonitor_TaskStack[0],
//      (TASK_STK_SIZE / 10),
//      TASK_STK_SIZE,
//      0u,
//      0u,
//      DEF_NULL,
//      (OS_OPT_TASK_STK_CLR),
//      &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSTaskCreate(&IdleTask_TCB, /* Create the start task */
//      "idle task",
//      Idle,
//      DEF_NULL,
//      25,
//      &Idle_TaskStack[0],
//      (TASK_STK_SIZE / 10),
//      TASK_STK_SIZE,
//      0u,
//      0u,
//      DEF_NULL,
//      (OS_OPT_TASK_STK_CLR),
//      &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}


void os_object_init (void)
{
  // Create OS Level Message Queue
  RTOS_ERR err;
//  OSQCreate (&LED_Output_MSG_Q,
//                   "LED Output Message Queue",
//                   25,
//                   &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  // Create the Semaphore data-structure
  OSSemCreate (&button_semaphore,
                     "button semaphore",
                     0,
                     &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSSemCreate (&slider_semaphore,
                     "slider semaphore",
                     0,
                     &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSSemCreate (&monitor_semaphore,
               "monitor semaphore",
               0,
               &err);


  // Create the mutex for the speed setpoint
  OSMutexCreate (&speed_set_point_mux,
                       "Speed Setpoint Mux",
                       &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  // Create the mutex for the direction variable
  OSMutexCreate (&direction_mux,
                         "Direction Mux",
                         &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


  OSFlagCreate(&LED_event_group,
               "LED Event Flag Group",
               0x0,
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


  OSFlagCreate(&VehicleMonitorEventGroup,
               "LED Event Flag Group",
               0x0,
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


 OSTmrCreate(&slider_timer,
                    "Slider Timer",
                    2,
                    2,
                    OS_OPT_TMR_PERIODIC,
                    steering_timer_callback_function,
                    (void*) 0,
                    &err);
 EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

 OSTmrCreate(&direction_timer,
                    "Turn Timer",
                    50,
                    2,
                    OS_OPT_TMR_ONE_SHOT,
                    turn_timer_callback_function,
                    (void*) 0,
                    &err);
 EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

 OSTmrCreate(&monitor_timer,
                     "Monitor Timer",
                     2,
                     2,
                     OS_OPT_TMR_PERIODIC,
                     monitor_timer_callback_function,
                     (void*) 0,
                     &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));





}


void app_init(void)
{
  gpio_open();
  // blink_init();
  // task_init();

}

static void LED_Output(void* random_arguement_parameter)
{
  RTOS_ERR err;
  PP_UNUSED_PARAM(random_arguement_parameter);
  while(1){
      OS_FLAGS who_dun_it = OSFlagPend(&LED_event_group,
                All_LED_events,
                100,
                OS_OPT_PEND_FLAG_SET_ANY  + OS_OPT_PEND_BLOCKING + OS_OPT_PEND_FLAG_CONSUME,
                NULL,
                &err
                );
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)){


        if (who_dun_it & DirectionViolation)
        {
          // Light up LED1

          digitalWrite(LED1_port, LED1_pin, HIGH);

//          OSFlagPost (&LED_event_group,
//                      DirectionViolation,
//                      OS_OPT_POST_FLAG_CLR,
//                      &err);
        }
        if (who_dun_it & SpeedViolation)
        {
          // Light up LED0
          digitalWrite(LED0_port, LED0_pin, 1);

//          OSFlagPost (&LED_event_group,
//                      SpeedViolation,
//                      OS_OPT_POST_FLAG_CLR,
//                      &err);
        }
        if (who_dun_it & NoSpeedViolation)
        {
          // Light up LED0
          digitalWrite(LED0_port, LED0_pin, 0);

//          OSFlagPost (&LED_event_group,
//                      NoSpeedViolation,
//                      OS_OPT_POST_FLAG_CLR,
//                      &err);
        }
        if (who_dun_it & NoDirectionViolation)
        {
          // Light up LED0
          digitalWrite(LED1_port, LED1_pin, 0);

//          OSFlagPost (&LED_event_group,
//                      NoDirectionViolation,
//                      OS_OPT_POST_FLAG_CLR,
//                      &err);
        }
      }
  }
}


static void VehicleDirection(void* random_arguement_parameter)
{
  PP_UNUSED_PARAM(random_arguement_parameter);

  bool channel_values [4];
  RTOS_ERR err;

  OSTmrStart(&slider_timer, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  CAPSENSE_Init();

  while(1){

      // Add kernel call to awake the task periodically
      OSSemPend(&slider_semaphore,
                      100,
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
      {

        // Add call to read the slider position
        CAPSENSE_Sense();

        for (int i = 0; i < 4; i++)
          {
            channel_values[i] = 0;
            channel_values[i] = CAPSENSE_getPressed(i);
          }
        OSMutexPend (&direction_mux,
                                            100,
                                            OS_OPT_PEND_BLOCKING,
                                            NULL,
                                            &err);
          if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
            {
              OSFlagPend (&VehicleMonitorEventGroup,
                                                  TurnTooLongFlag,
                                                  0,
                                                  OS_OPT_PEND_FLAG_SET_ALL +  OS_OPT_PEND_FLAG_CONSUME + OS_OPT_PEND_NON_BLOCKING,
                                                  NULL,
                                                  &err);
              if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
              {
                current_vehicle_direction.time_current_direction_held_constant_ms = 51;
                OSFlagPost (&VehicleMonitorEventGroup,
                                        DirectionUpdateFlag,
                                        OS_OPT_POST_FLAG_SET,
                                        &err);
              }

              enum vehicle_direction prev_direction = current_vehicle_direction.direction;

              current_vehicle_direction.direction = STRAIGHT;

              if (channel_values[2])
              {
                current_vehicle_direction.direction = RIGHT;
              }
              if (channel_values[1])
              {
                current_vehicle_direction.direction = LEFT;
              }
              if (channel_values[0])
              {
                current_vehicle_direction.direction = HARD_LEFT;
              }
              if (channel_values[3])
              {
                current_vehicle_direction.direction = HARD_RIGHT;
              }


              // if (current_vehicle_direction.direction == prev_direction)
              // {
              //   current_vehicle_direction.time_current_direction_held_constant_ms = OSTmr
              // }
              // Check if the vehicle direction changed
              if (current_vehicle_direction.direction != prev_direction)
              {

                // How do I deal with the amount of time that the vehicle is going its current direction?
                OSTmrStop (&direction_timer,
                        OS_OPT_TMR_NONE,
                        turn_timer_callback_function,
                        &err);

                current_vehicle_direction.time_current_direction_held_constant_ms = 0;
                // Start the OS timer
                OSTmrStart(&direction_timer,
                            &err);
                // Increment the number of turns variables
                if ( (current_vehicle_direction.direction == LEFT  || current_vehicle_direction.direction == HARD_LEFT) && (prev_direction != LEFT || prev_direction != HARD_LEFT) )
                {
                  current_vehicle_direction.num_left_turns ++;
                }
                if ( (current_vehicle_direction.direction == RIGHT || current_vehicle_direction.direction == HARD_RIGHT) && (prev_direction != RIGHT || prev_direction != HARD_RIGHT) )
                {
                  current_vehicle_direction.num_right_turns ++;
                }




                // Update the OS Flag Group if the vehicle direction has changed
                OSFlagPost (&VehicleMonitorEventGroup,
                        DirectionUpdateFlag,
                        OS_OPT_POST_FLAG_SET,
                        &err);
              }


              // release the mutex
              OSMutexPost(&direction_mux,
                    OS_OPT_POST_NONE,
                    &err);



            }

//          OSTimeDly(1, OS_OPT_TIME_DLY, &err);
      }




  }
}

static void SpeedSetpoint(void* random_arguement_parameter)
{
  RTOS_ERR err;
  PP_UNUSED_PARAM(random_arguement_parameter);
  while(1){

            // Pend on the semaphore
            OSSemPend(&button_semaphore,
                      100,
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
            // if the semaphore pend didn't time out
            if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
              {
                  // peak the event from the fifo
                  struct event_t* current_event = peek(&fifo);


                  // try to take a mutex
                  OSMutexPend (&speed_set_point_mux,
                                     100,
                                     OS_OPT_PEND_BLOCKING,
                                     NULL,
                                     &err);
                  if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
                    {



                      // update the speed setpoint variable
                      switch (current_event->button_event)
                      {
                        case (button_0_press):
                            current_speed_set_point.current_speed += 5;
                            current_speed_set_point.speed_increments ++;
                            break;
                        case (button_1_press):
                            current_speed_set_point.current_speed -= 5;
                            current_speed_set_point.speed_decrements ++;
                            break;
                      }


                      // release the mux
                      OSMutexPost(&speed_set_point_mux,
                                  OS_OPT_POST_NONE,
                                  &err);
                    }


                  // Add a post to a event flag
                  OSFlagPost (&VehicleMonitorEventGroup,
                    SpeedUpdateFlag,
                    OS_OPT_POST_FLAG_SET,
                    &err);

                  pop(&fifo);

                  // Delay
//
              }
  }
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

static void VehicleMonitor(void* random_arguement_parameter)
{
  RTOS_ERR err;
  PP_UNUSED_PARAM(random_arguement_parameter);

  struct SpeedSetPoint_t current_speed_set_point_local = current_speed_set_point;
  struct VehicleDirection_t current_vehicle_direction_local = current_vehicle_direction;


  while(1){



    // pend on the event flag designated for the vehicle monitor task
    OSFlagPend(&VehicleMonitorEventGroup,
          BothUpdates,
          100,
          OS_OPT_PEND_FLAG_SET_ANY  + OS_OPT_PEND_BLOCKING + OS_OPT_PEND_FLAG_CONSUME,
          NULL,
          &err
          );
    if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)
    {
      // OS_FLAGS who_dun_it = OSFlagPendGetFlagsRdy (&err);
      // check the speed update flag

      OSMutexPend (&speed_set_point_mux,
                                  100,
                                  OS_OPT_PEND_BLOCKING,
                                  NULL,
                                  &err);
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
        {
          current_speed_set_point_local = current_speed_set_point;
          OSMutexPost(&speed_set_point_mux,
                                                OS_OPT_POST_NONE,
                                                &err);
        }
      OSMutexPend (&direction_mux,
                             100,
                             OS_OPT_PEND_BLOCKING,
                             NULL,
                             &err);
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
      {
          current_vehicle_direction_local = current_vehicle_direction;
          OSMutexPost(&direction_mux,
                      OS_OPT_POST_NONE,
                      &err);
      }


        if ( abs(current_speed_set_point_local.current_speed) >= 75)
        {
            // Post a speed violation
            OSFlagPost (&LED_event_group,
                            SpeedViolation,
                            OS_OPT_POST_FLAG_SET,
                            &err);
        }
        else if ( (abs(current_speed_set_point_local.current_speed) >= 45) && current_vehicle_direction_local.direction != STRAIGHT)
        {
            // Post a speed violation
            OSFlagPost (&LED_event_group,
                            SpeedViolation,
                            OS_OPT_POST_FLAG_SET,
                            &err);
        }
        else
        {
            OSFlagPost(&LED_event_group,
                        NoSpeedViolation,
                        OS_OPT_POST_FLAG_SET,
                        &err);
        }

        // clear the flag
//        OSFlagPost (&VehicleMonitorEventGroup,
//                      SpeedUpdateFlag,
//                      OS_OPT_POST_FLAG_CLR,
//                      &err);


        if (current_vehicle_direction_local.time_current_direction_held_constant_ms >= 50 && (current_vehicle_direction_local.direction != STRAIGHT))
        {
          OSFlagPost (&LED_event_group,
                            DirectionViolation,
                            OS_OPT_POST_FLAG_SET,
                            &err);
        }
        else
        {
          OSFlagPost(&LED_event_group,
                      NoDirectionViolation,
                      OS_OPT_POST_FLAG_SET,
                      &err);
        }
        // clear the flag
        OSFlagPost (&VehicleMonitorEventGroup,
                        DirectionUpdateFlag,
                        OS_OPT_POST_FLAG_CLR,
                        &err);

    }
  }
}

void steering_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
{
    // make a post to the semaphore
    PP_UNUSED_PARAM(p_arg);
    PP_UNUSED_PARAM(p_tmr);
    RTOS_ERR err;
    OSSemPost(&slider_semaphore, OS_OPT_POST_ALL, &err);
}

void monitor_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
{
    // make a post to the semaphore
    PP_UNUSED_PARAM(p_arg);
    PP_UNUSED_PARAM(p_tmr);
    RTOS_ERR err;
    OSSemPost(&monitor_semaphore, OS_OPT_POST_ALL, &err);
}


void turn_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
{
    RTOS_ERR err;
    // set that the flag has taken too long
    OSFlagPost (&VehicleMonitorEventGroup,
                    TurnTooLongFlag,
                    OS_OPT_POST_FLAG_SET,
                    &err);
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
//
//
//void post_button0_event()
//{
//  RTOS_ERR err;
//    // Post to
//    OSFlagPost (&button_event_flag_group,
//      BUTTON0_Event,
//      OS_OPT_POST_FLAG_SET,
//      &err);
//}

