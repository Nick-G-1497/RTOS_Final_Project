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
#include "hm_physics.h"
#include "shield_physics.h"

#define PI 3.141592654


#define BALL_RADIUS 5

#define TRUE true
#define FALSE false


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
//  .frequency_HZ = 50, // frequency
  .duty_cycle_percent = 50, // duty cycle
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
 * Shield Physics Task Control Block
 */
OS_TCB ShieldPhysics_TCB;

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

OS_TCB HM_Physics_TCB;

/******************************************************************************
 * Task Stacks
 ******************************************************************************/

/**
 * Button Task Stack of size TASK_STK_SIZE
 */
CPU_STK  LCDDisplay_TaskStack [TASK_STK_SIZE];

CPU_STK ShieldPhysics_TaskStack[TASK_STK_SIZE];

CPU_STK HM_Physics_TaskStack [TASK_STK_SIZE];

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

ShieldPosition_t shield_position;
Harkonnen_Mass_Position_t hm_position;
GameConfigurations_v3_t config;
BoostTiming_t boostTime;


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
 * Task declaration for task responsible for updating the Harkonnen Mass physics engine
 * @param random_arguement_parameter
 */
static void HM_Physics_Task (void* random_arguement_parameter);


///**
// * Task declaration for task responsible for updating the shield physics
// * @param random_arguement_parameter
// */
//static void ShieldPhysics_Task (void* random_arguement_parameter);
//

/**
 * Task declaration function for lower priority task intended to save
 * energy by putting the micro-controller to sleep
 * @param random_arguement_parameter : void* pointer which is needed for
 * conforming to u-os syntax standards
 */
//static void Idle(void* random_arguement_parameter);



/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void){
  GPIO_IntClear(1 << BUTTON0_pin);
  RTOS_ERR err;

  boostTime.mostRecentPressTime = OSTimeGet(&err);
  OSSemPost(&boost_button_semaphore, OS_OPT_POST_ALL, &err);



}

/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void){
  GPIO_IntClear(1 << BUTTON1_pin);
//TODO///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  RTOS_ERR err;
  OSSemPost(&laser_semaphore, OS_OPT_POST_ALL, &err);
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

    OSTaskCreate(&HM_Physics_TCB, /* Create the start task */
          "HM Physics",
          HM_Physics_Task,
          DEF_NULL,
          HM_PHYSICS_PRIORITY,
          &HM_Physics_TaskStack[0],
          (TASK_STK_SIZE / 10),
          TASK_STK_SIZE,
          0u,
          0u,
          DEF_NULL,
          (OS_OPT_TASK_STK_CLR),
          &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSTaskCreate(&LaserTask_TCB, /* Create the start task */
          "Laser Task",
          LaserTask,
          DEF_NULL,
          LASER_TASK_PRIORITY,
          &Laser_TaskStack[0],
          (TASK_STK_SIZE / 10),
          TASK_STK_SIZE,
          0u,
          0u,
          DEF_NULL,
          (OS_OPT_TASK_STK_CLR),
          &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSTaskCreate(&BoostTask_TCB, /* Create the start task */
          "Boost Task",
          BoostTask,
          DEF_NULL,
          BOOST_TASK_PRIORITY,
          &Boost_TaskStack[0],
          (TASK_STK_SIZE / 10),
          TASK_STK_SIZE,
          0u,
          0u,
          DEF_NULL,
          (OS_OPT_TASK_STK_CLR),
          &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSTaskCreate(&SliderState_TCB, /* Create the start task */
          "Slider State Task",
          SliderStateTask,
          DEF_NULL,
          SLIDER_STATE_TASK_PRIORITY,
          &SliderState_TaskStack[0],
          (TASK_STK_SIZE / 10),
          TASK_STK_SIZE,
          0u,
          0u,
          DEF_NULL,
          (OS_OPT_TASK_STK_CLR),
          &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

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




  OSSemCreate (&hm_physics_semaphore,
                 "Harkonnen Mass Physics Update Semaphore",
                 0,
                 &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSTmrCreate(&hm_physics_timer,
                      "Harkonnen Mass Physics Update Timer",
                      20,
                      (int) config.tauPhysics/10,
                      OS_OPT_TMR_PERIODIC,
                      hm_physics_timer_callback_function,
                      (void*) 0,
                      &err);
   EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


//   OSSemCreate (&shield_physics_semaphore,
//                  "Shield Physics Update Semaphore",
//                  0,
//                  &err);
//   EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//   OSTmrCreate(&shield_physics_timer,
//                       "Shield Physics Update Timer",
//                       2,
//                       (int) config.tauPhysics/10,
//                       OS_OPT_TMR_PERIODIC,
//                       shield_physics_timer_callback_function,
//                       (void*) 0,
//                       &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSSemCreate (&slider_semaphore,
                   "Slider State Semaphore",
                   0,
                   &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


    OSTmrCreate(&slider_timer,
                           "Slider State Timer",
                           20,
                           5,
                           OS_OPT_TMR_PERIODIC,
                           slider_state_timer_callback_function,
                           (void*) 0,
                           &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSSemCreate (&slider_semaphore,
                   "Slider State Semaphore",
                   0,
                   &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));



    OSSemCreate (&LCD_semaphore,
                   "LCD Semaphore",
                   0,
                   &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


    OSTmrCreate(&LCD_timer,
                           "LCD Timer",
                           20,
                           (int) config.tauLCD/10,
                           OS_OPT_TMR_PERIODIC,
                           lcd_timer_callback_function,
                           (void*) 0,
                           &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSTmrCreate(&boost_timer,
                           "Boost Timer",
                           (int) config.shieldConfig.boostConfig.armingWindowBeforeImpact/10,
                           2,
                           OS_OPT_TMR_ONE_SHOT,
                           boost_timer_callback_function,
                           (void*) 0,
                           &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


    OSSemCreate (&laser_semaphore,
                   "Laser Semaphore",
                   0,
                   &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSSemCreate (&boost_deactivate_semaphore,
                       "Boost De-activation Semaphore",
                       0,
                       &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSSemCreate (&boost_button_semaphore,
                          "Boost Button Semaphore",
                          0,
                          &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSFlagCreate (&game_over_flags,
                        "End Game Flags",
                        0x00,
                        &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


    pwm_init (&PWM0_timer, &PWM0);

}


void app_init(void)
{
  gpio_open();
  // blink_init();
  // task_init();

  GameConfigurations_v3_t ConfigData = {
       .version = 4,
       .tauPhysics = 10,
       .tauLCD = 150,
       .gravity = -4800,
       .canyonSize = 100000,
       .holtzmanMassesConfig =
       {
          .num =  3,
          .displayDiameter = 8000,
          .initialConditions = 0,
          .initialVelocity =
          {
              .xvel = 0,
              .yvel = 0
          },
          .initialHorizontalPosition = 0,
          .userDefinedModeInput = {0,0,0,0,0,0,0,0},
       },
       .platformConfig =
       {
           .maxForce = 2500,
           .mass = 100,
           .length = 20000,
           .cw_bounce =
           {
               .enabled = true,
               .limited = false,
               .maxPlatformBounceSpeed = 500
           },
       },
       .shieldConfig =
       {
           .minimumEffectivePerpendicularSpeed = 1000,
           .exclusivelyPassiveBounceKineticEnergyReduction = 70,
           .boostConfig =
           {
               .kineticEnergyIncrease = 40,
               .armingWindowBeforeImpact = 500,
               .rechargeTimeAfterDisarm = 1000
           },
       },
       .laserConfig =
       {
           .numActivations = 1,
           .automaticControl = false
       }
  };

  config = ConfigData;

  Harkonnen_Mass_Position_t HM0 =
      {
          .x_cm = 0,
          .y_cm = (128/118) * config.canyonSize - 100,

          .v = {
              .xvel = 0,
              .yvel = 0
          },

          .max_x_cm = config.canyonSize/2 - (config.holtzmanMassesConfig.displayDiameter/2),
          .min_x_cm = -config.canyonSize/2 + (config.holtzmanMassesConfig.displayDiameter/2),

          .max_y_cm = (128/118) * config.canyonSize,
          .min_y_cm = 0,

          .mass = 5,

          .numMasses = 5
      };
//
  hm_position = HM0;

  ShieldPosition_t temp_shield = {
      .x_cm = 0,
      .max_x_cm = config.canyonSize/2 - config.platformConfig.length/2,
      .min_x_cm = -config.canyonSize/2 + config.platformConfig.length/2,

      .velocity_x = 0,
      .acceleration_x = 0,
      .isBoosted = false,

      .current_force = 0,
      .mass = 5
  };



  shield_position = temp_shield;


}


static void HM_Physics_Task (void* random_arguement_parameter)
{
  PP_UNUSED_PARAM(random_arguement_parameter);
  RTOS_ERR err;
  // pend on a semaphore

  OSTmrStart(&hm_physics_timer, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


  while (1)
    {
      // Pend on the semaphore
      OSSemPend(&hm_physics_semaphore,
                100,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);
      // if the semaphore pend didn't time out
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
      {

          // mutex locks on the shield_position and hm_position
          // mutex locks on the shield_position and hm_position
          OSMutexPend (&HM_mux,
                        10,
                        OS_OPT_PEND_BLOCKING,
                        NULL,
                        &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

          // mutex locks on the shield_position and hm_position
          OSMutexPend (&shield_mux,
                       10,
                       OS_OPT_PEND_BLOCKING,
                       NULL,
                       &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

          update_hm_physics ( &shield_position,
                              &hm_position,
                              &config,
                              &game_over_flags);
          OSMutexPost(&shield_mux,
                      OS_OPT_POST_NONE,
                      &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);
          OSMutexPost(&HM_mux,
                      OS_OPT_POST_NONE,
                      &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

      }


    }
}


static void LaserTask(void* random_arguement_parameter)
{
  RTOS_ERR err;

  PP_UNUSED_PARAM(random_arguement_parameter);

  uint8_t num_laser_activations = config.laserConfig.numActivations;

  while (1)
  {

    OSSemPend(&laser_semaphore,
                          10,
                          OS_OPT_PEND_BLOCKING,
                          NULL,
                          &err);
    if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
    {
        if (num_laser_activations > 0)
        {
          num_laser_activations --;

          OSMutexPend (&HM_mux,
                                  10,
                                  OS_OPT_PEND_BLOCKING,
                                  NULL,
                                  &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);


          hm_position.x_cm = config.holtzmanMassesConfig.initialHorizontalPosition;
          hm_position.y_cm = hm_position.max_y_cm - 20;
          hm_position.v = config.holtzmanMassesConfig.initialVelocity;
          hm_position.numMasses --;

          OSMutexPost(&HM_mux,
                      OS_OPT_POST_NONE,
                      &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

        }

    }
  }
}


static void SliderStateTask(void* random_arguement_parameter)
{
  RTOS_ERR err;

  PP_UNUSED_PARAM(random_arguement_parameter);

  bool channel_values [4];

  float force_HL = -config.platformConfig.maxForce;
  float force_L  = -config.platformConfig.maxForce/2;
  float force_R  = config.platformConfig.maxForce/2;
  float force_HR = config.platformConfig.maxForce;

  OSTmrStart(&slider_timer, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  CAPSENSE_Init();
  while (1)
  {
      // Add kernel call to awake the task periodically
      OSSemPend(&slider_semaphore,
                      0,
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
      {
      OSTimeDly(5, OS_OPT_TIME_DLY, &err);

        // Add call to read the slider position
        CAPSENSE_Sense();

        for (int i = 0; i < 4; i++)
          {
            channel_values[i] = 0;
            channel_values[i] = CAPSENSE_getPressed(i);
          }

        float force = 0;

        if (channel_values[2])
        {
          force = force_R;
        }
        if (channel_values[1])
        {
          force = force_L;
        }
        if (channel_values[0])
        {
          force = force_HL;
        }
        if (channel_values[3])
        {
          force = force_HR;
        }

        OSMutexPend (&shield_mux,
                      10,
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


        shield_position.current_force = force;

        OSMutexPost(&shield_mux,
                    OS_OPT_POST_NONE,
                    &err);
        EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);


      }

  }



}


//static void ShieldPhysics_Task (void* random_arguement_parameter)
//{
//
//  RTOS_ERR err;
//  // pend on a semaphore
//
//
//
//  OSTmrStart(&shield_physics_timer, &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//
//  while (1)
//    {
//      // Pend on the semaphore
//      OSSemPend(&shield_physics_semaphore,
//                100,
//                OS_OPT_PEND_BLOCKING,
//                NULL,
//                &err);
//      // if the semaphore pend didn't time out
//      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
//      {
//
//          // mutex locks on the shield_position and hm_position
////          OSMutexPend (&shield_mux,
////                                                      100,
////                                                      OS_OPT_PEND_BLOCKING,
////                                                      NULL,
////                                                      &err);
////          EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//          // update_shield_physics ( &shield_position, &config );
////          OSMutexPost(&shield_mux,
////                              OS_OPT_POST_NONE,
////                              &err);
//
//      }
//
//    }
//}




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

    glibContext.backgroundColor = Black;
    glibContext.foregroundColor = White;


    // Define the Left Wall
    left_wall.xMin = 0;
    left_wall.xMax = 4;
    left_wall.yMin = 0;
    left_wall.yMax = 127;

    // Define the Right Wall
    right_wall.xMin = 122;
    right_wall.xMax = 127;
    right_wall.yMin = 0;
    right_wall.yMax = 127;

    float cm_per_pixel = (float) config.canyonSize / (right_wall.xMin - left_wall.xMax);
    float height_scale_factor = (float) config.canyonSize / 118;


    uint8_t hm_x;
    uint8_t hm_y;

    uint32_t numPoints = 6;
    int32_t ball_polyPoints[12];

    uint8_t shield_position_center_pixel;

    float radius = config.holtzmanMassesConfig.displayDiameter/cm_per_pixel/2;

    OS_FLAGS game_over;


    OSTmrStart(&LCD_timer, &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));



  while(1){

//      OSTimeDly(10, OS_OPT_TIME_DLY, &err);

      OSSemPend(&LCD_semaphore,
                      100,
                      OS_OPT_PEND_BLOCKING,
                      NULL,
                      &err);
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
      {



          game_over =  OSFlagPend (&game_over_flags,
                                missed_platform | fell_thru_platform | winner_winner,
                                1,
                                OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME + OS_OPT_PEND_NON_BLOCKING,
                                (CPU_TS *)0,
                                &err);
         if (game_over != 0x00)
         {

             OSTmrDel (&LCD_timer,
                                    &err);
             EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

//             OSTmrDel (&slider_timer,
//                         &err);
//             EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

             OSTmrDel (&hm_physics_timer,
                      &err);
             EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

             pwm_start(&PWM0_timer);

             if (game_over == missed_platform)
             {
                 glibContext.backgroundColor = White;
                 glibContext.foregroundColor = Black;

                 GLIB_clear(&glibContext);
                 GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);
                 // write that you missed the platform

                 GLIB_drawStringOnLine(&glibContext,
                                         "GAME OVER",
                                         5,
                                         GLIB_ALIGN_LEFT,
                                         5,
                                         5,
                                         false);
                 GLIB_drawStringOnLine(&glibContext,
                                      "Your shield failed",
                                      6,
                                      GLIB_ALIGN_LEFT,
                                      5,
                                      5,
                                      false);
                 GLIB_drawStringOnLine(&glibContext,
                                       "to protect your",
                                       7,
                                       GLIB_ALIGN_LEFT,
                                       5,
                                       5,
                                       false);
                 GLIB_drawStringOnLine(&glibContext,
                                      "base",
                                      8,
                                      GLIB_ALIGN_LEFT,
                                      5,
                                      5,
                                      false);
                 DMD_updateDisplay();

                 while (1)
                   {}



             }
             if (game_over == fell_thru_platform)
             {
                 glibContext.backgroundColor = White;
                 glibContext.foregroundColor = Black;
                 GLIB_clear(&glibContext);
                 GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);

                 GLIB_drawStringOnLine(&glibContext,
                                         "GAME OVER",
                                         5,
                                         GLIB_ALIGN_LEFT,
                                         5,
                                         5,
                                         false);
                 GLIB_drawStringOnLine(&glibContext,
                                      "Your shield was",
                                      6,
                                      GLIB_ALIGN_LEFT,
                                      5,
                                      5,
                                      false);
                 GLIB_drawStringOnLine(&glibContext,
                                       "penetrated",
                                       7,
                                       GLIB_ALIGN_LEFT,
                                       5,
                                       5,
                                       false);

                 DMD_updateDisplay();

                 while (1)
                   {}
             }
             if (game_over == winner_winner)
             {
                 glibContext.backgroundColor = White;
                 glibContext.foregroundColor = Black;
                 GLIB_clear(&glibContext);
                 GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);
                 GLIB_drawStringOnLine(&glibContext,
                                         "YOU WIN",
                                         5,
                                         GLIB_ALIGN_LEFT,
                                         5,
                                         5,
                                         true);
                 GLIB_drawStringOnLine(&glibContext,
                                         "The Harkonnens",
                                         6,
                                         GLIB_ALIGN_LEFT,
                                         5,
                                         5,
                                         true);
                 GLIB_drawStringOnLine(&glibContext,
                                  "are retreating",
                                  7,
                                  GLIB_ALIGN_LEFT,
                                  5,
                                  5,
                                  true);
                 DMD_updateDisplay();

                 while (1)
                   {}

             }

         }

          // Get the position of the HM


          // Get the position of the shield

          // mutex locks on the shield_position and hm_position
          OSMutexPend (&shield_mux,
                        10,
                        OS_OPT_PEND_BLOCKING,
                        NULL,
                        &err);
          EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
          shield_position_center_pixel = (uint8_t) ( (shield_position.x_cm/cm_per_pixel) + 64);
          bool boost = shield_position.isBoosted;
          OSMutexPost(&shield_mux,
                              OS_OPT_POST_NONE,
                              &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);



          shield.xMin = shield_position_center_pixel - (config.platformConfig.length/cm_per_pixel/2);
          shield.xMax = shield_position_center_pixel + (config.platformConfig.length/cm_per_pixel/2);
          shield.yMin = 124;
          shield.yMax = 127;

          if (boost)
            {
              shield.yMin = 120;
            }


          OSMutexPend (&HM_mux,
                        10,
                        OS_OPT_PEND_BLOCKING,
                        NULL,
                        &err);
          EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
          hm_x =   (uint8_t) ( (hm_position.x_cm/cm_per_pixel) + 64);
          hm_y =   (uint8_t) 127 - (hm_position.y_cm/height_scale_factor) ;
          OSMutexPost(&HM_mux,
                              OS_OPT_POST_NONE,
                              &err);



          // Update the polygon points of the
          ball_polyPoints[0] = (int32_t) ((float)hm_x + (cos((90 * PI) / 180)) * radius);
          ball_polyPoints[1] = (int32_t) ((float)hm_y + (sin((90* PI) / 180)) * radius);
          ball_polyPoints[2] = (int32_t) ((float)hm_x + (cos((30* PI) / 180)) * radius);
          ball_polyPoints[3] = (int32_t) ((float)hm_y + (sin((30* PI) / 180)) * radius);
          ball_polyPoints[4] = (int32_t) ((float)hm_x + (cos((330* PI) / 180)) * radius);
          ball_polyPoints[5] = (int32_t) ((float)hm_y + (sin((330* PI) / 180)) * radius);
          ball_polyPoints[6] = (int32_t) ((float)hm_x + (cos((270* PI) / 180)) * radius);
          ball_polyPoints[7] = (int32_t) ((float)hm_y + (sin((270* PI) / 180)) * radius);
          ball_polyPoints[8] = (int32_t) ((float)hm_x + (cos((210* PI) / 180)) * radius);
          ball_polyPoints[9] = (int32_t) ((float)hm_y + (sin((210* PI) / 180)) * radius);
          ball_polyPoints[10] = (int32_t) ((float)hm_x + (cos((150* PI) / 180)) * radius);
          ball_polyPoints[11] = (int32_t) ((float)hm_y + (sin((150* PI) / 180)) * radius);


//          ball_polyPoints[0] = hm_x;
//          ball_polyPoints[1] = hm_y + 6;
//          ball_polyPoints[2] = hm_x + 5;
//          ball_polyPoints[3] = hm_y + 3;
//          ball_polyPoints[4] = hm_x + 5;
//          ball_polyPoints[5] = hm_y - 3;
//          ball_polyPoints[6] = hm_x;
//          ball_polyPoints[7] = hm_y - 6;
//          ball_polyPoints[8] = hm_x - 5;
//          ball_polyPoints[9] = hm_y - 3;
//          ball_polyPoints[10] = hm_x - 5;
//          ball_polyPoints[11] = hm_y + 3;

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
      }

  }
}


static void BoostTask(void* random_arguement_parameter)
{
  RTOS_ERR err;
  PP_UNUSED_PARAM(random_arguement_parameter);


  while (1)
  {
      OSSemPend(&boost_button_semaphore,
                0,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);

      if (boostTime.mostRecentPressTime > boostTime.mostRecentBoostActivationTime + config.shieldConfig.boostConfig.armingWindowBeforeImpact/100 + config.shieldConfig.boostConfig.rechargeTimeAfterDisarm/100)
      {
          boostTime.mostRecentBoostActivationTime = boostTime.mostRecentPressTime;

          OSMutexPend (&shield_mux,
                        10,
                        OS_OPT_PEND_BLOCKING,
                        NULL,
                        &err);
          EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
          shield_position.isBoosted = true;

          OSMutexPost(&shield_mux,
                              OS_OPT_POST_NONE,
                              &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

          // start OS timer to set isBoosted to false
          OSTmrStart(&boost_timer, &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);



      }
      OSSemPend(&boost_deactivate_semaphore,
                0,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);
      // if the semaphore pend didn't time out
      if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE))
      {
          OSMutexPend (&shield_mux,
                           10,
                           OS_OPT_PEND_BLOCKING,
                           NULL,
                           &err);
          EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
          shield_position.isBoosted = false;
//
          OSMutexPost(&shield_mux,
                               OS_OPT_POST_NONE,
                               &err);
          EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

      }



  }
}


static void DesiredShieldForceTask(void* random_arguement_parameter)
{
  RTOS_ERR err;
  PP_UNUSED_PARAM(random_arguement_parameter);


  // pwm_start(&PWM0_timer);

  float t_till_impact;
  float F_needed;
  float x_crit;

  while (1)
    {
      OSTimeDly(100, OS_OPT_TIME_DLY, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // mutex


      OSMutexPend (&HM_mux,
                    100,
                    OS_OPT_PEND_BLOCKING,
                    NULL,
                    &err);
      EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

      // mutex locks on the shield_position and hm_position
      OSMutexPend (&shield_mux,
                   100,
                   OS_OPT_PEND_BLOCKING,
                   NULL,
                   &err);
      EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

      float t1 = (-hm_position.v.yvel + sqrtf(pow(hm_position.v.yvel, 2) - (2*(config.gravity)*hm_position.y_cm)))/(config.gravity);
      float t2 = (-hm_position.v.yvel - sqrtf(pow(hm_position.v.yvel, 2) - (2*(config.gravity)*hm_position.y_cm)))/(config.gravity);

      t_till_impact = t1 > t2 ? t1 : t2;

      EFM_ASSERT(t_till_impact >= 0);

      x_crit = shield_position.x_cm + shield_position.velocity_x * t_till_impact;

      F_needed = sqrtf(2*shield_position.mass*(shield_position.x_cm + shield_position.velocity_x*t_till_impact - x_crit));


      OSMutexPost(&shield_mux,
                  OS_OPT_POST_NONE,
                  &err);

      EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

      OSMutexPost(&HM_mux,
                  OS_OPT_POST_NONE,
                  &err);
      EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);


      // Translate Desired Force into a duty cycle





    }

}

//
//static void Idle(void* random_arguement_parameter)
//{
//  RTOS_ERR err;
//
//  PP_UNUSED_PARAM(random_arguement_parameter);
//
//  while (1)
//    {
//      EMU_EnterEM1();
//      OSTimeDly(100, OS_OPT_TIME_DLY, &err);
//      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//    }
//
//}




void lcd_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
{
  PP_UNUSED_PARAM(p_arg);
  PP_UNUSED_PARAM(p_tmr);
  RTOS_ERR err;
  OSSemPost(&LCD_semaphore, OS_OPT_POST_ALL, &err);
}

void slider_state_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
{
  PP_UNUSED_PARAM(p_arg);
  PP_UNUSED_PARAM(p_tmr);
  RTOS_ERR err;
  OSSemPost(&slider_semaphore, OS_OPT_POST_ALL, &err);
}


//void shield_physics_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
//{
//  PP_UNUSED_PARAM(p_arg);
//  PP_UNUSED_PARAM(p_tmr);
//  RTOS_ERR err;
//  OSSemPost(&shield_physics_semaphore, OS_OPT_POST_ALL, &err);
//}


void hm_physics_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
{
  PP_UNUSED_PARAM(p_arg);
  PP_UNUSED_PARAM(p_tmr);
  RTOS_ERR err;
  OSSemPost(&hm_physics_semaphore, OS_OPT_POST_ALL, &err);
}


void boost_timer_callback_function(OS_TMR* p_tmr, void* p_arg)
{
  PP_UNUSED_PARAM(p_arg);
  PP_UNUSED_PARAM(p_tmr);
  RTOS_ERR err;
  OSSemPost(&boost_deactivate_semaphore, OS_OPT_POST_ALL, &err);
}


