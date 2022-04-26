/***************************************************************************//**
 * @file
 * @brief Blink examples functions
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

//#include "sl_simple_led.h"
//#include "sl_simple_led_instances.h"
#include "os.h"
#include "blink.h"
#include "glib.h"
#include "dmd.h"

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

//static OS_TCB tcb;
//static CPU_STK stack[BLINK_TASK_STACK_SIZE];
//
//static GLIB_Context_t glibContext;
//static int currentLine = 0;
//
///*******************************************************************************
// *********************   LOCAL FUNCTION PROTOTYPES   ***************************
// ******************************************************************************/
//
//static void blink_task(void *arg);
//
//
//
//void GPIO_EVEN_IRQHandler(void)
//{
// uint32_t intmask = GPIO_IntGet();
// if (intmask & (1 << BSP_GPIO_PB0_PIN))
// {
//   GPIO_IntClear(intmask);
// }
// else EFM_ASSERT(false);
//}
//
///***************************************************************************//**
//* @brief
//* the gpio odd irq handler will be called whenever and idd gpio interrupt is hit
//*
//* @details
//* if the interrupt mask is the same as the pin for button 1 then clear the interrupts and call the btn1 function
//* else efm_assert false
//*
//* @note
//* if any other gpio odd interrupt is triggered we want to assert false because none of them should even be enabled
//*
//******************************************************************************/
//void GPIO_ODD_IRQHandler(void)
//{
// uint32_t intmask = GPIO_IntGet();
// if (intmask & (1 << BSP_GPIO_PB1_PIN))
// {
//   GPIO_IntClear(intmask);
// }
// else EFM_ASSERT(false);
//}
//
///*******************************************************************************
// **************************   GLOBAL FUNCTIONS   *******************************
// ******************************************************************************/
//
///***************************************************************************//**
// * Initialize blink example.
// ******************************************************************************/
//void blink_init(void)
//{
//  RTOS_ERR err;
//  uint32_t status;
//
//  /* Enable the memory lcd */
//  status = sl_board_enable_display();
//  EFM_ASSERT(status == SL_STATUS_OK);
//
//  /* Initialize the DMD support for memory lcd display */
//  status = DMD_init(0);
//  EFM_ASSERT(status == DMD_OK);
//
//  /* Initialize the glib context */
//  status = GLIB_contextInit(&glibContext);
//  EFM_ASSERT(status == GLIB_OK);
//
//  glibContext.backgroundColor = White;
//  glibContext.foregroundColor = Black;
//
//  /* Fill lcd with background color */
//  GLIB_clear(&glibContext);
//
//  /* Use Narrow font */
//  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);
//
//  // Create Blink Task
//  OSTaskCreate(&tcb,
//               "blink task",
//               blink_task,
//               DEF_NULL,
//               BLINK_TASK_PRIO,
//               &stack[0],
//               (BLINK_TASK_STACK_SIZE / 10u),
//               BLINK_TASK_STACK_SIZE,
//               0u,
//               0u,
//               DEF_NULL,
//               (OS_OPT_TASK_STK_CLR),
//               &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//}
//
///***************************************************************************//**
// * Blink task.
// ******************************************************************************/
//static void blink_task(void *arg)
//{
//    PP_UNUSED_PARAM(arg);
//
//    RTOS_ERR err;
//    while (1)
//    {
//        OSTimeDly(1000, OS_OPT_TIME_DLY, &err);
//        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//        GPIO_PinOutToggle(LED0_port, LED0_pin);
//        GLIB_clear(&glibContext);
//        if(currentLine == 12)
//        {
//            currentLine = 0;
//        }
//        GLIB_drawStringOnLine(&glibContext,
//                              "Hello World!",
//                               currentLine++,
//                               GLIB_ALIGN_LEFT,
//                               5,
//                               5,
//                               true);
//        DMD_updateDisplay();
//    }
//
//}
//
