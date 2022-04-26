/*
 * pwm_driver.c
 *
 *  Created on: Mar 18, 2022
 *      Author: nickg
 */
#include "pwm.h"


/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
//void TIMER0_IRQHandler(void)
//{
//  /* Clear flag for TIMER0 overflow interrupt */
//  TIMER_IntClear(TIMER0, TIMER_IF_OF);
//
//
//
//  // increment the counter variable
//   pwm0_params.counter ++;
//
//    if (pwm0_params.counter == 1)
//    {
//        digitalWrite(pwm0_params.port, pwm0_params.pin, 1);
//    }
//
//    // compare the counter against the DC (duty cycle) value. On match
//    //      drive PWM pin LOW
//    if (pwm0_params.counter >= pwm0_params.duty_cycle_percent)
//    {
//        digitalWrite(pwm0_params.port, pwm0_params.pin, 0);
//    }
//
//
//    // compare the counter against the period value. On match -> {Drive PWM
//    //     Pin HIGH, Reset the counter}
//    if (pwm0_params.counter >= 100)
//    {
//        digitalWrite(pwm0_params.port, pwm0_params.pin, 1);
//        pwm0_params.counter = 0;
//
//    }
//}

 void pwm_callback_function (OS_TMR* p_tmr, void* p_arg)
 {

     pwm_paramaters_t* this = (pwm_paramaters_t *) p_arg;

     RTOS_ERR err;


     // increment the counter variable
     this->counter ++;

     if (this->counter == 1)
     {
         digitalWrite(this->port, this->pin, 1);
     }

     // compare the counter against the DC (duty cycle) value. On match
     //      drive PWM pin LOW
     if (this->counter >= this->duty_cycle_percent)
     {
         digitalWrite(this->port, this->pin, 0);
     }


     // compare the counter against the period value. On match -> {Drive PWM
     //     Pin HIGH, Reset the counter}
     if (this->counter >= 100)
     {
         digitalWrite(this->port, this->pin, 1);
         this->counter = 0;

     }


 }


//void pwm_init (TIMER_TypeDef *  timer, pwm_paramaters_t* parameters)
//{
//
//    // Convert the desired frequency into a number of ticks
//    /* 13671 Hz -> 14Mhz (clock frequency) / 1024 (prescaler)
//     Setting TOP to 27342 results in an overflow each 2 seconds */
//     float scaled_clock_frequency = 13671;
//
//     int top = (int) ( scaled_clock_frequency / (parameters->frequency_HZ*100) );
//
//    /* Select timer parameters */
//    TIMER_Init_TypeDef timerInit =
//    {
//        .enable     = true,
//        .debugRun   = true,
//        .prescale   = timerPrescale1024,
//        .clkSel     = timerClkSelHFPerClk,
//        .fallAction = timerInputActionNone,
//        .riseAction = timerInputActionNone,
//        .mode       = timerModeUp,
//        .dmaClrAct  = false,
//        .quadModeX4 = false,
//        .oneShot    = false,
//        .sync       = false,
//    };
//
//
//    /* Enable overflow interrupt */
//    TIMER_IntEnable(timer, TIMER_IF_OF);
//
//
//    /* Set TIMER Top value */
//    TIMER_TopSet(timer, top);
//
//    /* Configure TIMER */
////     TIMER_Init(timer, &timerInit);
//
//
//}

 void pwm_init(OS_TMR* timer, pwm_paramaters_t* parameters)
 {
   RTOS_ERR err;

   OSTmrCreate (timer,
                      parameters->timer_name,
                      20,
                      10,
                      OS_OPT_TMR_PERIODIC,
                      pwm_callback_function,
                      parameters,
                      &err);
   EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
 }


 void pwm_start (OS_TMR* timer)
 {
     RTOS_ERR err;
     OSTmrStart (timer,
                          &err);
     EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
         
 }

//void pwm_start (TIMER_TypeDef *  timer)
//{
//    if (timer == TIMER0)
//    {
//        /* Enable TIMER0 interrupt vector in NVIC */
//        NVIC_EnableIRQ(TIMER0_IRQn);
//    }
//
//    if (timer == TIMER1)
//    {
//        /* Enable TIMER1 interrupt vector in NVIC */
//        NVIC_EnableIRQ(TIMER1_IRQn);
//    }
//}


//void pwm_stop (TIMER_TypeDef *  timer)
//{
//    if (timer == TIMER0)
//    {
//        /* Enable TIMER0 interrupt vector in NVIC */
//        digitalWrite(pwm0_params.port, pwm0_params.pin, 0);
//        NVIC_DisableIRQ(TIMER0_IRQn);
//    }
//
//    if (timer == TIMER1)
//    {
//        /* Enable TIMER1 interrupt vector in NVIC */
//        digitalWrite(pwm1_params.port, pwm1_params.pin, 0);
//        NVIC_DisableIRQ(TIMER1_IRQn);
//    }
//}

 void pwm_stop (OS_TMR* timer)
 {
     RTOS_ERR err;
     OSTmrStop (timer,
                         OS_OPT_TMR_NONE,
                         pwm_callback_function,
                         &err);
     EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
 }

 void pwm_delete (OS_TMR* timer)
 {
     RTOS_ERR err;
     OSTmrDel (timer,
             &err);
     EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
 }
