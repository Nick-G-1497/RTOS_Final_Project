/*
 * pwm_driver.c
 *
 *  Created on: Mar 18, 2022
 *      Author: nickg
 */
#include "pwm.h"



void pwm_callback_function (OS_TMR* p_tmr, void* p_arg)
{
    // PP_UNUSED_PARAM(p_arg);
    pwm_paramaters_t* this = (pwm_paramaters_t *) p_arg;
    PP_UNUSED_PARAM(p_tmr);
    RTOS_ERR err;
    

    // increment the counter variable
    this->counter ++;

    if (this->counter == 1)
    {
        digitalWrite(this->port, this->pin, 1);
    }

    // compare the counter against the DC (duty cycle) value. On match
    //      drive PWM pin LOW
    if (this->counter >= this->duty_cycle_percent/100)
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


void pwm_init (OS_TMR* timer, pwm_paramaters_t* parameters)
{
    RTOS_ERR err;

    float freq  = (float)parameters->frequency_HZ;

    OS_TICK os_period = (OS_TICK)((1/freq) * 1000/10);

    // create the OS timer kernel object
     OSTmrCreate(timer,
                     parameters->timer_name,
                     2,  // OS Tick delay before timer starts
                     1,  // OS Tick period
                     OS_OPT_TMR_PERIODIC,
                     pwm_callback_function,
                     (void *) parameters,
                     &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


}

void pwm_start (OS_TMR* timer, pwm_paramaters_t* parameters)
{
    RTOS_ERR err;
    OSTmrStart (timer,
                         &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
         
}


void pwm_stop (OS_TMR* timer, pwm_paramaters_t* parameters)
{
    RTOS_ERR err;
    OSTmrStop (timer,
                        OS_OPT_TMR_NONE,
                        pwm_callback_function,
                        &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

void pwm_delete (OS_TMR* timer, pwm_paramaters_t* parameters)
{
    RTOS_ERR err;
    OSTmrDel (timer,
            &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}   
