/**
 * @file PWM
 * @author Nick Goralka
 * @brief PWM driver
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef PWM_H
#define PWM_H

#include "gpio.h"
#include "os.h"

typedef struct{
    const float frequency_HZ;
    volatile unsigned int duty_cycle_percent;

    CPU_CHAR* timer_name;

    const GPIO_Port_TypeDef port;
    const unsigned int pin;

    volatile unsigned int counter;
    
} pwm_paramaters_t;


extern void pwm_callback_function (OS_TMR* p_tmr, void* p_arg);

extern void pwm_init (OS_TMR* timer, pwm_paramaters_t* parameters);

extern void pwm_start (OS_TMR* timer, pwm_paramaters_t* parameters);

extern void pwm_stop (OS_TMR* timer, pwm_paramaters_t* parameters);

extern void pwm_delete (OS_TMR* timer, pwm_paramaters_t* parameters);

#endif  // APP_H
