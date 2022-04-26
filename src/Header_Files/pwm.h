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
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"


#define PWM0_TIMER TIMER0
#define PWM1_TIMER TIMER1

typedef struct{
    // const float frequency_HZ;
    volatile unsigned int duty_cycle_percent;

    CPU_CHAR* timer_name;

    const GPIO_Port_TypeDef port;
    const unsigned int pin;

    volatile unsigned int counter;
    
} pwm_paramaters_t;


//static pwm_paramaters_t pwm0_params =
//{
//    .frequency_HZ = 60,
//    .duty_cycle_percent = 75,
//    .timer_name = "PWM 0 Timer",
//    .port = LED0_port,
//    .pin = LED0_pin,
//    .counter = 0
//};
//
//
//static pwm_paramaters_t pwm1_params =
//{
//    .frequency_HZ = 60,
//    .duty_cycle_percent = 75,
//    .timer_name = "PWM 1 Timer",
//    .port = LED1_port,
//    .pin = LED1_pin,
//    .counter = 0
//};



extern void pwm_callback_function (OS_TMR* p_tmr, void* p_arg);

extern void pwm_init (OS_TMR* timer, pwm_paramaters_t* parameters);

extern void pwm_start (OS_TMR* timer);

extern void pwm_stop (OS_TMR* timer);



#endif  // APP_H
