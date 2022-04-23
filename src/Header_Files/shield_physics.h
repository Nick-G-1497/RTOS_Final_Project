/*
 * shield_physics.h
 *
 *  Created on: Apr 15, 2022
 *      Author: nickg
 */
#include "config_v3.h"
#include "os.h"


#ifndef SRC_HEADER_FILES_SHIELD_PHYSICS_H_
#define SRC_HEADER_FILES_SHIELD_PHYSICS_H_


//static OS_SEM shield_physics_semaphore;


void update_shield_physics (ShieldPosition_t* state, GameConfigurations_v3_t* config);


///**
// * @brief Slider timer callback function
// * @param p_tmr
// * @param p_arg
// */
//void shield_physics_timer_callback_function(OS_TMR* p_tmr, void* p_arg);




#endif /* SRC_HEADER_FILES_SHIELD_PHYSICS_H_ */
