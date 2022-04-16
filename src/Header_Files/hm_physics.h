/*
 * hm_physics.h
 *
 *  Created on: Apr 12, 2022
 *      Author: nickg
 */

#include "config_v3.h"
#include <math.h>
#include "os.h"

#ifndef SRC_HEADER_FILES_HM_PHYSICS_H_
#define SRC_HEADER_FILES_HM_PHYSICS_H_




static OS_TMR hm_physics_timer;


/**
 * @brief Update the mass' position
 *
 */
void update_hm_physics(ShieldPosition_t* shield, Harkonnen_Mass_Position_t* state, GameConfigurations_v3_t* config);

//
///**
// * @brief Slider timer callback function
// * @param p_tmr
// * @param p_arg
// */
//void hm_physics_timer_callback_function(OS_TMR* p_tmr, void* p_arg);
//


#endif /* SRC_HEADER_FILES_HM_PHYSICS_H_ */
