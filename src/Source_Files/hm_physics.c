/*
 * hm_physics.c
 *
 *  Created on: Apr 12, 2022
 *      Author: nickg
 */


#include "hm_physics.h"
#include "os.h"
#include <math.h>

OS_FLAGS missed_platform = 0x01;
OS_FLAGS fell_thru_platform = 0x02;
OS_FLAGS winner_winner = 0x04;

void update_hm_physics(ShieldPosition_t* shield, Harkonnen_Mass_Position_t* state, GameConfigurations_v3_t* config, OS_FLAG_GRP* game_over_flags){


  /******************************************************************************
     * Calculate Acceleration
     ******************************************************************************/
    float F = shield->current_force;
    float m = shield->mass;
    shield->acceleration_x = (F/m) * 100 ; // [ cm / s**2 ]


    /*******************************************************************************
     * Calculate Velocity
     ******************************************************************************/
    shield->velocity_x = shield->velocity_x + (shield->acceleration_x * config->tauPhysics/1000);



    /*******************************************************************************
     * Calculate Position
     ******************************************************************************/
    float predicted_x = shield->x_cm + shield->velocity_x * ((float) config->tauPhysics /1000);

    // If the shield is going to bounce off the right wall
    if ( (predicted_x >= shield->max_x_cm) )
    {
        if (config->platformConfig.cw_bounce.enabled == true)
        {
            shield->velocity_x = -shield->velocity_x;
            shield->x_cm = shield->max_x_cm;
        }
        else
        {
            shield->velocity_x = 0;
            shield->x_cm = shield->max_x_cm;
        }
    }

    // Else if the shield is going to bounce off the left wall
    else if ( (predicted_x <= shield->min_x_cm) )
    {
        if (config->platformConfig.cw_bounce.enabled == true )
        {
            shield->velocity_x = -shield->velocity_x;
            shield->x_cm = shield->min_x_cm;
        }
        else
        {
            shield->velocity_x = 0;
            shield->x_cm = shield->min_x_cm;
        }
    }

    // Otherwise the shield is unimpeded
    else
    {
        shield->x_cm = predicted_x;
    }











    /**********************************************************************************************************
     * Horizontal  axis
     **********************************************************************************************************/

    if (state->numMasses <= 0)
    {
      RTOS_ERR err;
      OSFlagPost (game_over_flags,
                  winner_winner,
                  OS_OPT_POST_FLAG_SET,
                  &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));;
    }

    // Calculate the position that the HM would be if it continued it's current trajectory
    float updated_x = state->x_cm + state->v.xvel * ((float) config->tauPhysics / 1000);

    // If the HM does not come in contact with either of the walls
    if ( (updated_x < state->max_x_cm) && (updated_x > state->min_x_cm) )
    {
        state->x_cm = updated_x; // update the position accordingly

        // TODO map x_cm to x_pixel_value
    }

    // Otherwise the mass comes in contact with the walls it will bounce off it
    else if ( updated_x >= state->max_x_cm)
    {
        state->x_cm = state->max_x_cm;
        state->v.xvel = - (state->v.xvel);

        // TODO map x_cm to x_pixel_value
    }

    else if (updated_x <= state->min_x_cm)
    {
        state->x_cm = state->min_x_cm;
        state->v.xvel = - (state->v.xvel);

        // TODO map x_cm to x_pixel_value
    }



    /************************************************************************************************************
     * Vertical axis
     ***********************************************************************************************************/
    state->v.yvel = state->v.yvel + ( (config->gravity * 10) * config->tauPhysics/1000 ) ;

    float predicted_y = state->y_cm + state->v.yvel* ((float) config->tauPhysics/1000);

    // check if the next position will have the mass hit the platform
    if (predicted_y <= state->min_y_cm)
    {
        // if the mass does come into contact with the shield
        if ( ( state-> x_cm  <= (shield->x_cm + config->platformConfig.length/2) ) && ( state->x_cm >= (shield->x_cm - config->platformConfig.length/2) ))
        {

            // check if the mass if below the velocity threshold
            if ( ( abs(state->v.yvel) <= config->shieldConfig.minimumEffectivePerpendicularSpeed) && (shield->isBoosted == false) )
            {
                RTOS_ERR err;
                OSFlagPost (game_over_flags,
                            fell_thru_platform,
                            OS_OPT_POST_FLAG_SET,
                            &err);
                EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
            }



            // calculate kinetic energy
            float v = sqrtf(pow(state->v.xvel,2) + pow(state->v.yvel,2)) ;
            float kinetic_energy = ( state->mass*(pow(v,2)) ) / 2 ;
            // printf("\n Old Kinetic Energy %f \n", kinetic_energy);
            // scale the kinetic energy
            if (shield->isBoosted)
            {
                kinetic_energy = kinetic_energy * (1 + (config->shieldConfig.boostConfig.kineticEnergyIncrease/100) );
            }
            else
            {
                kinetic_energy = kinetic_energy * (1 - config->shieldConfig.exclusivelyPassiveBounceKineticEnergyReduction/100) ;
            }

            float new_velocity_squared = (2*kinetic_energy) / state->mass ;

            // update the velocity
            state->y_cm = state->min_y_cm;
            state->v.yvel = sqrtf( new_velocity_squared - pow(state->v.xvel,2) );

            // TODO map y_cm to y_pixel_value
        }
        else
        {
            RTOS_ERR err;
            OSFlagPost (game_over_flags,
                                  missed_platform,
                                  OS_OPT_POST_FLAG_SET,
                                  &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        }


    }

    // Otherwise if the mass is knocked out of the canyon
    else if (predicted_y >= state->max_y_cm)
    {
        state->numMasses --;

        if (state->numMasses <= 0)
        {
            RTOS_ERR err;
            OSFlagPost (game_over_flags,
                        winner_winner,
                        OS_OPT_POST_FLAG_SET,
                        &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
            return;
        }

        // Get ready to instantly drop another mass
        state->y_cm = state->max_y_cm - 100;
        state->x_cm = config->holtzmanMassesConfig.initialHorizontalPosition;

        // TODO map x_cm to x_pixel_value
        // TODO map y_cm to y_pixel_value


        state->v = config->holtzmanMassesConfig.initialVelocity;
    }

    // Else the mass is just in free fall
    else
    {
        state->y_cm = predicted_y;
        return;
        // TODO map y_cm to y_pixel_value
    }

}






