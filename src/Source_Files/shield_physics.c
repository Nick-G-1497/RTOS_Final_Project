/*
 * shield_physics.c
 *
 *  Created on: Apr 15, 2022
 *      Author: nickg
 */


#include "shield_physics.h"


void update_shield_physics(ShieldPosition_t* state, GameConfigurations_v3_t* config)
{

    /******************************************************************************
     * Calculate Acceleration
     ******************************************************************************/
    float F = state->current_force;
    float m = state->mass;
    state->acceleration_x = (F/m) * 100 ; // [ cm / s**2 ]


    /*******************************************************************************
     * Calculate Velocity
     ******************************************************************************/
    state->velocity_x = state->velocity_x + (state->acceleration_x * config->tauPhysics/1000);



    /*******************************************************************************
     * Calculate Position
     ******************************************************************************/
    float predicted_x = state->x_cm + state->velocity_x * ((float) config->tauPhysics /1000);

    // If the shield is going to bounce off the right wall
    if ( (predicted_x >= state->max_x_cm) )
    {
        if (config->platformConfig.cw_bounce.enabled == true)
        {
            state->velocity_x = -state->velocity_x;
            state->x_cm = state->max_x_cm;
        }
        else
        {
            state->velocity_x = 0;
            state->x_cm = state->max_x_cm;
        }
    }

    // Else if the shield is going to bounce off the left wall
    else if ( (predicted_x <= state->min_x_cm) )
    {
        if (config->platformConfig.cw_bounce.enabled == true )
        {
            state->velocity_x = -state->velocity_x;
            state->x_cm = state->min_x_cm;
        }
        else
        {
            state->velocity_x = 0;
            state->x_cm = state->min_x_cm;
        }
    }

    // Otherwise the shield is unimpeded
    else
    {
        state->x_cm = predicted_x;
    }
}



