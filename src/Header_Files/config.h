///**
// * @file config.h
// * @author your name (you@domain.com)
// * @brief
// * @version 0.1
// * @date 2022-04-04
// *
// * @copyright Copyright (c) 2022
// *
// */
//
//#include <stdint.h>
//#include <stdbool.h>
//
//
//#ifndef CONFIG_H
//#define CONFIG_H
//
///**
// * @brief Vehicle Direction Structure
// */
//typedef struct ShieldPosition_t
//{
//  float x_cm;
//
//  uint8_t x_pixel_value;
//
//  float max_x_cm;
//  float min_x_cm;
//
//  float velocity_x;
//  float acceleration_x;
//  bool isBoosted;
//
//  float current_force;
//
//  float mass;
//
//}  ShieldPosition_t ;
//
//
///**
// * @brief Harkonnen Mass Position type
// */
//typedef struct Harkonnen_Mass_Position_t
//{
//
//  uint8_t x_pixel_value;
//  uint8_t y_pixel_value;
//
//  float x_cm;
//  float y_cm;
//
//  Velocity_t v;
//
//  float max_x_cm;
//  float min_x_cm;
//
//  float max_y_cm;
//  float min_y_cm;
//
//  float mass;
//
//  uint16_t numMasses;
//
//} Harkonnen_Mass_Position_t;
//
//
///**
// * @brief Structure containing all of the game configurations
// *
// */
//typedef struct {
//
//    uint8_t version;
//
//    /**
//     * @brief Gravity measured int mm/s^2
//     *
//     */
//    float gravity;
//
//    /**
//     * @brief Size of the canyon measured in cm
//     *
//     */
//    float canyon_size;
//
//    uint8_t tau_phy;
//
//
//    /***********************************************************************
//     * Harkonnen Mass Configurations
//     * ********************************************************************/
//
//    /**
//     * @brief The number of Holtzman Masses to fall before
//     * the battle is won
//     *
//     */
//    uint8_t num_HM;
//
//    /**
//     * @brief The diameter of each HM on the display
//     *
//     */
//    float display_diameter_HM;
//
//    /**
//     * @brief Initial Conditions for HM
//     * @details  [enum: Fixed=0.  127+ are available for user-defined modes]
//     *
//     */
//    uint8_t initial_conditions_HM;
//
//    float initial_velocity_x_HM;
//    float initial_velocity_y_HM;
//
//    float initial_horizontal_position;
//
//    uint8_t user_defined_mode_input;
//
//
//    /******************************************************************************************
//     * Platform Configurations
//     * ***************************************************************************************/
//
//    float mass_force_P;
//    float mass_P;
//    float length_P;
//
//    bool automatic_control_platform;
//
//
//    /*****************************************************************************************
//     * Canyon Wall Bounce Configurations
//     *****************************************************************************************/
//
//    bool cw_bounce_enabled;
//    bool cw_bounce_limited;
//    float max_platform_bounce_speed;
//
//
//    /*****************************************************************************************
//     * Holtzman Shield Configurations
//     * **************************************************************************************/
//    float kinetic_energy_conserved_from_bounce;
//    float kinetic_energy_increase_from_boost;
//    uint16_t arming_window_before_impact_for_boost;
//    uint16_t recharge_time_after_discharge_for_boost;
//
//    /*****************************************************************************************
//     * Laser Configurations
//     * **************************************************************************************/
//    uint16_t num_laser_activations;
//    bool laser_automatic_control;
//
//}GameConfigurations_t ;
//
//
//
//
//#endif
