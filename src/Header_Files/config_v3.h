/*
 * config_3.h
 *
 *  Created on: Apr 12, 2022
 *      Author: nickg
 */

#include <stdbool.h>
#include <stdint.h>
#include "os.h"


#ifndef SRC_HEADER_FILES_CONFIG_3_H_
#define SRC_HEADER_FILES_CONFIG_3_H_


typedef struct BoostTiming_t
{
  OS_TICK  mostRecentPressTime;
  OS_TICK  mostRecentBoostActivationTime;
}BoostTiming_t;


/**
 * @brief Vehicle Direction Structure
 */
typedef struct ShieldPosition_t
{
  float x_cm;

  uint8_t x_pixel_value;

  float max_x_cm;
  float min_x_cm;

  float velocity_x;
  float acceleration_x;
  bool isBoosted;

  float current_force;

  float mass;

}  ShieldPosition_t ;


typedef struct Velocity_t
{
  /**
   * Velocity in the x direction [cm/s]
   */
  float xvel;

  /**
   * Velocity in the y direction [cm/s]
   */
  float yvel;
} Velocity_t;


/**
 * @brief Harkonnen Mass Position type
 */
typedef struct Harkonnen_Mass_Position_t
{

//  uint8_t x_pixel_value;
//  uint8_t y_pixel_value;

  float x_cm;
  float y_cm;

  Velocity_t v;

  float max_x_cm;
  float min_x_cm;

  float max_y_cm;
  float min_y_cm;

  float mass;

  uint16_t numMasses;

} Harkonnen_Mass_Position_t;





typedef struct LaserConfig_t
{
  uint8_t numActivations;
  bool automaticControl;
} LaserConfig_t;



typedef struct BoostConfig_t
{
  float kineticEnergyIncrease;
  uint32_t armingWindowBeforeImpact;
  uint32_t rechargeTimeAfterDisarm;
} BoostConfig_t;


typedef struct HoltzmanShield_t
{
  float exclusivelyPassiveBounceKineticEnergyReduction;

  BoostConfig_t boostConfig;
} HoltzmanShield_t;


typedef struct BounceFromCanyonWalls_t
{
  bool enabled;
  bool limited;

  float maxPlatformBounceSpeed;

} BounceFromCanyonWalls_t;


typedef struct PlatformConfig_t
{
  float maxForce;
  float mass;
  float length;

  BounceFromCanyonWalls_t cw_bounce;

  bool automaticControl;

} PlatformConfig_t;




typedef struct HoltzmanMassesConfig_t
{

  /**
   * Number of HMs which will be dropped
   */
  uint16_t num;

  /**
   * Diameter of the Holtzman [cm]
   */
  float displayDiameter;

  /**
   * @note available for user-defined modes
   */
  uint8_t initialConditions;


  Velocity_t initialVelocity;

  /**
   * Initial Horizontal Position [mm]
   */
  float initialHorizontalPosition;

  uint32_t userDefinedModeInput[8];


} HoltzmanMassesConfig_t;



typedef struct GameConfigurations_v3_t
{
  uint32_t version;

  /**
   * Period the physics gets updated [ms]
   *
   */
  uint32_t tauPhysics;

  /**
   * Period the LCD gets updated [ms]
   */
  uint32_t tauLCD;


  /**
   * Gravity [mm/s^2]
   */
  float gravity;


  /**
   * Size of the Canyon [cm]
   */
  int32_t canyonSize;

  // should be HarkonnenMasses since they're the bad guys
  HoltzmanMassesConfig_t holtzmanMassesConfig;

  PlatformConfig_t platformConfig;

  HoltzmanShield_t shieldConfig;

  LaserConfig_t laserConfig;


} GameConfigurations_v3_t;
#endif /* SRC_HEADER_FILES_CONFIG_3_H_ */
