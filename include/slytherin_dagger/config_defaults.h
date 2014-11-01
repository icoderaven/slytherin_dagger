/** @file config_defaults.h
 *	@brief File to store default values for configuration. This should only
 *	need to be included by config.cpp.
 *
 *	@author	Cornell Wright
 */
#ifndef CONFIG_DEFAULTS_H
#define CONFIG_DEFAULTS_H

#ifndef M_PI
#define M_PI 3.1415926
#endif

/** @brief Which com port we should connect on. */
#define COM_PORT_NAME  "\\\\.\\COM7"

/***************************************************\
* motor 0 = inner snake drive motor                 *
* motor 1 = outer snake drive motor                 *
* motor 2 = inner snake tensioner motor             *
* motor 3 = outer snake bottom tensioner motor      *
* motor 4 = outer snake upper-left tensioner motor  *
* motor 5 = outer snake upper-right tensioner motor *
*													*
* Note that changing this will require changing a	*
* number of the arrays below!!						*
\***************************************************/

#define INNER_SLIDE 0
#define OUTER_SLIDE 1
#define INNER_TENSIONER 2
#define BOT_TENSIONER 3
#define UL_TENSIONER 4
#define UR_TENSIONER 5

/** @brief Starting position in encoder ticks for every motor. */
#define ZERO_POS 0

/** @brief Conversion constants between adc ticks and force */
#define SLIDE_N_PER_LSB 1
#define INNER_N_PER_LSB 1
#define OUTER_N_PER_LSB 1
/** @brief Conversion constant between tg ticks and milliseconds */
#define MS_TO_TG 0.25f

/** @brief Amount of time the user is allowed to steer in continuous mode
 *	in milliseconds
 */
#define CONT_STEERING_TIME 3000


/********************************************************\
* General SIIS Info:                                     *
* -Link length: .834cm for 12mm OD links (slightly less  *
*  than actual length, accounts for link overlap.)       *
* -Linear actuator screw pitch: 3.3mm/revolution         *
* -Linear actuators: 0.172 A/127 mNm                     *
*     Motor #118705:                                     *
*      ·Max. continuous current/torque = 0.172A/5.4mNm   *
*      ·Torque constant = 31.4 mNm/A                     *
*     Gearhead #118185 (29:1 ratio):                     *
*      ·Max. continuous output torque = 150 mNm          *
*      ·Max. efficiency = 81%                            *
*      ·150 mNm/(31.4 mNm/A * 29 * 0.81) = 0.203 A       *
*      ·Max. current is limited by motor, not gearhead   *
* -Inner tensioner:  0.122 A (33.7 lbs on wire)          *
*     Motor #118639:                                     *
*      ·Max. continuous current/torque = 0.238A/3.08mNm  *
*      ·Torque constant = 12.94mNm/A                     *
*     Gearhead #110316 (275:1 ratio):                    *
*      ·Max. continuous output torque = 300mNm           *
*      ·Max. efficiency = 69%                            *
*      ·300mNm/(12.94mNm/A * 275 * 0.69) = 0.122A        *
* -Outer tensioners: 0.197 A (16.8 lbs on wire)          *
*     Motor #256101:                                     *
*      ·Max continuous current/torque = 0.339A/1.55mNm   *
*      ·Torque constant = 4.57mNm/A                      *
*     Gearhead #218418 (256:1 ratio):                    *
*      ·Max continuous output torque = 150mNm            *
*      ·Max. efficiency = 65%                            *
*      ·150mNm/(4.57mNm/A * 256 *0.65) = 0.197A          *
\********************************************************/


//The index of each motor in the outer/inner command structs
const int motor_idx_const[6] = {1, 0, 2, 2, 1, 0};

//The true maximum currents the motors can handle, above which they start breaking
const float motor_max_current_const[6] = {.668f, .668f, .668f, .600f, .600f, .600f};
// The true maximum force the motors can handle, above which they start breaking
const float motor_max_force_const[6] = {622, 622, 622, 558, 558, 558};
// The force limits to use when steering
const float motor_steer_force_const[6] = {622, 622, 622, 558, 558, 558};

// The Actual length of a slide in mm.
// able to make one of the slides move this far without current limiting.
#define SLIDE_LENGTH 375.0f

/*******************************************************************************
 *******************************************************************************
 *	THESE VALUES ARE FOR 12MM SNAKE!!!!!! **************************************
 *******************************************************************************
 ******************************************************************************/
//	#define LINK_RADIUS 6.0f
//	#define STEER_MULTIPLIER 0.3f
//	#define STEER_DIST_FROM_HOME 3.0f
//	#define OUTER_UNLOCK_DIST 4.15f // 4.0f
//	#define INNER_UNLOCK_DIST 6.0f
//	#define OUTER_LOCK_DIST 8.0f
//	#define INNER_LOCK_DIST 10.0f

	#define LINK_RADIUS 6.0f
	#define STEER_MULTIPLIER 0.60f
	#define STEER_DIST_FROM_HOME 4.0f
	#define OUTER_UNLOCK_DIST 4.2f
	#define INNER_UNLOCK_DIST 6.0f
	#define OUTER_LOCK_DIST 8.0f
	#define INNER_LOCK_DIST 8.0f

//	#define LINK_RADIUS 6.0f
//	#define STEER_MULTIPLIER 0.3f
//	#define STEER_DIST_FROM_HOME 3.0f
//	#define OUTER_UNLOCK_DIST 4.0f
//	#define INNER_UNLOCK_DIST 6.0f
//	#define OUTER_LOCK_DIST 8.0f
//	#define INNER_LOCK_DIST 10.0f


	// All TG values are in ms. The new TG values were found for the 12mm
	// snake and should me modified for the 10mm snake
	#define STEER_TG 300
	#define OUTER_LOCK_TG 500 // 3000
	#define OUTER_UNLOCK_TG 275 //500
	#define INNER_LOCK_TG 350 // 500
	#define INNER_UNLOCK_TG 300 // 500
	#define HOME_TG 80000
	#define STEPPED_TG 700 // 800
	#define WITHDRAW_TG 20000
	#define MANUAL_TG 500
	#define RELATIVE_TG 400

	#define OUTER_TG 500
	#define OUTER_TA 62
	#define OTHER_TG 400
	#define OTHER_TA 120
	// The fraction of the true max currents that we'll use as the actual working max
	// None of these numbers should exceed 1.0!
	const float motor_limit_current_fractions_const[6] = {0.8f, 0.7f, 0.8f, 0.38f, 0.38f, 0.38f};

	/*LINK_LENGTH: The length of a snake (outer) link in encoder ticks.
	For old, Garolite snake: 15mm -> 8436
	For Brett's polysulfone snake: 8.34mm -> 4690 */
	#define LINK_LENGTH 7.973f


/* Do not change: these numbers represent which direction is positive for each motor,
 * where positive is defined as extending the snake or increasing the tension.
 */
const char motor_direction_const[6] = {1, 1, 1, -1, -1, -1};

/* AMPS_PER_LSB: The number of amperes represented by one LSB of the ADC.
 * This is a useful constant to know, but not used anywhere in the code anymore
 */
//#define AMPS_PER_LSB 0.000322265

/* CLOSE_ENOUGH: The acceptable distance or force between two values for
 * them to be considered "equal"
 */
#define CLOSE_ENOUGH_DIST 1.0f /* mm */

#define CLOSE_ENOUGH_FORCE 20 /* N */

/* STAGGER: Equal to the inner snake's distance when its end is flush
 * with the outer snake's (when the outer snake is at his home position) minus the
 * inner snake's distance when it is in its home position.  User can enter other
 * values for stagger in the config file
 */
#define STAGGER 0.0f

/* HOMING_DELAY_COUNT: Number of cycles that linear actuator motors must be at
 * max_current before it's decided that this position will be the home_position
 */
#define HOMING_DELAY_COUNT 2

/* NUM_MODES = the number of different operating modes the system has
 * Note that this is only defined here (not in the config file) because it
 * has to be constant at compile time.
 */
#define NUM_MODES 18

/* MANUAL_STEP_INCR: the distance the manual slides should be moved by the
 * arrow keys.
 */
#define MANUAL_STEP_INCR 0.1f

/* MANUAL_PAGE_INCR: the distance the manual slides should be moved by the
 * page-up page-down keys.
 */
#define MANUAL_PAGE_INCR 1.0f

/** JOY_DEADZONE: percentage deadzone for steering (must be in range [-1 1] */
#define JOY_DEADZONE 0.10f

#endif /* CONFIG_DEFAULTS_H */


