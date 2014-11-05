/** @file guided_motions.cpp
 *	@brief Funciton definitions for the guided motions component.
 *
 *	@author Cornell Wright
 */

#include <stdio.h>
#include <math.h>
#include <Windows.h>
#include "motor_abs.h"
#include "guided_motions.h"
#include "config_defaults.h"
#include "log.h"
#include "conio.h"
#include "CardioError.h"
#include "ErrorData.h"
#include "superCommon.h"


/** @brief The number of motors */
#define NUM_MOTORS 6

/** @brief Holds information for us about each motor */
static struct {
	float cmd_pos;
	float force_lim;
	float max_force_lim;
	float home_pos;
	float home_unlock_pos;
} mot[NUM_MOTORS];

static int mot_cmd_seq_numbers[NUM_MOTORS];

#define INNER 0
#define OUTER 1 

/** @brief Whether the inner or outer snake is locked or locking, and how
 *	many times it has been locked/unlocked in test mode
 */
static struct snake_info {
	int locked, locking;
	int unlocked, unlocking;
	int steering, setting_steer;
	int homed, homing;
	int test_lock_count;
} snake[2];


/** @brief What mode guided motions is in right now. Globally visible to
 *	provide status. No other parts of the software should write to this
 *	variable.
 */
int guided_mode;

/** @brief Tells whether or not we are paused. Globally visible to provide
 *	status. No other parts of the software should write to this variable.
 */
int guided_paused;

/** @brief Separation distance between the inner and outer snake. Used
 *	when steering the snake.
 */
float guided_separation;

/** @brief What mode to change to. -1 if we are not changing modes. */
int change_mode = -1;


// Static function prototypes
static void lock_snake(int which);
static void unlock_snake(int which);
static void set_steering(void);
static void home_slide(int which);
static int is_goal_reached(int motor);
static int set_position_save(int motor, float dest_pos, float tg);
static int set_force_lim_save(int motor, float force);

static FILE *guided_mode_log_file;
FILE * guided_motions_log_init( char*fname )
{
	FILE *log_file = NULL;
	fopen(&log_file, fname, "a");
	fprintf(log_file, "NewMode, InnerSlide, OuterSlide, InnerTensioner, ULTensioner, URTensioner, BTensioner \n");
	return (log_file);
}
/** @brief Initializes guided mode. Does not actually move the probe.
 */
void guided_motions_init(void) {
	int i;

	for (i = 0; i < NUM_MOTORS; i++) {
		// Set all motors' command position to their current position
		mot[i].cmd_pos = get_position(i);
		// Set the force limits given in the config file
		mot[i].max_force_lim = motor_max_force_const[i] *
				motor_limit_current_fractions_const[i];
		// Tell the motor what its force limit is
		set_force_lim_save(i, mot[i].max_force_lim);
		// Set the home pos to their current positionf or now, since we
		// don't know where anything is
		mot[i].home_pos = mot[i].cmd_pos;
	}

	// Since we don't know positions of any parts of the snake, we can't say
	// that it's locked, steering, etc.
	memset((void *)&snake[INNER], 0, sizeof(struct snake_info));
	memset((void *)&snake[OUTER], 0, sizeof(struct snake_info));

	// Initialized to initializing state. Automatically switches to homing
	// when unpaused
	guided_mode = INITIALIZING_INIT;
	guided_paused = 1;

	// Set the default separation for steering to be one link.
	guided_separation = LINK_LENGTH;

	// Set change to homing mode because if the user unpauses we want to
	// act like we're in a new mode
	change_mode = HOMING_SCREWS;
	guided_mode_log_file = guided_motions_log_init("mode_change_cmd.csv");
}

/**	@brief Reinitializes guided mode. This is necessary if the user switches
 *	to manual mode and back to guided mode.
 */
void guided_motions_reinit(void) {
	int i;

	if (guided_mode < 0) {
		pause();
	}

	for (i = 0; i < NUM_MOTORS; i++) {
		// Set all motors' command position to their current position
		mot[i].cmd_pos = get_position(i);
		// Also set the home position to the current position
		mot[i].home_pos = mot[i].cmd_pos;
		// Set the force limits to whatever they were before
		set_force_lim(i, mot[i].force_lim);
	}


}


/** @brief Changes the current mode.
 *
 *	@param new_mode The new mode to switch to. See guided_motions.h for valid
 *	constants.
 */
void set_mode(int new_mode) {

	// Ignore calls to set_mode() if we're not paused.
	// Requirement 8.3.1
	if ((!guided_paused) || (ErrorState != RUN_STATE && new_mode != ABANDON)){
		return;
	}

	// First make sure the slides aren't still moving
	// Requirement 8.3.2
	set_position_save(INNER_SLIDE,
			get_position(INNER_SLIDE), 0);
	set_position_save(OUTER_SLIDE,
			get_position(OUTER_SLIDE), 0);
	mot_apply();

	// Sets our new mode based upon our current mode and the requested new
	// mode. In some cases it is not valid to switch to certain modes. For
	// example you can only switch to REINSERT_INNER from WITHDRAW_INNER.
	// Requirement 8.3.3 (this whole switch statement)
	switch (guided_mode) {
		case INITIALIZING_INIT:
			switch (new_mode) {
				case HOMING: change_mode= HOMING_SCREWS; break;
				case STEPPED: break;
				case RETRACT: break;
				case ABANDON: break;
				default: break;
			}
			break;
		case HOMING_SCREWS:
			switch (new_mode) {
				case HOMING:
					if (change_mode >= 0) {
						change_mode = HOMING_SCREWS;
					}
					break;
				case STEPPED: change_mode = STEPPED_ADVANCE_OUTER; break;
				case RETRACT: change_mode = RETRACT_INNER; break;
				case ABANDON: break;
				default: break;
			}

			if (new_mode != HOMING)
			{
				set_position_tracking(INNER_SLIDE, 1);
				set_position_tracking(OUTER_SLIDE, 1);
			}

			break;
		case STEPPED_STEERING:
			switch (new_mode) {
				case HOMING: change_mode = HOMING_SCREWS; break;
				case STEPPED:
					if (change_mode >= 0 &&
							change_mode != STEPPED_ADVANCE_INNER &&
							change_mode != STEPPED_ADVANCE_OUTER) {
						change_mode = STEPPED_STEERING;
					}
					break;
				case RETRACT: change_mode = RETRACT_STEERING; break;
				case ABANDON: change_mode = ABANDON_ABANDONING; break;
				default: break;
			}
			break;
		case STEPPED_ADVANCE_INNER:
			switch (new_mode) {
				case HOMING: change_mode = HOMING_SCREWS; break;
				case STEPPED:
					if (change_mode >= 0 &&
							change_mode != STEPPED_STEERING &&
							change_mode != STEPPED_ADVANCE_OUTER) {
						change_mode = STEPPED_ADVANCE_INNER;
					}
					break;
				case RETRACT: change_mode = RETRACT_INNER; break;
				case ABANDON: change_mode = ABANDON_ABANDONING; break;
				default: break;
			}
			break;
		case STEPPED_ADVANCE_OUTER:
			switch (new_mode) {
				case HOMING: change_mode = HOMING_SCREWS; break;
				case STEPPED:
					if (change_mode >= 0 &&
							change_mode != STEPPED_STEERING &&
							change_mode != STEPPED_ADVANCE_INNER) {
						change_mode = STEPPED_ADVANCE_OUTER;
					}
					break;
				case RETRACT: change_mode = RETRACT_OUTER; break;
				case ABANDON: change_mode = ABANDON_ABANDONING; break;
				default: break;
			}
			break;
		case RETRACT_INNER:
			switch (new_mode) {
				case HOMING: change_mode = HOMING_SCREWS; break;
				case STEPPED: change_mode = STEPPED_ADVANCE_OUTER; break;
				case RETRACT:
					if (change_mode >= 0 && change_mode != RETRACT_OUTER) {
						change_mode = RETRACT_INNER;
					}
					break;
				case ABANDON: change_mode = ABANDON_ABANDONING; break;
				default: break;
			}
			break;
		case RETRACT_OUTER:
			switch (new_mode) {
				case HOMING: change_mode = HOMING_SCREWS; break;
				case STEPPED: change_mode = STEPPED_ADVANCE_OUTER; break;
				case RETRACT:
					if (change_mode >= 0 && change_mode != RETRACT_INNER) {
						change_mode = RETRACT_OUTER;
					}
					break;
				case ABANDON: change_mode = ABANDON_ABANDONING; break;
				default: break;
			}
			break;
		case RETRACT_STEERING:
			switch(new_mode) {
				case HOMING: change_mode = HOMING_SCREWS; break;
				case STEPPED: change_mode = STEPPED_STEERING; break;
				case RETRACT:
					if (change_mode >= 0 && change_mode != RETRACT_OUTER) {
						change_mode = RETRACT_STEERING;
					}
					break;
				case ABANDON: change_mode = ABANDON_ABANDONING; break;
				default: break;
			}
		case ABANDON_ABANDONING:
			switch (new_mode) {
				case HOMING: change_mode = HOMING_SCREWS; break;
				case STEPPED: change_mode = STEPPED_ADVANCE_OUTER; break;
				case RETRACT: change_mode = RETRACT_INNER; break;
				case ABANDON: break;
				default: break;
			}
			break;
		default:
            SetErrorBit(M_DCU,DCU_ERR_STATE_TRANS);
			log_printf(LOG_ERROR, "Invalid mode number %d at line %d in %s.\n",
					guided_mode, __LINE__, __FILE__);
			break;
	}
}


/** @brief Pauses the motion immediately. */
void pause(void) {

	// Tell all the motors to pause
	// Requirement 8.1
	pause_motor(INNER_SLIDE, 1);
	pause_motor(OUTER_SLIDE, 1);
	pause_motor(INNER_TENSIONER, 1);
	pause_motor(BOT_TENSIONER, 1);
	pause_motor(UL_TENSIONER, 1);
	pause_motor(UR_TENSIONER, 1);
	mot_apply();

	// Show that we're paused
	guided_paused = 1;

}

/** @brief Unpauses and resumes motion. */
void unpause(void) {
	// Ignore if we're not paused
	// Requirement 8.2.1
	if ((guided_paused) && (ErrorState == RUN_STATE)){

		// Tell all the motors to go wherever thery were going before we
		// paused.

		// Only move the slides if we're not switching modes since they will
		// just be commanded to stop anyway.
		// Requirement 8.2.2.1
		if (change_mode >= 0) {
			set_position(INNER_SLIDE,
					get_position(INNER_SLIDE), 0);
			set_position(OUTER_SLIDE,
					get_position(OUTER_SLIDE), 0);
		}

		// Unpause all the motors and show we're not paused
		// Requirement 8.2.2.2
		guided_paused = 0;
		pause_motor(INNER_SLIDE, 0);
		pause_motor(OUTER_SLIDE, 0);
		pause_motor(INNER_TENSIONER, 0);
		pause_motor(BOT_TENSIONER, 0);
		pause_motor(UL_TENSIONER, 0);
		pause_motor(UR_TENSIONER, 0);
		mot_apply();
	}

}

/** @brief Moves to the next step in a stepped mode. */
void execute(void) {
	// Make sure we're not paused and that we're in the correct mode
	// Requirement 8.12.1
	if (guided_paused || (guided_mode != STEPPED_STEERING &&
			guided_mode != RETRACT_STEERING)) {
		return;
	}

	// Switch into the advancing or retracting stage
	if (guided_mode == STEPPED_STEERING) {
		change_mode = STEPPED_ADVANCE_INNER;
	}
	else if (guided_mode == RETRACT_STEERING) {
		change_mode = RETRACT_OUTER;
	}

}

/** @brief Steers the head of the snake when in the appropriate mode.
 *
 *	@param x The amount to steer in the x direction (between -1 and 1
 *	inclusive).
 *
 *	@param y The amount to steer in the y direction (between -1 and 1
 *	inclusive).
 */
void steer(float x, float y) {
	float rad, theta;

	// Mkae sure we're not paused or changing modes and that we are in one
	// of the steering modes.
	// Requirement 8.10.1
	if (!guided_paused && change_mode == -1 &&
			(guided_mode == STEPPED_STEERING ||
			guided_mode == RETRACT_STEERING)) {
			
		// Convert to polar coordinates
		// Requirement 8.10.2
		rad = sqrt(x*x + y*y);
		theta = atan2(y, x);

		// Make sure we're within a circle of radius 1
		// Requirement 8.10.3
		if (rad > 1) {
			rad = 1;
		}

		// Set each motor to the appropriate distance to point the head at
		// that angle
		// Requirement 8.10.4
		set_position_save(BOT_TENSIONER,
				mot[BOT_TENSIONER].home_pos -
				STEER_DIST_FROM_HOME + (-rad * (float)sin(theta))
				* STEER_MULTIPLIER * guided_separation,
				STEER_TG);
		// Requirement 8.10.5
		set_position_save(UL_TENSIONER,
				mot[UL_TENSIONER].home_pos -
				STEER_DIST_FROM_HOME + (-rad * (float)cos(theta +
				(float)M_PI/6.0)) * STEER_MULTIPLIER *
				guided_separation, STEER_TG);
		// Requirement 8.10.6
		set_position_save(UR_TENSIONER,
				mot[UR_TENSIONER].home_pos -
				STEER_DIST_FROM_HOME + (rad * (float)cos(theta -
				(float)M_PI/6.0)) * STEER_MULTIPLIER *
				guided_separation, STEER_TG);
		mot_apply();

	}

}


/** @brief Changes the separation distance. - This affects how much snake
 *	is extended when steering.
 *
 *	@param dist The separation distance in mm.
 */
void set_separation(float dist) {
	// Requirement 8.11
	guided_separation = dist;
}


/** @brief Locks the snake specified.
 *
 *	@param which Which snake to lock - either INNER or OUTER.
 */
static void lock_snake(int which) {
	float force_inner;

	// Return if the snake is already locked.
	// Requirements 8.4.1 and 8.5.1
	if (snake[which].locked) {
		return;
	}

	switch (which) {
		case INNER:
			// If we're already locking the inner snake, see if we're drawing
			// enough current to mean it's locked yet
			if (snake[INNER].locking) {
				force_inner = get_force(INNER_TENSIONER);
				if (force_inner >
						mot[INNER_TENSIONER].force_lim - 
						CLOSE_ENOUGH_FORCE) {
					// Show that the snake is now locked
					// Requirement 8.5.5
					snake[INNER].locked = 1;
					snake[INNER].locking = 0;
					// Reset the home position
					printf("Set inner home pos to: %f\n",
					mot[INNER_TENSIONER].home_pos =
							get_position(INNER_TENSIONER));
				}
				// Check to see if we've reached our goal. If so, then we
				// aren't reeling in enough cable to lock the snake, so
				// reel in some more. Also log a warning if we're not homing,
				// since this may indicate something wrong with the probe.
				else {
				
					if (is_goal_reached(INNER_TENSIONER)) {
						if (guided_mode != HOMING_SCREWS) {
                            SetErrorBit(M_DCU,DCU_ERR_FORCE_LOCK);
							log_printf(LOG_WARNING, "Inner snake not locked, "
									"but goal position was was reached. Goal = "
									"%f. Inner lock dist = %f.\n",
									mot[INNER_TENSIONER].cmd_pos,
									INNER_LOCK_DIST);
						}
						set_position_save(INNER_TENSIONER,
								mot[INNER_TENSIONER].cmd_pos +
								INNER_LOCK_DIST,
								INNER_LOCK_TG);
						mot_apply();

					}
				}
			}
			else {
				// Show that we're locking the inner snake
				// Requirement 8.5.3
				snake[INNER].locking = 1;
				snake[INNER].unlocked = 0;
				snake[INNER].unlocking = 0;
				snake[INNER].steering = 0;
				snake[INNER].setting_steer = 0;

				// Set the appropriate force limit
				// Requirement 8.5.2
				set_force_lim_save(INNER_TENSIONER,
						mot[INNER_TENSIONER].max_force_lim);
				set_position_tracking(INNER_TENSIONER,0);  // not tracking
				set_lock_indicator(INNER_TENSIONER,1);  // locking
				// Set the inner tensioner to reel in enough cable to cause the
				// snake to lock
				// Requirement 8.5.4
				set_position_save(INNER_TENSIONER,
						mot[INNER_TENSIONER].cmd_pos + 
						INNER_LOCK_DIST,
						INNER_LOCK_TG);
				mot_apply();
			}

			break;


		case OUTER:
			// If we're already locking the outer snake, see if it's locked
			// yet
			if (snake[OUTER].locking) {
				if (get_force(BOT_TENSIONER) >
						mot[BOT_TENSIONER].force_lim -
						CLOSE_ENOUGH_FORCE
						&& get_force(UL_TENSIONER) >
						mot[UL_TENSIONER].force_lim -
						CLOSE_ENOUGH_FORCE
						&& get_force(UR_TENSIONER) >
						mot[UR_TENSIONER].force_lim - 
						CLOSE_ENOUGH_FORCE) {
					// Show that the snake is now locked
					// Requirement 8.4.5
					snake[OUTER].locked = 1;
					snake[OUTER].locking = 0;
					// Reset home positions for all motors
					mot[BOT_TENSIONER].home_pos =
							get_position(BOT_TENSIONER);
					mot[UL_TENSIONER].home_pos =
							get_position(UL_TENSIONER);
					mot[UR_TENSIONER].home_pos =
							get_position(UR_TENSIONER);
					// Command all the motors to the same distance beyond
					// their home position so that they have the same trajectory
					set_position_save(BOT_TENSIONER,
							mot[BOT_TENSIONER].home_pos +
							OUTER_LOCK_DIST,
							OUTER_LOCK_TG);
					set_position_save(UL_TENSIONER,
							mot[UL_TENSIONER].home_pos +
							OUTER_LOCK_DIST,
							OUTER_LOCK_TG);
					set_position_save(UR_TENSIONER,
							mot[UR_TENSIONER].home_pos +
							OUTER_LOCK_DIST,
							OUTER_LOCK_TG);
				}
				// If we're not locked, make sure no motors have reached
				// their goal. If any have, we must command all motors to
				// reel in more cable. We should also log a warning as this
				// may indicate that something is wrong with the snake unless
				// we are homing.
				else if (is_goal_reached(BOT_TENSIONER) ||
						is_goal_reached(UL_TENSIONER) ||
						is_goal_reached(UR_TENSIONER)) {
					if (guided_mode != HOMING_SCREWS) {
						// Check the bottom tensioner
						if (is_goal_reached(BOT_TENSIONER)) {
                            SetErrorBit(M_DCU,DCU_ERR_FORCE_LOCK);                                              
							log_printf(LOG_WARNING, "Outer snake not locked, "
									"but goal position was was reached on bot "
									"tensioner. Goal = %f. Outer lock "
									"dist = %f.\n",
									mot[BOT_TENSIONER].cmd_pos,
									OUTER_LOCK_DIST);
						}
						// Check the upper left tensioner
						if (is_goal_reached(UL_TENSIONER)) {
                             SetErrorBit(M_DCU,DCU_ERR_FORCE_LOCK);                                          
							log_printf(LOG_WARNING, "Outer snake not locked, "
									"but goal position was was reached on ul "
									"tensioner. Goal = %f. Outer lock "
									"dist = %f.\n",
									mot[UL_TENSIONER].cmd_pos,
									OUTER_LOCK_DIST);
						}
						// Check the upper right tensioner
						if (is_goal_reached(UR_TENSIONER)) {
                            SetErrorBit(M_DCU,DCU_ERR_FORCE_LOCK);                                          
                            log_printf(LOG_WARNING, "Outer snake not locked, "
									"but goal position was was reached on ur "
									"tensioner. Goal = %f. Outer lock "
									"dist = %f.\n",
									mot[UR_TENSIONER].cmd_pos,
									OUTER_LOCK_DIST);
						}
					}
					// Bring all the tensioners further back
					set_position_save(BOT_TENSIONER,
							mot[BOT_TENSIONER].cmd_pos +
							OUTER_LOCK_DIST,
							OUTER_LOCK_TG);
					set_position_save(UL_TENSIONER,
							mot[UL_TENSIONER].cmd_pos +
							OUTER_LOCK_DIST,
							OUTER_LOCK_TG);
					set_position_save(UR_TENSIONER,
							mot[UR_TENSIONER].cmd_pos +
							OUTER_LOCK_DIST,
							OUTER_LOCK_TG);
					mot_apply();
				}

			}
			else {
				// Show that we're locking the snake
				// Requirement 8.4.3
				snake[OUTER].locking = 1;
				snake[OUTER].unlocked = 0;
				snake[OUTER].unlocking = 0;
				snake[OUTER].steering = 0;
				snake[OUTER].setting_steer = 0;
				// Set all force limits
				// Requirement 8.4.2
				set_force_lim_save(BOT_TENSIONER,
						mot[BOT_TENSIONER].max_force_lim);
				set_force_lim_save(UL_TENSIONER,
						mot[UL_TENSIONER].max_force_lim);
				set_force_lim_save(UR_TENSIONER,
						mot[UR_TENSIONER].max_force_lim);
				// Set all three tensioners to reel in enough cable to cause the
				// snake to lock
				// Requirement 8.4.4
				set_position_save(BOT_TENSIONER,
						mot[BOT_TENSIONER].cmd_pos +
						OUTER_LOCK_DIST,
						OUTER_LOCK_TG);
				set_position_tracking(BOT_TENSIONER,0);  // bottom not tracking
				set_lock_indicator(BOT_TENSIONER,1);  // bottom locking

				set_position_save(UL_TENSIONER,
						mot[UL_TENSIONER].cmd_pos +
						OUTER_LOCK_DIST,
						OUTER_LOCK_TG);
				set_position_tracking(UL_TENSIONER,0);  // ul not tracking
				set_lock_indicator(UL_TENSIONER,1);  // ul locking

				set_position_save(UR_TENSIONER,
						mot[UR_TENSIONER].cmd_pos +
						OUTER_LOCK_DIST,
						OUTER_LOCK_TG);
				set_position_tracking(UR_TENSIONER,0); // ur not tracking
				set_lock_indicator(UR_TENSIONER,1);  // ur locking
				mot_apply();
			}

			break;

		default:
             SetErrorBit(M_DCU,DCU_ERR_INDEX_SNAKE);
     		log_printf(LOG_ERROR, "Invalid snake number %d specified at line "
					"%d in %s.\n", which, __LINE__, __FILE__);
			break;
	}
}

/** @brief Unlocks the specified snake
 *
 *	@param which Which snake to unlock - either INNER or OUTER
 */
static void unlock_snake(int which) {

	// If the snake is already locked, just return
	// Requirements 8.6.1 and 8.7.1
	if (snake[which].unlocked) {
		return;
	}

	switch (which) {
		case INNER:

			if (snake[INNER].unlocking) {
				// If we've already started unlocking, just check and see if
				// we've let out enough cable
				// Requirement 8.7.4
				if (is_goal_reached(INNER_TENSIONER)) {
					snake[INNER].unlocked = 1;
					snake[INNER].unlocking = 0;
		            set_position_tracking(INNER_TENSIONER,1);  // start tracking once unlock is finished
				}
			}
			else {
				// Otherwise, command the snake to unlock
				// Requirements 8.7.2 and 8.7.3
				snake[INNER].unlocking = 1;
				snake[INNER].locking = 0;
				snake[INNER].locked = 0;
				snake[INNER].steering = 0;
				snake[INNER].setting_steer = 0;
				set_force_lim_save(INNER_TENSIONER, 
						mot[INNER_TENSIONER].max_force_lim);
				set_position_save(INNER_TENSIONER,
						get_position(INNER_TENSIONER) -
						INNER_UNLOCK_DIST,
						INNER_UNLOCK_TG);
	            set_lock_indicator(INNER_TENSIONER, 0);  // not locked or locking
				mot_apply();
			}
			break;
		case OUTER:
			if (snake[OUTER].unlocking) {
				// If we've already started unlocking, just see if we've reached
				// our goal.
				if (is_goal_reached(BOT_TENSIONER) &&
						is_goal_reached(UL_TENSIONER) &&
						is_goal_reached(UR_TENSIONER)) {
					// Requirement 8.6.4
					snake[OUTER].unlocked = 1;
					snake[OUTER].unlocking = 0;

		            set_position_tracking(BOT_TENSIONER, 1);
		            set_position_tracking(UL_TENSIONER, 1);
		            set_position_tracking(UR_TENSIONER, 1);
				}
			}
			else {
				// Otherwise, command the snake to unlock
				// Requirements 8.6.2 and 8.6.3
				snake[OUTER].unlocking = 1;
				snake[OUTER].locking = 0;
				snake[OUTER].locked = 0;
				snake[OUTER].steering = 0;
				snake[OUTER].setting_steer = 0;

				// Set all the force limits so that we know the motors can move
				set_force_lim_save(BOT_TENSIONER, 
						mot[BOT_TENSIONER].max_force_lim);
				set_force_lim_save(UL_TENSIONER, 
						mot[UL_TENSIONER].max_force_lim);
				set_force_lim_save(UR_TENSIONER, 
						mot[UR_TENSIONER].max_force_lim);
				set_lock_indicator(BOT_TENSIONER,0);  // not locked or locking
				set_lock_indicator(UL_TENSIONER,0);  // not locked or locking
				set_lock_indicator(UR_TENSIONER,0);  // not locked or locking
				// When we're not homing, we want to unlock a fixed distance
				if (change_mode != HOMING_SCREWS) {
					// Unlock the bottom tensioner
					set_position_save(BOT_TENSIONER,
							mot[BOT_TENSIONER].home_pos -
							OUTER_UNLOCK_DIST,
							OUTER_UNLOCK_TG);

					// Unlock the upper left tensioner
					set_position_save(UL_TENSIONER,
							mot[UL_TENSIONER].home_pos -
							OUTER_UNLOCK_DIST,
							OUTER_UNLOCK_TG);

					// Unlock the upper right tensioner
					set_position_save(UR_TENSIONER,
							mot[UR_TENSIONER].home_pos -
							OUTER_UNLOCK_DIST,
							OUTER_UNLOCK_TG);
				}
				// Otherwise, we want to go to the position we started at
				else {
					// Unlock the bottom tensioner
					if (mot[BOT_TENSIONER].home_pos >
							mot[BOT_TENSIONER].home_unlock_pos) {
						set_position_save(BOT_TENSIONER,
								mot[BOT_TENSIONER].home_unlock_pos
								- OUTER_UNLOCK_DIST,
								OUTER_UNLOCK_TG);
					}
					else {
						set_position_save(BOT_TENSIONER,
								mot[BOT_TENSIONER].home_pos -
								OUTER_UNLOCK_DIST,
								OUTER_UNLOCK_TG);
					}

					// Unlock the upper left tensioner
					if (mot[UL_TENSIONER].home_pos >
							mot[UL_TENSIONER].home_unlock_pos) {
						set_position_save(UL_TENSIONER,
								mot[UL_TENSIONER].home_unlock_pos
								- OUTER_UNLOCK_DIST,
								OUTER_UNLOCK_TG);
					}
					else {
						set_position_save(UL_TENSIONER,
								mot[UL_TENSIONER].home_pos -
								OUTER_UNLOCK_DIST,
								OUTER_UNLOCK_TG);
					}
					// Unlock the upper right tensioner
					if (mot[UR_TENSIONER].home_pos >
							mot[UR_TENSIONER].home_unlock_pos) {
						set_position_save(UR_TENSIONER,
								mot[UR_TENSIONER].home_unlock_pos
								- OUTER_UNLOCK_DIST,
								OUTER_UNLOCK_TG);
					}
					else {
						set_position_save(UR_TENSIONER,
								mot[UR_TENSIONER].home_pos -
								OUTER_UNLOCK_DIST,
								OUTER_UNLOCK_TG);
					}
				}
				mot_apply();
			}
			break;
		default:
            SetErrorBit(M_DCU,DCU_ERR_INDEX_SNAKE);
			log_printf(LOG_ERROR, "Invalid snake number %d specified at line "
					"%d in %s.\n", which, __LINE__, __FILE__);
			break;
	}
}

/** @brief Sets the outer snake to the appropriate tension for steering */
static void set_steering(void)
{
	int i;

	// See if we're set to steer already and just return if we are
	// Requirement 8.9.1
	if (snake[OUTER].steering) {
		return;
	}

	// If we've already commanded the outer snake to go to steer tension,
	// just check to see if we've arrived yet
	// Requirement 8.9.4
	if (snake[OUTER].setting_steer) {
		if (is_goal_reached(BOT_TENSIONER) &&
				is_goal_reached(UL_TENSIONER) &&
				is_goal_reached(UR_TENSIONER)) {
			snake[OUTER].setting_steer = 0;
			snake[OUTER].steering = 1;
		}
	}
	// Otherwise command the tensioners to go to the steering distance
	else {
		// Requirement 8.9.3
		snake[OUTER].setting_steer = 1;
		snake[OUTER].locked = 0;
		snake[OUTER].locking = 0;
		snake[OUTER].unlocked = 0;
		snake[OUTER].unlocking = 0;

		// Set all the force limits so that we know the motors can move
		// Requirement 8.9.2
		for (i = 0; i < NUM_MOTORS; i++) {
			set_force_lim_save(i, motor_steer_force_const[i] *
				motor_limit_current_fractions_const[i]);
		}

		// Move the bottom tensioner
		set_position_save(BOT_TENSIONER,
				mot[BOT_TENSIONER].home_pos -
				STEER_DIST_FROM_HOME,
				STEER_TG);

		// Move the upper left tensioner
		set_position_save(UL_TENSIONER,
				mot[UL_TENSIONER].home_pos -
				STEER_DIST_FROM_HOME,
				STEER_TG);

		// Move the upper right tensioner
		set_position_save(UR_TENSIONER,
				mot[UR_TENSIONER].home_pos -
				STEER_DIST_FROM_HOME,
				STEER_TG);
		mot_apply();
	}


}

/** @brief Brings the requested slide back until it hits a current limit.
 *
 *	@param which Which slide to home (either INNER or OUTER)
 */
static void home_slide(int which) {
	int motor_num;

	// Double check that we have a valid snake number. If not throw and error
	// in the log and return.
	if (which != INNER && which != OUTER) {
         SetErrorBit(M_DCU,DCU_ERR_INDEX_SNAKE);
		log_printf(LOG_ERROR, "Invalid snake number %d specified at line %d in "
				"%s.\n", which, __LINE__, __FILE__);
		return;
	}

	// If this snake is already homed, don't do anything.
	// Requirement 8.8.1
	if (snake[which].homed) {
		return;
	}

	// Figure out the motor number associated with the given slide
	motor_num = (which == INNER) ? INNER_SLIDE :
			OUTER_SLIDE;

	// Check to see if we're already homing
	if (snake[which].homing) {
		// If we are and we've already reached our desired current limit
		// then we're done

		if (get_limit_sw(motor_num)) {

			// Show we're home
			// Requirement 8.8.4
			snake[which].homed = 1;
			snake[which].homing = 0;

			// Set the position to this so it will stop pushing so hard on
			// the end of the feeder
			set_position_save(motor_num, get_position(motor_num), 0);
			set_position_tracking(motor_num, 0);  // turn off tracking for the slide motor
			mot_apply();
		}
		// Otherwise, double check that we haven't reached our goal (it should
		// be impossible to get there)
		else if (is_goal_reached(motor_num)) {
			// If we did, log a warning, since this is probably just caused
			// by a constant getting set too small, but may also indicate
			// snake trouble
			SetErrorBit(M_DCU,DCU_ERR_FORCE_LOCK );
			log_printf(LOG_WARNING, "Goal reached on slide %d when should have "
			"hit current limit. Goal = %f. Slide Length = %f.\n", motor_num,
			mot[motor_num].cmd_pos, SLIDE_LENGTH);

			// Then try going further.
			set_position_save(motor_num, mot[motor_num].cmd_pos -
					SLIDE_LENGTH,
					HOME_TG);
			mot_apply();
		}
	}
	else {
		// If we haven't started homing yet, show that we are
		// Requirement 8.8.3
		snake[which].homing = 1;

		// Set our force limit so that we know we can move
		// Requirement 8.8.2
		set_force_lim_save(motor_num, mot[motor_num].max_force_lim);
		// Tell the slide to go back further than should be possible so that
		// we will hit a current limit
		set_position_save(motor_num, mot[motor_num].cmd_pos -
				SLIDE_LENGTH,
				HOME_TG);
		mot_apply();
	}
}


/** @brief Checks to see if a motors actual position is close enough to its
 *	commanded position to say that it's there.
 *
 *	@param motor which motor to check.
 */
static int is_goal_reached(int motor) {
	// fabs() is math.h function for floating point absolute value
	float pos;
	int to_ret;

	pos = get_position(motor);
	to_ret = (fabs(pos - mot[motor].cmd_pos) <=
			CLOSE_ENOUGH_DIST) ? 1 : 0;


	return to_ret;
}

/** @brief Wrapper which saves the commanded position before calling
 *	set_position().
 *
 *	@param motor Which motor to change the position o
 *	@param dest_pos Desination position for that motor
 *
 *	@return Whatever set_position() returns
 */
static int set_position_save(int motor, float dest_pos, float tg)
{
	mot[motor].cmd_pos = dest_pos;
	return set_position(motor, dest_pos, tg);
}

/** @brief Wrapper which saves the commanded force limit before calling
 *	set_force_lim().
 *
 *	@param motor Which motor to change the force limit
 *	@param force Desired force limit
 *
 *	@return Whatever set_force_lim returns
 */
static int set_force_lim_save(int motor, float force)
{
	mot[motor].force_lim = force;
	return set_force_lim(motor, force);
}


/** @brief Main loop for guided motions. This should be run often as it is
 *	what controls the snake in guided mode.
 */
void guided_motions_main_loop(void)
{
	float force_inner;
	float DesiredPosition;            //jwm - added for range check

	// If we are paused, don't do anything
	// Requirement 8.13.1
	if ((guided_paused) || (ErrorState != RUN_STATE)){
		return;
	}

	// See if we're switching between modes
	if (change_mode >= 0) {

		// If so, do whatever we need to do to switch into the new mdoe
		switch (change_mode) {
			case HOMING_SCREWS:
				// Not homed or homing yet
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// When switching to Homing mode, we need both snakes unlocked.
				// Then we begin moving both slides all the way back
				// Requirement 8.13.2.1.1
				unlock_snake(INNER);
				unlock_snake(OUTER);

				if (! (snake[INNER].unlocked && snake[OUTER].unlocked)) {
					return;
				}

				// Set all force limits to 0 while we home so that backdriving
				// is possible.
				// Requirement 8.13.2.1.2 (note change_mode is set after
				// the switch statement)
				set_force_lim_save(INNER_TENSIONER, 0.0f);
				set_position_tracking(INNER_TENSIONER,0);  // not tracking
				set_force_lim_save(BOT_TENSIONER, 0.0f);
				set_position_tracking(BOT_TENSIONER,0);  // not tracking
				set_force_lim_save(UL_TENSIONER, 0.0f);
				set_position_tracking(UL_TENSIONER,0);  // not tracking
				set_force_lim_save(UR_TENSIONER, 0.0f);
				set_position_tracking(UR_TENSIONER,0);  // not tracking
				set_position_tracking(INNER_SLIDE,0);
				set_position_tracking(INNER_SLIDE,0);

				break;

			case STEPPED_STEERING:
				set_position_tracking(INNER_SLIDE,1);
				set_position_tracking(OUTER_SLIDE,1);
				// Not homed or homing
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// When switching to stepped steering, lock the inner snake
				// and set the outer snake to the steering tension.
				// Requirement 8.13.2.2.1
				lock_snake(INNER);
				if (! snake[INNER].locked) {
					return;
				}
				// Requirement 8.13.2.2.2.1
				set_steering();
				if (! snake[OUTER].steering) {
					return;
				}
				// Requirement 8.13.2.2.2.2 (note change_mode is set after the
				// switch statement)
				break;

			case STEPPED_ADVANCE_INNER:
				set_position_tracking(INNER_SLIDE, 1);
				set_position_tracking(OUTER_SLIDE, 1);
				// Not homed or homing
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// When switching to stepped advance inner, we need to lock
				// the outer snake and then unlock the inner snake. Then we
				// need to command the inner snake to advance
				// Requirement 8.13.2.3.1
				lock_snake(OUTER);
				if (! snake[OUTER].locked) {
					return;
				}
				// Requirement 8.13.2.3.2.1
				unlock_snake(INNER);
				if (! snake[INNER].unlocked) {
					return;
				}

				// Move the inner snake forward
				// Requirement 8.13.2.3.2.2 (note change_mode is set after the
				// switch statement)
				//jwm do check on range
				DesiredPosition = mot[OUTER_SLIDE].cmd_pos - STAGGER;
				if(DesiredPosition > MAX_POS_INNER_SLIDE){
					 log_printf(LOG_INFO,
					 		"Outer slide reached maximum travel distance.\n");
                     pause();
                }
				else {
					set_position_save(INNER_SLIDE,
							DesiredPosition, STEPPED_TG);
					mot_apply();
				}
				break;

			case STEPPED_ADVANCE_OUTER:
				set_position_tracking(INNER_SLIDE, 1);
				set_position_tracking(OUTER_SLIDE, 1);
				// Not homed or homing
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// To advance the outer snake, we fist lock the inner and then
				// unlock the outer snake. Then we begin to move the outer slide
				// forward.
				// Requirement 8.13.2.4.1
				lock_snake(INNER);
				if (! snake[INNER].locked) {
					return;
				}
				// Requirement 8.13.2.4.2.1
				unlock_snake(OUTER);
				if (! snake[OUTER].unlocked) {
					return;
				}

				// Requirement 8.13.2.4.2.2
				// Move the outer snake forward
				// add check to make sure we don't move too far forward
				DesiredPosition = mot[INNER_SLIDE].cmd_pos + guided_separation + STAGGER;
				if(DesiredPosition > MAX_POS_OUTER_SLIDE){
					 log_printf(LOG_INFO,
					 		"Inner slide reached maximum travel distance.\n");
                     pause();
                }
				else {
					set_position_save(OUTER_SLIDE,
							DesiredPosition, STEPPED_TG);
					mot_apply();
				}
				break;

			case RETRACT_INNER:
				set_position_tracking(INNER_SLIDE, 1);
				set_position_tracking(OUTER_SLIDE, 1);
				// Not homed or homing
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// To retract the inner, we must first lock the outer and
				// then unlock the inner. Then we begin to move the inner
				// slide back.
				// Requirement 8.13.2.5.1
				lock_snake(OUTER);
				if (! snake[OUTER].locked) {
					return;
				}
				// Requirement 8.13.2.5.2.1
				unlock_snake(INNER);
				if (! snake[INNER].unlocked) {
					return;
				}

				// Requirement 8.13.2.5.2.2
				DesiredPosition = mot[OUTER_SLIDE].cmd_pos -
						guided_separation - STAGGER;

				if (DesiredPosition < 1.0) {
					log_printf(LOG_INFO,
						"Inner slide reached maximum travel distance.\n");
                    pause();
				}
				else {
					set_position_save(INNER_SLIDE, DesiredPosition,
							STEPPED_TG);
					mot_apply();
				}
				break;

			case RETRACT_OUTER:
				set_position_tracking(INNER_SLIDE, 1);
				set_position_tracking(OUTER_SLIDE, 1);
				// Not homed or homing
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// To retract the outer, first lock the inner snake then unlock
				// the outer snake. Next, bring back the outer slide.
				// Requirement 8.13.2.6.1
				lock_snake(INNER);
				if (! snake[INNER].locked) {
					return;
				}
				// Requirement 8.13.2.6.2.1
				unlock_snake(OUTER);
				if (! snake[OUTER].unlocked) {
					return;
				}


				// Requirement 8.13.2.6.2.2
				DesiredPosition = mot[INNER_SLIDE].cmd_pos + STAGGER;

				if (DesiredPosition < 1.0) {
					log_printf(LOG_INFO,
						"Outer slide reached maximum travel distance.\n");
                    pause();
				}
				else {
					set_position_save(OUTER_SLIDE, DesiredPosition, STEPPED_TG);
					mot_apply();
				}

				break;

			case RETRACT_STEERING:
				set_position_tracking(INNER_SLIDE,1);
				set_position_tracking(OUTER_SLIDE,1);
				// Not homed or homing
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// When switching to retract steering, lock the inner snake
				// and set the outer snake to the steering tension.
				lock_snake(INNER);
				if (! snake[INNER].locked) {
					return;
				}

				set_steering();
				if (! snake[OUTER].steering) {
					return;
				}
				break;


			case ABANDON_ABANDONING:
				set_position_tracking(INNER_SLIDE, 1);
				set_position_tracking(OUTER_SLIDE, 1);
				// Not homed or homing
				snake[INNER].homed = 0;
				snake[INNER].homing = 0;
				snake[OUTER].homed = 0;
				snake[OUTER].homing = 0;
				// Lock outer snake, then unlock inner snake
				lock_snake(OUTER);
				if (! snake[OUTER].locked) {
					return;
				}

				unlock_snake(INNER);
				if (! snake[INNER].unlocked) {
					return;
				}

				// Bring inner snake to appropriate position to get back to
				// stepped steering
				set_position_save(INNER_SLIDE,
						mot[OUTER_SLIDE].cmd_pos -
						guided_separation - STAGGER,
						STEPPED_TG);
				mot_apply();
				break;

			default:
				// This should never happen. It means change_mode was set to
				// something bad.
				SetErrorBit(M_DCU,DCU_ERR_STATE_TRANS);
				log_printf(LOG_ERROR, "Invalid mode number %d specified at "
						"line %d in %s.\n", change_mode, __LINE__, __FILE__);
				break;
		}
		//printf("\n\n***Entering new mode!***\n\n\n");
		// If we make it all the way down to here, then we must have finished
		// change to the new mode. So update the mode variable.
		// Satisifies all the parts about setting change mode in requirements
		// 8.13.2.1.2, 8.13.2.2.2.2, 8.13.3.2.2, 8.13.2.4.2.2, 8.13.2.5.2.2,
		// 8.13.2.6.2.2, 8.13.2.7.2.2.2, 8.13.2.8.2, 8.13.2.9.2.2,
		// 8.13.2.10.2.2, 8.13.2.11.2.2

// log mode with time,  current command sequence numbers as modified by mot_apply
		fprintf(guided_mode_log_file, "%i, ", change_mode);
		get_motor_cmd_numbers(mot_cmd_seq_numbers);
		fprintf (guided_mode_log_file," %i, %i, %i, %i, %i, %i \n",
				mot_cmd_seq_numbers[0],
				mot_cmd_seq_numbers[1],
				mot_cmd_seq_numbers[2],
				mot_cmd_seq_numbers[3],
				mot_cmd_seq_numbers[4],
				mot_cmd_seq_numbers[5]);

		guided_mode = change_mode;
		change_mode = -1;
	}


	// Do whatever we need to do for the mode we're in
	switch (guided_mode) {
		case HOMING_SCREWS:
			// First thing is we need to make sure both snakes are already
			// back (we already know they're unlocked)
			// Requirement 8.13.3.1.1
			home_slide(INNER);
			home_slide(OUTER);

			// Wait until both are all the way back
			if (! (snake[INNER].homed && snake[OUTER].homed)) {
				return;
			}

			// Then lock both snakes to home the tensioners
			// Requirement 8.13.3.2.1
			lock_snake(INNER);
			lock_snake(OUTER);

			// Wait for both snakes to lock
			if (! (snake[INNER].locked && snake[OUTER].locked)) {
				return;
			}

			mot[BOT_TENSIONER].home_unlock_pos =
					get_position(BOT_TENSIONER);
			mot[UL_TENSIONER].home_unlock_pos =
					get_position(UL_TENSIONER);
			mot[UR_TENSIONER].home_unlock_pos =
					get_position(UR_TENSIONER);

			// Reset zero position for both slides
			// Requirement 8.13.3.1.2.2.1
			rezero(INNER_SLIDE);
			mot[INNER_SLIDE].cmd_pos = 0.0f;
			rezero(OUTER_SLIDE);
			mot[OUTER_SLIDE].cmd_pos = 0.0f;
			
			// At this point, show that neither snakes are homed, because
			// we're about to switch to a mode that will unhome them
			snake[INNER].homed = 0;
			snake[OUTER].homed = 0;
			snake[INNER].homing = 0;
			snake[OUTER].homing = 0;

			// Move us forward by the stagger amount, if stagger is nonzero.
			// This prevents us from taking a large step when we switch to
			// stepped advance outer.
			if (STAGGER != 0) {	
				DesiredPosition = mot[INNER_SLIDE].cmd_pos +
						STAGGER;

				if (mot[OUTER_SLIDE].cmd_pos != DesiredPosition) {
					set_position_save(OUTER_SLIDE, 
							mot[INNER_SLIDE].cmd_pos +
							STAGGER, STEPPED_TG);
				}
				else if (! is_goal_reached(OUTER_SLIDE)) {
					return;
				}
			}

			// At this point we can switch to stepped_advance_outer to get
			// the separation right
			// Requirement 8.13.3.1.2.2.2
			change_mode = STEPPED_ADVANCE_OUTER;

			set_position_tracking(INNER_SLIDE, 1);
			set_position_tracking(OUTER_SLIDE, 1);
			break;

		case STEPPED_STEERING:
			// All we need to do is wait for an execute command, as the
			// steer() function does everything else for us. The execute()
			// function will also take care of switching modes for us, so we
			// don't need to worry about that either.
			// Requirement 8.13.3.2.1
			break;

		case STEPPED_ADVANCE_INNER:

			// Check that the inner slide hasn't run into anything
			force_inner = get_force(INNER_SLIDE);
			
			// Wait until the inner snake advances enough
			if (! is_goal_reached(INNER_SLIDE)) {
				return;
			}

			// Once that happens switch to advancing the outer snake
			// Requirement 8.13.3.3.2
			change_mode = STEPPED_ADVANCE_OUTER;

			break;

		case STEPPED_ADVANCE_OUTER:
			// Wait until the outer snake advances enough
			if (! is_goal_reached(OUTER_SLIDE)) {
				return;
			}

			// Once that happens, switch to steering
			// Requirement 8.13.3.4.1
			change_mode = STEPPED_STEERING;
			break;

		case RETRACT_INNER:
			// Wait until the inner snake retracts enough
			if (! is_goal_reached(INNER_SLIDE)) {
				return;
			}

			// Once that happens, switch to retract steering
			change_mode = RETRACT_STEERING;
			break;

		case RETRACT_OUTER:
			// Wait until the outer snake retracts enough
			if (! is_goal_reached(OUTER_SLIDE)) {
				return;
			}

			// Once that happens, switch to retract inner
			// Requirement 8.13.3.6.1
			change_mode = RETRACT_INNER;
			break;

		case RETRACT_STEERING:
			// Nothing to do, since we're just waiting for the execute call
			break;

		case ABANDON_ABANDONING:
			// Wait until the inner slide is in the right position
			if (! is_goal_reached(INNER_SLIDE)) {
				return;
			}

			// Once that happens, switch to steering
			change_mode = STEPPED_STEERING;
			break;
		default:
			SetErrorBit(M_DCU,DCU_ERR_STATE_TRANS);
			log_printf(LOG_ERROR, "Invalid mode number %d specified at "
					"line %d in %s.\n", change_mode, __LINE__, __FILE__);
			break;
	}

}


