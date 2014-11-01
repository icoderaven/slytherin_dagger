/** @file guided_motions.h
 *	@brief Function prototypes and constants for the guided motions component.
 *
 *	@author Cornell Wright
 */

#ifndef GUIDED_MOTIONS_H
#define GUIDED_MOTIONS_H



// Global status variables. See guided_motions.cpp for information.
extern int guided_mode;
extern int guided_paused;
extern int change_mode;
extern float guided_separation;


// Modes used for the guided_mode parameter
#define HOMING_SCREWS             0
#define STEPPED_STEERING          1
#define STEPPED_ADVANCE_INNER     2
#define STEPPED_ADVANCE_OUTER     3
#define RETRACT_INNER             4
#define RETRACT_OUTER             5
#define RETRACT_STEERING          6
#define INITIALIZING_INIT        16
#define ABANDON_ABANDONING		 17

// Constants for the new_mode parameter
#define HOMING          0
#define STEPPED         1
#define RETRACT         2
#define INITIALIZING    7
#define ABANDON			8



void guided_motions_main_loop(void);
void guided_motions_init(void);
void guided_motions_reinit(void);
void set_mode(int new_mode);
void pause(void);
void unpause(void);
void execute(void);
void steer(float x, float y);
void set_separation(float dist);


#endif /* GUIDED_MOTIONS_H */


