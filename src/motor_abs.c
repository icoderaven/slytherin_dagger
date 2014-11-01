/** @file motor_abs.c
 *	@brief Contains function definitions for the motor abstraction component.
 *
 *	@author Cornell Wright
 */


#include <stdio.h>
#include <conio.h>
#include "motor_abs.h"
#include "config_defaults.h"
#include "log.h"
#include "../system/crc32.h"		// Only need this to build the table on init
#include "../system/state.h"		// Used to keep track of device state
#include "../communication_interface/usbcomm.h"	// Used to send an receive, etc.
#include "log_state.h"
#include "CardioError.h"
#include "ErrorData.h"
#include "superCommon.h"
#include "keithley.h"
#include "../system/motor.h"

#define NUM_MOTORS 6

#define INNER 0
#define OUTER 1

/** @brief Assert to make sure the pointer to a motor_t's command is valid */
#define VALIDATE_MOT_CMD(cmd)												\
	if (!(																	\
			((unsigned int)(cmd) >= (unsigned int)base_cmd.motors &&		\
			(unsigned int)(cmd) < ((unsigned int)base_cmd.motors +			\
			sizeof(base_cmd.motors)))										\
			||																\
			((unsigned int)(cmd) >= (unsigned int)outer_cmd.motors &&		\
			(unsigned int)(cmd) < ((unsigned int)outer_cmd.motors +			\
			sizeof(outer_cmd.motors)))) ) {									\
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);								\
	}

/** @brief Holds information about a motor including its MotorStat which is
 *	necessary for talking to the comm component. */
typedef struct {
	int zero_pos;		// Local idea of where zero is for this motor - allows
						// us to "re-zero" as we're running
	float mm_per_tick;	// How many millimeters this motor or slide moves per
						// tick.
	float n_per_lsb;	// How many Newtons force this motor puts out per ADC
						// value.
	int reversed;		// If this motor moves backwards from the others
	float last_pos;		// The last position update we received from this motor
	float last_force;	// The last force update we received from this motor
	int updated;		// Whether we've changed the position
	sMotorCmd *cmd;		// Pointer to set command values for this motor
	sMotorState *state;	// Pointer to get state values for this motor
} motor_t;

/** @brief Command structs for the base board and outer board */
static sCmd base_cmd, outer_cmd;

/** @brief State structs for the base board and outer board */
static sState base_state, outer_state;

/** @brief Keeps track of all the USB stuff */
static sUSBComm usb_state;
extern sUSBComm *Pusb_state;

static FILE *base_command_log;
static FILE *outer_command_log;
static FILE *base_communication_log;
static FILE *outer_communication_log;
static FILE *pos_log;
static FILE *base_state_log;
static FILE *outer_state_log;
static unsigned int base_last_timestamp;
static unsigned int outer_last_timestamp;


/** @brief Our array of motors */
static motor_t motors[NUM_MOTORS];


#ifdef DEBUG_UNIT_TEST
/** @brief Used to get pointers to static variables for unit testing. The
 *	DEBUG_UNIT_TEST symbol should only be defined when compiling with the
 *	unit test framework.
 *
 *	@param base_cmd_ptr Pointer to where to put a pointer to base_cmd
 *	@param outer_cmd_ptr Pointer to where to put a pointer to outer_cmd
 *	@param base_state_ptr Pointer to where to put a pointer to base_state
 *	@param outer_state_ptr Pointer to where to put a pointer to outer_state
 *	@param usb_state_ptr Pointer to where to put a pointer to usb_state
 */
void motor_abs_debug_getptrs(sCmd **base_cmd_ptr, sCmd **outer_cmd_ptr,
		sState **base_state_ptr, sState **outer_state_ptr,
		sUSBComm **usb_state_ptr) {

	// Give 'em the pointers they asked for
	*base_cmd_ptr = &base_cmd;
	*outer_cmd_ptr = &outer_cmd;
	*base_state_ptr = &base_state;
	*outer_state_ptr = &outer_state;
	*usb_state_ptr = &usb_state;
}
#endif



/** @brief Initializes the motor abstraction component.
 *
 *	@param com_port_name The name of the COM port we are to communicate to.
 */
void motor_abs_init(char *com_port_name) {
	int i;
	char fname[100];

	sprintf_s(fname, sizeof(fname), "logs/%s_base_state_log.csv",
			filename_time());
	base_state_log = init_state_log(fname);

	sprintf_s(fname, sizeof(fname), "logs/%s_outer_state_log.csv",
			filename_time());
	outer_state_log = init_state_log(fname);

	sprintf_s(fname, sizeof(fname), "logs/%s_base_command_log.csv",
			filename_time());
	base_command_log = NULL;
	fopen_s(&base_command_log,fname, "a");

	sprintf_s(fname, sizeof(fname), "logs/%s_outer_command_log.csv",
			filename_time());
	outer_command_log = NULL;
	fopen_s(&outer_command_log,fname, "a");

	sprintf_s(fname, sizeof(fname), "logs/%s_base_communication_log.csv",
			filename_time());
	base_communication_log = NULL;
	fopen_s(&base_communication_log,fname, "a");

	sprintf_s(fname, sizeof(fname), "logs/%s_outer_communication_log.csv",
			filename_time());
	outer_communication_log = NULL;
	fopen_s(&outer_communication_log,fname, "a");
	pos_log = NULL;
	fopen_s(&pos_log,"pos_log.txt", "w");

	fprintf(base_command_log, "Time, Packet Seq Num, SafetyCmd, Outer Slide Seq, Outer Slide K_P, Outer Slide K_I, Outer Slide K_D, Outer Slide Goal, Outer Slide Ta, Outer Slide Tg, Outer Slide Current Limit, Outer Slide Current Inc, Outer Slide Current Dec, Outer Slide mtr_bits, Inner Slide Seq, Inner Slide K_P, Inner Slide K_I, Inner Slide K_D, Inner Slide Goal, Inner Slide Ta, Inner Slide Tg, Inner Slide Current Limit, Inner Slide Current Inc, Inner Slide Current Dec, Inner Slide mtr_bits, Inner Tensioner Seq, Inner Tensioner K_P, Inner Tensioner K_I, Inner Tensioner K_D, Inner Tensioner Goal, Inner Tensioner Ta, Inner Tensioner Tg, Inner Tensioner Current Limit, Inner Tensioner Current Inc, Inner Tensioner Current Dec, Inner Tensioner mtr_bits, \n");
	fprintf(outer_command_log,"Time, Packet Seq Num, SafetyCmd, UR Tensioner Seq, UR Tensioner K_P, UR Tensioner K_I, UR Tensioner K_D, UR Tensioner Goal, UR Tensioner Ta, UR Tensioner Tg, UR Tensioner Current Limit, Upper Right Tensioner Current Inc, Upper Right Tensioner Current Dec, Upper Right Tensioner mtr_bits, UL Tensioner Seq, UL Tensioner K_P, UL Tensioner K_I, UL Tensioner K_D, UL Tensioner Goal, UL Tensioner Ta, UL Tensioner Tg, UL Tensioner Current Limit, UL Tensioner Current Inc, UL Tensioner Current Dec, UL Tensioner mtr_bits, Bottom Tensioner Seq, Bottom Tensioner K_P, Bottom Tensioner K_I, Bottom Tensioner K_D, Bottom Tensioner Goal, Bottom Tensioner Ta, Bottom Tensioner Tg, Bottom Tensioner Current Limit, Bottom Tensioner Current Inc, Bottom Tensioner Current Dec, Bottom Tensioner mtr_bits\n");

	fprintf(base_communication_log, "Timestamp, comm rx count, comm rx lastseq, comm tx count, comm crc err,  comm overrun, comm frame err, usb rx count, usb rx error, usb rx lastseq, usb tx_ count \n");
	fprintf(outer_communication_log, "Timestamp, comm rx count, comm rx lastseq, comm tx count, comm crc err,  comm overrun, comm frame err \n");


	// First, build the CRC table
	crc32_build_table();

	// Initialize the commands
	init_sCmd(&base_cmd);
	init_sCmd(&outer_cmd);

	// Initialize the USB stuff
	usbcomm_init(&usb_state);
	usbcomm_open(&usb_state, com_port_name);

	Pusb_state = &usb_state;

	base_cmd.destination = SD_BASE;
	outer_cmd.destination = SD_OUTER;

	base_last_timestamp = 0;
	outer_last_timestamp = 0;


	// Do all the initialization necessary for each motor
	for (i = 0; i < NUM_MOTORS; i++) {
		motors[i].reversed = (motor_direction_const[i] == 1) ? 0 : 1;

		// Set this motor's local zero
		motors[i].zero_pos = ZERO_POS;

		// We haven't updated this motor
		motors[i].updated = 0;

		// See if this motor is on the outer board
		if (i == BOT_TENSIONER || i == UL_TENSIONER
				|| i == UR_TENSIONER) {

			// Initialize the pointer to send this motor commands and get its
			// state
			motors[i].cmd = &outer_cmd.motors[motor_idx_const[i]];
			motors[i].state = &outer_state.motors[motor_idx_const[i]];

			// Set the time to accel (TA) and time to goal (TG) consts for each motor
			motors[i].cmd->Ta = OUTER_TA;
			motors[i].cmd->Tg = OUTER_TG;
			motors[i].cmd->K_P *= 2;

			// Set its distance and force constants
			// Requirements 9.1.3.1, 9.1.3.2, 9.2.3, 9.3.3.1, 9.3.3.2, and 9.4.3
			// in that it make sure the appropriate constant will be used for the
			// given motor.
			motors[i].mm_per_tick = (float)OUTER_MM_PER_TICK;
			motors[i].n_per_lsb = OUTER_N_PER_LSB;
		}
		// Otherwise it must be a slide or inner tensioner
		else {

			// Initialize the pointer to send this motor commands and get its
			// state
			motors[i].cmd = &base_cmd.motors[motor_idx_const[i]];
			motors[i].state = &base_state.motors[motor_idx_const[i]];

			// Set the time to accel (TA) and time to goal (TG) consts for each motor
			motors[i].cmd->Ta = OTHER_TA;
			motors[i].cmd->Tg = OTHER_TG;

			// Set the distance and force constants
			if (i == INNER_TENSIONER) {
				// Requirements 9.1.2.1, 9.1.2.2, 9.2.2, 9.3.2.1, 9.3.2.2, and 9.4.2
				// in that it make sure the appropriate constant will be used for the
				// given motor.
				motors[i].mm_per_tick = (float)INNER_MM_PER_TICK;
				motors[i].n_per_lsb = INNER_N_PER_LSB;
			}
			else { // Must be a slide
				// Requirements 9.1.1.1, 9.1.1.2, 9.2.1, 9.3.1.1, 9.3.1.2, and 9.4.1
				// in that it make sure the appropriate constant will be used for the
				// given motor.
				motors[i].mm_per_tick = (float)SLIDE_MM_PER_TICK;
				motors[i].n_per_lsb = SLIDE_N_PER_LSB;
			}
		}
	}

	// Send this to the motors
	mot_apply();
}

/** @brief Sends any changes made by set_position() or set_force() to the
 *	motors. This allows for synchronization as multiple updates are send
 *	at once.
 */
void mot_apply() {
	int i;
	int sendCmds_retval;

	// Increment the sequence numbers on any motors we've updated since
	// the last time we applied changes
	for (i = 0; i < NUM_MOTORS; i++) {
		if (motors[i].updated) {
			// Make sure the command is valid
			VALIDATE_MOT_CMD(motors[i].cmd);
			motors[i].cmd->cmd_seq_number++;
			motors[i].updated = 0;
		}
	}

	// Requirement 9.5
	sendCmds_retval = usbcomm_sendCmds(&usb_state, &base_cmd, &outer_cmd);

	if (sendCmds_retval != 0) {
        SetErrorBit(M_DCU,DCU_ERR_INIT_SERIAL);
	}


	fprintf(pos_log, "apply()\n");
	fflush(pos_log);

	// Save what we just sent

	usbcomm_printCmd(base_command_log, &base_cmd);
	usbcomm_printCmd(outer_command_log, &outer_cmd);

}

/** @brief Pauses or unpauses the selected motor.
 *
 *	@param motor The motor to pause.
 *	@param paused Nonzero to pause the motor, 0 to unpause
 *
 *	@return 0 on success, -1 on error
 */
int pause_motor(int motor, int paused) {
	motor_t *mot;

	// Make sure it's a valid motor
	if (motor < 0 || motor >= NUM_MOTORS) {
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
		return -1;
	}

	// Figure out which motor they're referring to
	mot = &motors[motor];

	// Tell it to pause or unpaused depending on what paused is set to
	if (paused) {
		// Requirement 9.7.1
		mot->cmd->mtr_bits |= (1 << MTR_BITS_PAUSE);
	}
	else {
		// Requirement 9.7.2
		mot->cmd->mtr_bits &= ~(1 << MTR_BITS_PAUSE);
	}

	return 0;
}


/** @brief Sets the position for a given motor in mm.
 *
 *	@param motor Which motor to set the position for. See constants in header file
 *	@param dist Target position in millimeters.
 *	@return -1 on error, 0 if out value is stale, 1 if out value is fresh.
 */
int set_position(int motor, float dist, float tg) {
    int dest_pos;
	motor_t *mot;

	//fprintf(snake_log, "\nset_position(motor=%d, dist=%f, tg=%f)\n", motor, dist, tg);
	fprintf(pos_log, "sp(%d,%f)\n", motor, dist);
	fflush(pos_log);
	//printf("\nset_position(motor=%d, dist=%f, tg=%f)\n", motor, dist, tg);

	// Make sure we're given a valid motor
	if (motor < 0 || motor >= NUM_MOTORS) {
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
		return -1;
	}

	if (tg < 150) {
		tg = 150;
	}

	// Figure out which motor they're referring to
	mot = &motors[motor];

	// Calculate the desination position in ticks
	// All requirements 9.1.1*, 9.1.2*, and 9.1.3*
	dest_pos = (int)(((mot->reversed) ? -1.0f : 1.0f) * ((float)dist / mot->mm_per_tick)) +
			mot->zero_pos;

	//printf("Commanding position of motor %d to 0x%x ticks. Reversed = %d, Dist = %.3f, mm/tick=%.5f\n", motor, dest_pos, mot->reversed, dist, mot->mm_per_tick);

	// Set the destination position and time to goal
	mot->cmd->goal = dest_pos;
	// Requirement 9.1.4
	mot->cmd->Tg = (unsigned int)(tg * MS_TO_TG);
	mot->cmd->Ta = mot->cmd->Tg / 4;
	mot->updated = 1;



	return 0;
}


/** @brief Returns the last known position of the desired motor.
 *
 *	@param motor Which motor to get position for. See constants in header file
 *	@return last known position of the desired motor
 */
float get_position(int motor) {
	int pos;
	float pos_in_mm;
	motor_t *mot;

	// Make sure we're given a valid motor
	if (motor < 0 || motor >= NUM_MOTORS) {
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
		log_printf(LOG_ERROR, "Invalid motor number %d passed to get_position!\n",
				motor);
		printf("Invalid motor number %d passed to get_position\n", motor);
		return -1;
	}

	// Get the motor structure.
	mot = &motors[motor];

	// Get the position of the motor
	pos = mot->state->quadcount;

	// Return the result of the converting encoder ticks to mm, taking into account
	// motor reversal
	// All requirements 9.3*
	pos_in_mm = (float)((pos - mot->zero_pos) * ((mot->reversed) ? -1 : 1))
			* mot->mm_per_tick;
	return pos_in_mm;
}

/** @brief Sets the position considered zero for the requested motor to its
 *	current position.
 *
 *	@param motor Which motor to get position for. See constants in header file
 *	@return -1 on error, 0 on success.
 */
void rezero(int motor) {
	int pos;
	motor_t *mot;

	// Make sure that we're given a valid motor
	if (motor < 0 || motor >= NUM_MOTORS) {
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
		log_printf(LOG_ERROR, "Invalid motor number %d specified at line "
				"%d in %s.\n", motor, __LINE__, __FILE__);
		return;
	}

	// Get the motor structure.
	mot = &motors[motor];

	// Get the position of the motor
	pos = mot->state->quadcount;

	// Reset our idea of zero
	// Requirement 9.6
	mot->zero_pos = pos;
}


/** @brief Sets the force limit for the desired motor
 *
 *	@param motor Which motor to set the force limit for. See constants in header file
 *	@param force The desired force in Newtons
 *	@return -1 on error, 0 if out value is stale, 1 if out value is fresh
 */
int set_force_lim(int motor, float force)
{
	unsigned int lim;
	motor_t *mot;

	// Make sure we're given a valid motor
	if (motor < 0 || motor >= NUM_MOTORS) {
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
		return -1;
	}

	// Get the motor structure
	mot = &motors[motor];

	// Calculate the current limit as an adc value
	// All requirements 9.2*
	lim = (unsigned int)(force/mot->n_per_lsb);

	//printf("Setting motor %d current to %6.2f N = %d lsb.\n", motor, force, lim);

	// Set the actual current limit and return the result of setting it
	mot->cmd->cur_limit = lim;

	return 0;
}


/** @brief Gets the current that the specified motor is providing.
 *
 *	@param motor Which motor to get the force of. See constants in header file.
 *	@return The force of the specified motor in Newtons or -1 on error.
 */
float get_force(int motor) {
	unsigned int current;
	float force;
	motor_t *mot;

	// Make sure we're given a valid motor
	if (motor < 0 || motor >= NUM_MOTORS) {
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
		log_printf(LOG_ERROR, "Invalid motor number %d passed to get_force().",
				motor);
		//exit(1);
		return -1;
	}

	// Get the motor structure
	mot = &motors[motor];

	// Get the current of the desired motor
	current = mot->state->current;

	// Convert from an ADC value to force in Newtons
	// All requirements 9.4*
	force = (float)current * mot->n_per_lsb;
	mot->last_force = force;
	return force;
}


/** @brief Main loop for motor abstraction */
void mot_main(void) {
	// Let the comm do its thing
	usbcomm_timeslice(&usb_state);
	usbcomm_timeslice(&usb_state);
	// Get any updates in state
	usbcomm_getStates(&usb_state, &base_state, &outer_state);

	// save processor state packets
	if (base_state.timestamp > base_last_timestamp )
	{
		base_last_timestamp = base_state.timestamp;
		log_state_packet(base_state_log, &base_state);
		usbcomm_printState(base_communication_log, &base_state);
	}
	if (outer_state.timestamp > outer_last_timestamp)
	{
		outer_last_timestamp = outer_state.timestamp;
		log_state_packet (outer_state_log, &outer_state);
		usbcomm_printState(outer_communication_log, &outer_state);
	}
}


int get_limit_sw(int mot) {
	if (mot == INNER_SLIDE) {
		return (base_state.SafetyStat & SFT_S_IN_LIM) ? 1 : 0;
	}
	else if (mot == OUTER_SLIDE) {
		return (base_state.SafetyStat & SFT_S_OUT_LIM) ? 1 : 0;
	}
	else {
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
		log_printf(LOG_ERROR,
				"Invalid motor number %d passed to get_limit_sw().", mot);
		return 0;
	}

}
void get_motor_cmd_numbers(int mot_cmd_seq_numbers[])
{
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		mot_cmd_seq_numbers[i] = motors[i].cmd->cmd_seq_number;
	}
}

// set mtr_bits MTR_BITS_TRACKING  for the specified motor, to 0 or 1 depending on value
// if set then the firmware should track position and report errors in
// tracking,  if clear, then firmware should not detect/report tracking errors
void set_position_tracking(int motor, char value)
{
	motor_t *mot;
	// Make sure we're given a valid motor
	if (motor < 0 || motor >= NUM_MOTORS)
	{
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
	}
	else
	{
		// Get the motor structure
		mot = &motors[motor];
		if ( value == 0)
		{
			mot->cmd->mtr_bits &= ~(1 << MTR_BITS_TRACKING);
		}
		else
		{
			mot->cmd->mtr_bits |= (1 << MTR_BITS_TRACKING);
		}
	}
}
// set mtr_bits MTR_BITS_LOCK  for the specified motor, to 0 or 1 depending on value
// if set then the firmware should watch for a sudden current drop
//  if clear, then firmware should not detect/report sudden current drop
void set_lock_indicator(int motor, char value)
{
	motor_t *mot;
	// Make sure we're given a valid motor
	if (motor < 0 || motor >= NUM_MOTORS)
	{
        SetErrorBit(M_DCU,DCU_ERR_INDEX_MOTOR);
	}
	else
	{
		// Get the motor structure
		mot = &motors[motor];
		if ( value == 0)
		{
			mot->cmd->mtr_bits &= ~(1 << MTR_BITS_LOCK);
		}
		else
		{
			mot->cmd->mtr_bits |= (1 << MTR_BITS_LOCK);
		}
	}
}

