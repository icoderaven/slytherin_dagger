/** @file log_state.c
 *	@brief used to print out packets in CSV format so that it can be opened
 *	in excel, octave, matlab, etc.
 *
 *	@author Cornell Wright
 */
#include <stdio.h>
#include <windows.h>
#include <time.h>
#include "../system/state.h"
#include "../communication_interface/usbcomm.h"


/** @brief The time in ms that this run started. Used for time stamps */
static int start_time;


/** @brief Initializes the state log.
 *
 *	@param fname The file name to log state packets to.
 */
FILE* init_state_log(char *fname) {
	// Try to open the file
	FILE* state_log = 0;

	state_log = NULL;
	fopen(&state_log, fname, "a");
	if (state_log == NULL) {
		printf("ERROR!! Failed to open log %s.\n", fname);
	}

	// Print a header
	fprintf(state_log, "DcuTime,LocalTime,PacketSafetyStat,PacketSafetyError,"
			"cmd0,quadcount0,quadcount02,goal0,pwm0,current0,pwm_limit0,traj_goal0,traj_vel0,traj_velmax0,traj_accel0,traj_ticks0,traj_state0,mtrBits0,safetyError0,"
			"cmd1,quadcount1,quadcount12,goal1,pwm1,current1,pwm_limit1,traj_goal1,traj_vel1,traj_velmax1,traj_accel1,traj_ticks1,traj_state1,mtrBits1,safetyError1,"
			"cmd2,quadcount2,quadcount22,goal2,pwm2,current2,pwm_limit2,traj_goal2,traj_vel2,traj_velmax2,traj_accel2,traj_ticks2,traj_state2,mtrBits2,safetyError2\n");

	// Figure out our start time
	start_time = GetTickCount();
	return (state_log);
}

/** @brief Writes a state packet to the log.
 *
 *	@param log_state pointer to the state packet we should log.
 */
void log_state_packet(FILE *state_log, sState *log_state) {
	int i;


	usbcomm_stateWordSwap(log_state);
	fprintf(state_log, "%i,", GetTickCount());  // DCU current time in ms
	fprintf(state_log, "%i,", log_state->timestamp);
	fprintf(state_log, "%x,", log_state->SafetyStat);
	fprintf(state_log, "%x,", log_state->SafetyError);
	// Print all the data

	for (i = 0; i < 3; i++) {
		fprintf(state_log, "%i,", log_state->motors[i].cmd_seq_last);
		fprintf(state_log, "%d,", log_state->motors[i].quadcount);
		fprintf(state_log, "%d,", log_state->motors[i].quadcount2);
		fprintf(state_log, "%d,", log_state->motors[i].goal);
		fprintf(state_log, "%d,", log_state->motors[i].pwm);
		fprintf(state_log, "%u,", log_state->motors[i].current);
		fprintf(state_log, "%d,", log_state->motors[i].pwm_limit);
		fprintf(state_log, "%.10f,", (log_state->motors[i].traj_goal));
		fprintf(state_log, "%.10f,", (log_state->motors[i].traj_vel));
		fprintf(state_log, "%.10f,", (log_state->motors[i].traj_velmax));
		fprintf(state_log, "%.10f,", (log_state->motors[i].traj_accel));
		fprintf(state_log, "%u,", log_state->motors[i].traj_ticks);
		fprintf(state_log, "%d,", log_state->motors[i].traj_state);
		fprintf(state_log, "%d,", log_state->motors[i].mtr_bits);

		// handle the trailing comma, since Octave doesn't like it
		if (i<2) {
			fprintf(state_log, "%x,", log_state->motors[i].SafetyError);
		}
		else {
			fprintf(state_log, "%x", log_state->motors[i].SafetyError);
		}
	}

	//Print a new line
	fprintf(state_log, "\n");
	fflush(state_log);
}
