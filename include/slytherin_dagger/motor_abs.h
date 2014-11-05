/** @file motor_abs.h
 *	@brief Function prototypes and constants for the Motor Abstraction
 *	Component
 *
 *	@author Cornell Wright
 */


#ifndef MOTOR_ABS_H
#define MOTOR_ABS_H

// Function to get access to some static variables. (Used only for unit
// testing)
#ifdef DEBUG_UNIT_TEST
#include "medhost/state.h"
#include "medhost/usbcomm.h"
void motor_abs_debug_getptrs(sCmd **base_cmd_ptr, sCmd **outer_cmd_ptr,
		sState **base_state_ptr, sState **outer_state_ptr,
		sUSBComm **usb_state_ptr);
#endif

#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#endif

void mot_apply(void);
void motor_abs_init(char *com_port_name);
int pause_motor(int motor, int paused);
int set_position(int motor, float dist, float tg);
float get_position(int motor);
void rezero(int motor);
int set_force_lim(int motor, float force);
float get_force(int motor);
void mot_main(void);
int get_limit_sw(int mot);
void get_motor_cmd_numbers(int mot_cmd_seq_numbers[]);
void set_position_tracking(int motor, char value);
void set_lock_indicator(int motor, char value);




#endif /* MOTOR_ABS_H */

