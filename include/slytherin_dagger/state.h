/**
 * \file state.h
 * This file contains definitions of the state and command structures
 * used to implement the communications between boards and host and also
 * for storing the internal state of the system
 *
 * \author Michael Schwerin
 * \date 2007
 * 2009.2.8 MBS modified for version 3.0 board.  Added second current field.
 */
#ifndef STATE_H_
#define STATE_H_

#include "types.h"

/************************ REFERENCE AREA *************************************\
 *
 * PWM has a range of +/- 1600 (board.h)
 * Current is measured on a 0.2 Ohm resistor with a 50x gain amplifier fed into
 *    a 10 bit ADC with a 3.3v reference
 *    lsb of ADC represents 3.3v/1024 = 3.223 mV / 50x gain =  64.45 uV
 *    64.45 uV on 0.2 Ohm resistor -> 322.265 uA per ADC lsb
 * Loop units are currently 4 ms (control loop runs at ~250 Hz) (board.h)
 *    (actually its 4.00007 ms.  I'm stealing those 70ns and saving them up kinda
 *     like the stealing a fraction of a cent from bank transactions scam BWAHAHAHAHA)
 *
 * CRC is 32bit with polynomial used in MPEG2, IEEE 802.3, V.42:  0x04C11DB7
 * The firmware has an internal 32 bit msec counter so don't let it run for more than 49 days
\*****************************************************************************/

/*******************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
****                                                                        ****
**** WARNING!! Alignment is critical here. Doubles must be 8-byte aligned.  ****
**** ARMgcc allows them to fall on 4 byte alignment which confuses intel    ****
**** machines.  Every struct must be a multiple of 8 bytes in length. Ints  ****
**** must be 4-byte aligned.                                                ****
****                                                                        ****
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
********************************************************************************
*******************************************************************************/

// define bit fields used for controlling motor behaviors
/// Used to pause a motor which is implemented by pausing the trajectory generator
#define MTR_BITS_PAUSE 0
/// Used to reverse the positive rotation direction of the motor
#define MTR_BITS_REVERSE 1
//1=command should be position tracked
#define MTR_BITS_TRACKING 2
// 1 means locked, or locking, to distinguish between unlocking / locking
#define MTR_BITS_LOCK 3
#define MTR_BITS_LOCK_MASK  0x08


/** \brief per motor structure containing current state of the physical system and control structure
 *
 *  This structure holds the state information for a single motor including its quadrature count,
 *  ADC sampled current reading, velocity, position error, trajectory, drive strength/direction, etc.
 *  Structure size v3.0:  7*8 + 14*4 = 112 bytes
 */
typedef struct _s_motor_state {
	double	integral;		///< integral accumulation of error
	double	error;			///< position error
	double	lasterror;		///< last value of position error used for D term calculation in PID

	// trajectory related values
	double	traj_goal;		///< the working goal that is updated during every motion cycle
	double	traj_vel;		///< the current velocity
	double	traj_velmax;	///< the velocity of the trapazoid plateau
	double	traj_accel;		///< the current acceleration rate

	uint32	traj_ticks;		///< the number of motion cycles until the end of the current trajectory stage
	traj_states_t  traj_state;	///< stores the current phase of the trajectory

	int32	quadcount;		///< motor position according to the output encoder
	int32	quadcount2;		///< motor position according to the motor encoder
	int32	goal;			///< The goal commanded by the last command packet
	int32	zeropos;		///< the value of quadcount when the slide limit is reached for slides
	int32	pwm;			///< the PWM value the motor is being driven with
	uint32	current;		///< the measured motor current
	uint32	current2;		///< the redundant motor current measurement
	int32	pwm_limit;		///< the PWM value magnitude which may not be exceeded (output of current limiting)

	uint32	mtr_bits;		///< active status of the various aspects of this channel
	uint32	SafetyError;	///< Error flags
	uint32	padding;		///< round out the structure size to multiple of 8 bytes

	uint32	cmd_seq_last;	///< the last command sequence executed
} sMotorState, *psMotorState;

/**
 * This structure holds the command information for a single motor.  The data
 * is sent by the host computer and includes PID gains, trajectory properties,
 * and operational limits (motor current), as well as some control bitfields
 * Structure size v3.0: 12*4 = 48 bytes
 */
typedef struct _s_motor_cmd {
	uint32	cmd_seq_number;	///< used on the receiving end to tell if there is a new command to process (prevent reexecution)
	// PID related components
	float	K_P;			///< Proportional gain constant
	float	K_I;			///< Integral gain constant
	float	K_D;			///< Derivative gain constant
	// trajectory related components
	int32	goal;			///< the goal used to calculate new trajectory
	int32	zeropos;		///< used to store the zero position state on the robot for PC software failure recovery
	uint32	Ta;				///< acceleration (ramp up and down) time in loop ticks (explaned in the reference section above)
	uint32	Tg;				///< total time to goal for trajectory in loop ticks (explaned in the reference section above)
	// current related components
	uint32	cur_limit;		///< the maximum allowed current reading (if above then throttle the PWM)
	uint32	cur_inc;		///< ammount to increase pwm limit by if motor current reading < limit
	uint32	cur_dec;		///< ammount to decrease pwm limit by if motor current reading > limit
	int32	mtr_bits;		///< control of various aspects of this channel (pause, reversed, etc.)
} sMotorCmd, *psMotorCmd;

/** \brief communications packet format for status returned to host
 *
 *  This packet format is used to carry data back from the motor controller
 *  board to the host computer and includes information such as motor postion,
 *  sensor values, trajectory information, etc.
 *  Structure size v3.0: 3*112 + 2*2 + 17*4 = 408 bytes
 */
typedef struct _s_state {
	uint16	padding;			///< make sure the packet doesn't start with an escape byte (over USB) to ensure single escape at start of transmission
	uint16	len;				///< packet length (simply sizeof(sState)) so host software can talk to different revisions if packet grows/changes
	uint32	firmware_version;	///< firmware version info so host software can talk to different revisions if packet grows/changes
	src_dest_t  source;			///< where did this state information come from @see src_dest
	uint32	timestamp;			///< number of milliseconds since boot

	sMotorState  motors[3];		///< state of the three individual motors this board controls

	// USB link counters (only used by the base board)
	uint32	usb_rx_count;		///< count of USB objects successfully recved
	uint32	usb_rx_error;		///< count of CRC errors on recved USB objects on the BASE board
	uint32	usb_rx_lastseq;		///< on BASE set to pkt_seq_number of incoming usb sCmd
								///< objects with SD_BASE destination (to measure round trip latency)
	uint32	usb_tx_count;		///< number of state objects transmitted over the USB link

	// Async comm link counters
	uint32	comm_rx_count;		///< packets received AND processed by the main program loop (incremented when loop asks for a packet)
	uint32	comm_crc_error;		///< crc error on recv
	uint32	comm_ovre_error;	///< data overrun errors on recv
	uint32	comm_frame_error;	///< frame error on recv
	uint32	comm_rx_lastseq;	///< on BASE set to pkt_seq_number of incoming usb sCmd object with SD_OUTER destination
								///< on OUTER set to pkt_seq_number of incoming sCmd object over RS485 link (to measure round trip latency)
	uint32	comm_tx_count;		///< number of packets sent to the other end of the link
	uint32	spurious_irqs;		///< counts spurious interrupts which occur for example if a interrupt is disabled between the time the AIC is signalled  and the CPU responds to it
	uint32	SafetyStat;			///< reports current status of safety inputs
	uint32	SafetyError;		///< reports error conditions
	uint32	crc32;				///< CRC32 calculated with polynomial used in MPEG2, IEEE 802.3, V.42 0x04C11DB7
} sState, *psState;

/**
 *  This packet format is used to carry data to the motor controller from the
 *  host computer and includes information such as motor goals, gains, etc.
 *  Structure size v3.0: 2*2 + 5*4 + 3*48 = 168 bytes
 */
typedef struct _s_cmd {
	uint16	padding;			///< make sure the packet doesn't start with an escape byte to ensure single escape at start of transmission
	uint16	len;				///< packet length (simply sizeof(sState)) so host software can talk to different revisions if packet grows/changes
	uint32	firmware_version;	///< firmware version info so host software can talk to different revisions if packet grows/changes
	src_dest_t  destination;	///< which board should these commands be executed on? @see src_dest
	uint32	pkt_seq_number;		///< counter of packets sent

	sMotorCmd motors[3];		///< commands for the three individual motors controlled by this board @see sMotorCmd

	uint32	SafetyCmd;			///< Safety system commands from DCU
	uint32	crc32;				///< CRC32 calculated with polynomial used in MPEG2, IEEE 802.3, V.42 0x04C11DB7
} sCmd, *psCmd;

extern sState g_state;			///< single global instance of the structure which stores all state of the system
extern sCmd g_cmd;				///< single global instance of the structure which stores all commands for the system

/**
 * Safety system flags used for SafetyStat and SafetyCmd
 * commands from DCU and local are either ORed or ANDed together
 */
// safety commands
#define SFT_C_BUZZER	(1<<0)	///< (OR) Command a device to turn on its buzzer
#define SFT_C_ENABLE	(1<<1)	///< (AND) Command a device to assert SAFETY_EN line
#define SFT_C_RESET		(1<<2)	///< (DCU only) Command the base CPU to assert SAFETY_RESET

#define SFT_C_VOLT_TEST	(1<<4)	///< (DCU only) Command the base to assert V_MONITOR_TEST
#define SFT_C_CLR_ERR	(1<<5)	///< (DCU only) Clear error condition and resume normal operation
#define SFT_C_M1_LOCK	(1<<6)	///< (DCU only) Signal that the motor is tensioner in lock state so increase allowable tracking error in locking direction
#define SFT_C_M2_LOCK	(1<<7)	///< (DCU only) Signal that the motor is tensioner in lock state so increase allowable tracking error in locking direction

#define SFT_C_M3_LOCK	(1<<8)	///< (DCU only) Signal that the motor is tensioner in lock state so increase allowable tracking error in locking direction

/// This block is used to trigger conditionally compiled test code
#ifdef PACKET_BASED_SAFETY_TESTS
	#define SFT_C_WATCHDOG_TEST (1<<10)		///< Used to trigger an infinite while loop to test watchdog timer
	#define SFT_C_TRAJ_TEST 	(1<<11)		///< Used to trigger a trajectory generator error by causing an overshoot in traj_goal
	#define SFT_C_DATA_ABORT	(1<<12)		///< Force a data abort by mangling a pointer and dereferencing it
#endif

// safety output pin/DCU command status
#define SFT_S_BUZZER	(1<<0)	///< Reflects output value of signal
#define SFT_S_ENABLE	(1<<1)	///< Reflects output value of signal
#define SFT_S_RESET		(1<<2)	///< Reflects output value of signal
#define SFT_S_BRAKE		(1<<3)	///< Reflects output value of signal

#define SFT_S_VOLT_TEST	(1<<4)	///< Reflects output value of signal
#define SFT_S_CLR_ERR	(1<<5)	///< Reflects command received from DCU
// Values 6-9 reserved for future commands
// safety input pin status

#define SFT_S_VOLT_MON	(1<<10)	///< Value of the voltage monitor input pin
#define SFT_S_SFT_STAT	(1<<11)	///< Value of the SAFETY_OUT input pin

#define SFT_S_TEST_STAT	(1<<12)	///< Value of the SAFETY_TEST_OUT input pin
#define SFT_S_OUT_LIM	(1<<13)	///< Outer slide limit switch status
#define SFT_S_IN_LIM	(1<<14)	///< Inner slide limit switch status
//  LLIM an LIM bits share same location
#define SFT_S_OUT_LLIM	(1<<13)	///< Outer slide logical limit switch status determined by zero stored at limit switch press

#define SFT_S_IN_LLIM	(1<<14)	///< Inner slide logical limit switch status determined by zero stored at limit switch press

#define SFT_S_E_STOP	(1<<15) ///< Indicates the status of the E-stop switch as reflected by the E_STOP-OUT signal on the board

// global safety error flags
#define SFT_SE_R1		(1<<1)	///< Safety system detected R1 category error
#define SFT_SE_R2		(1<<2)	///< Safety system detected R2 category error
#define SFT_SE_R3		(1<<3)	///< Safety system detected R3 category error

#define SFT_SE_COMM_ER	(1<<4)	///< Inter CPU serial connection error rate exceeded limit
#define SFT_SE_COMM_TO	(1<<5)	///< Inter CPU serial connection timeout without receive
#define SFT_SE_USB_ER	(1<<6)	///< USB connection error rate exceeded limit
#define SFT_SE_USB_TO	(1<<7)	///< USB connection timeout without receive

#define SFT_SE_WDOG_RST	(1<<8)	///< Last reset was a watchdog reset
#define SFT_SE_VOLT_MON	(1<<9)	///< voltage monitor has detected an error
#define SFT_SE_SAFE_STATE	(1<<10)	///< Indicates to the DCU that the processor is in safe state
#define SFT_SE_MSEC_LIM (1<<11)	///< Indicates that the system has been running too long
#define SFT_SE_GLB_CLEAR_MASK	(SFT_SE_SAFE_STATE)	///< use &= with this mask to clear all clearable errors

/// Which bits are errors
#define SFT_SE_ERROR_MASK (SFT_SE_R2 | SFT_SE_R3 | SFT_SE_COMM_TO | SFT_SE_USB_TO | SFT_SE_WDOG_RST | SFT_SE_VOLT_MON | SFT_SE_MSEC_LIM)
/// Which bits are warning
#define SFT_SE_WARN_MASK  (SFT_SE_R1 | SFT_SE_COMM_ER | SFT_SE_USB_ER)
/// non critical bits which are allowed to be true when leaving safe state
#define SFT_SE_EXIT_SAFE_MASK	(SFT_SE_SAFE_STATE | SFT_SE_WARN_MASK)

// per motor safety error flags
#define SFT_SE_CURRENT	(1<<0)	///< Indicates a mismatch error between current sensors
#define SFT_SE_QUAD		(1<<1)	///< Indicates a mismatch error between encoders
#define SFT_SE_TRAJ		(1<<2)	///< Trajectory generator ouput too large of a change in goal
#define SFT_SE_POS		(1<<3)	///< Error between Goal and Encoder is too large

#define SFT_SE_FORCE	(1<<4)	///< Difference between Force goal and actual exceeded limit
#define SFT_SE_CUR_RT	(1<<5)	///< Rate of change of current exceeded limit
#define SFT_SE_MAX_CUR	(1<<6)	///< Maximum allowed motor current was exceeded (failure of h-bridge current limiting)
#define SFT_SE_MTR_CLEAR_MASK	(SFT_SE_QUAD)	///< use &= with this mask to clear all clearable errors

#define SFT_SE_MOTOR_MASK	(SFT_SE_CURRENT | SFT_SE_QUAD | SFT_SE_TRAJ | SFT_SE_POS | SFT_SE_FORCE | SFT_SE_CUR_RT | SFT_SE_MAX_CUR)
#define SFT_SE_BUZZ_MASK ( SFT_SE_R2 | SFT_SE_R3)	///< which errors to enable buzzer for

#endif /*STATE_H_*/
