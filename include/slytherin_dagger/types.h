/**
 * \file types.h
 * This file contains definitions of various shorthand types and constants
 *
 * \author Michael Schwerin
 * \date 2009.2.8
 */
#ifndef TYPES_H
#define TYPES_H


// define specific size integer values for the ARM7 arch
typedef signed char				int8;
typedef short int 				int16;
typedef int 					int32;
typedef long long int			int64;

typedef unsigned char 			uint8;
typedef unsigned short int		uint16;
typedef unsigned int 			uint32;
typedef unsigned long long int	uint64;

/// gives names for the states that the trajectory generator state machine can be in
enum traj_states_e {TRAJ_RAMP_UP=0, TRAJ_PLATEAU=1, TRAJ_RAMP_DOWN=2, TRAJ_COMPLETE=3};
/// give names to the possible sources and destinations for communications packets
enum src_dest_e {SD_BASE=0, SD_OUTER=1, SD_HOST=2};

typedef enum traj_states_e		traj_states_t;
typedef enum src_dest_e			src_dest_t;


// so we don't need stdlib.h
#ifndef NULL
#  define NULL 		((void *)0)
#endif

#endif
