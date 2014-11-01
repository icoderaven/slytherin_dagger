///// Constants specific to the slide, inner tensioner, and outer tensioner motors


#ifndef MOTOR_H
#define MOTOR_H

/**  Conversion constants between encoder ticks and millimeters */
#define SLIDE_MM_PER_TICK (1.17333333e-4)
#define INNER_MM_PER_TICK (3.60769556e-4)
#define OUTER_MM_PER_TICK (4.73324608e-4)


// velocity in (((mm/sec / mm_per_tick = ticks/sec) /1000 = ticks/ms) *4) = ticks / cycle
// acceleration
// distance in mm


#define OUTER_MAX_ACCELERATION (13.0 / OUTER_MM_PER_TICK / 1000.0 * 4)
#define OUTER_MAX_VELOCITY (61.8 / OUTER_MM_PER_TICK /1000.0 * 4)
#define OUTER_MAX_DISTANCE (20 / OUTER_MM_PER_TICK)

#define INNER_MAX_ACCELERATION (13.0 / INNER_MM_PER_TICK / 1000.0 * 4)
#define INNER_MAX_VELOCITY (46.7 / INNER_MM_PER_TICK / 1000.0 * 4)
#define INNER_MAX_DISTANCE (10.0 / INNER_MM_PER_TICK)

#define SLIDE_MAX_ACCELERATION (13.0 / SLIDE_MM_PER_TICK / 1000.0 * 4)
#define SLIDE_MAX_VELOCITY (15.58 / SLIDE_MM_PER_TICK / 1000.0 * 4)
#define SLIDE_MAX_DISTANCE (297.0 / SLIDE_MM_PER_TICK )

typedef struct motor_specification
{
	double max_distance;         // maximum distance motor should be commanded to travel
	double max_velocity;         // maximum velocity motor "
	double max_acceleration;     // max acceleration for this motor
	double counts_per_mm;		//  number of encoder counts in a mm
} mot_spec;

#endif /* MOTOR_H */
