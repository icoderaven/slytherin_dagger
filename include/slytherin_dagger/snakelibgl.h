/*-------------------------------------------------------------------------------------*\
|                                                                                       |
|  snakelibgl.h                                                                         |
|                                                                                       |
|  Author: Stephen Tully													            |
|  Description: Functions dealing with registration between 3D models and the sensor.   |
|  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com                                 |
|                                                                                       |
\*-------------------------------------------------------------------------------------*/

#ifndef SNAKELIBGL_H
#define SNAKELIBGL_H

#define USE_MANUAL_DISP
#define SNAKELIB_LOGGING_EN

// get these guys from the config structure so only that has to be changed    
#define SNAKELIB_LINK_LENGTH 7.9f
#define SNAKELIB_LINK_RADIUS 6.0f
#define SNAKELIB_CABLE_RADIUS 5.5f
#define CABLE_ONE BOT_TENSIONER
#define CABLE_TWO UL_TENSIONER
#define CABLE_THREE UR_TENSIONER

/** @brief Used for window bounding areas */
#define WINDOW_INC 40
#define EXTRA_ROOM 12

#define MAX_GL_LISTS							10
#define MAX_TEXT_LENGTH							100

class SnakeLibGL {
public:
	static void closeUpShop(void);
    static void init(int argc, char* argv[]);
	static bool captureScreenshot(char *filename);  
    static bool main_loop(void);
	static void startNewLogFile(char *filename);
};

#endif SNAKELIBGL_H

