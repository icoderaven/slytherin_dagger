/** @file main.c
 *	@brief Initializes everybody and contains the main loop.
 *
 *	@author Cornell Wright
 */
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include "log.h"
#include "config_defaults.h"
#include "motor_abs.h"

#ifdef USE_MANUAL_MODE
#include "manual_mode_gtk.h"
#endif /* USE_MANUAL_MODE */

#include "guided_mode_console.h"
#include "guided_motions.h"
#include <windows.h>
#include "gui_main.h"
#include "gui_support.h"
#include "gui_callbacks.h"
#include <gtk/gtk.h>
#include "keithley.h"
#include "superCommon.h"
#include "../system/crc32.h"
#include "../snakeVisualizer/snakelibgl.h"

#define BUFSIZE 1024

// Prototype for our main loop function
int main_loop(void *dummy);

// Subroutine to perform CRC on the execcutable image
int Exe_Crc(void){
    FILE *fp;
    char pbuf[BUFSIZE];
    char str[30];
    unsigned int k,count,val,workCRC;
    int status=1;
    
    //read in the CRC value for the executable file
    fp = NULL;
	fopen_s(&fp,"crcfile.txt","r");
    if(!fp){
        printf("Can't open crcfile.txt. Exiting application\n");
        Sleep(5000);
        status = 0;
        return status; 
    }
    fscanf_s(fp,"%s",str);
    val = atol(str);
    fclose(fp);
    fp = NULL;
	fopen_s(&fp,"dcu.exe","r");
    if(!fp){
        printf("Can't open executable file. Exiting application\n");
        Sleep(5000);
        status = 0;
        return status; 
    }
    crc32_start(&workCRC);
    // cycle until end of file reached
    while(!feof(fp)){
        count = fread(pbuf,sizeof(char),BUFSIZE,fp);
        if(ferror(fp)){
             break;
        }
        for(k = 0; k < count; k++){
           crc32_update(pbuf[k],&workCRC);
        }  
    }
    crc32_finish(&workCRC);
    if(val != workCRC){
           status = 0;
		   printf("Exe crc=%u, but crcfile.txt=%u\n", workCRC, val);
		   Sleep(10000);
    }
    else{
         status = 1;
    }
    fclose(fp);
    return status;
}



/** @brief C main function */
int main(int argc, char **argv) {
//perform CRC on executable file
    crc32_build_table();

    
	
/*\    if(!Exe_Crc()){
         printf("CRC error in executable file. Exit Application\n");
         return 0;
    }*/
	


	gui_init(argc, argv, main_loop);

	// Initialize logging
	log_init();

	// Initialize the motor abstraction component
	motor_abs_init(COM_PORT_NAME);

	// Initialize the guided motions component
	guided_motions_init();

	// Initialize guided mode
	guided_mode_init();
	
	//initialize Keithley card and supervisor
	//must come after motor_abs_init
	InitDCUIO();
	RunSupervisor(INIT_SUPERVISOR);

	// Initialize manual mode
#ifdef USE_MANUAL_MODE
	manual_init(argc, argv, main_loop);
#endif /* USE_MANUAL_MODE */

	SnakeLibGL::init(argc, argv);

     	while (main_loop(NULL) != 0) ;

	SnakeLibGL::closeUpShop();

	// Once we make it here we're quitting, so close the log file
	log_close();

	gui_quit();

	return 0;
}


/** @brief Main loop function. This should be run repeatedly to run every thing
 *	in the system.
 *
 *	@param dummy Does nothing. This is necessary to allow the main loop to be
 *	run as an idle function.
 *
 *	@return 0 if the main loop should be no longer run, nonzero if it should
 *	no longer be run.
 */
int main_loop(void *dummy) {

#ifdef USE_MANUAL_MODE
	// Depending on which page of the notebook is up, run the appropriate
	// main loop for those components
	if (get_notebook_page("main_notebook") == 1) {
#endif /* USE_MANUAL_MODE */
		guided_motions_main_loop();
		guided_mode_main();
		RunSupervisor(RUN_SUPERVISOR);

#ifdef USE_MANUAL_MODE
	}
	else {
		manual_main();
	}
#endif /* USE_MANUAL_MODE */

	// Run any gui stuff if it exists
	while (gtk_events_pending()) {
		gtk_main_iteration();
	}

	// Run the motor abstraction main which is pretty much just com stuff
	mot_main();

	// If we have requested a quit, quit called will be set to true. In that
	// case we should return false
	if (!SnakeLibGL::main_loop())
		quit_called = 1;

	return !quit_called;
}



