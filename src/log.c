/** @file log.c
 *	@brief Implementation of the log component (section 10).
 *
 *	@author Cornell Wright
 */

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <malloc.h>
#include "log.h"
#include "CardioError.h"
#include "superCommon.h"
#include "ErrorData.h"

/** @brief File handle for the log file. Initialized to NULL both to create
 *	a strong binding and so that we can tell if the log file is open or not.
 */
FILE *log_file = NULL;

/** @brief Set to nonzero of an error has been logged since the software was
 *	initialized.
 */
int log_error_this_run = 0;

/** @brief Set to nonzero of a warning has been logged since the software was
 *	initialized.
 */
int log_warning_this_run = 0;


/** @brief Used to hold the time in format used for logfile names */
static char filename_time_str[50];


/** @brief Initializes logging mostly by opening the log file. Prints any
 *	errors to stdout and stderr.
 */
void log_init(void) {
	char fname[100];

	// Make sure that we aren't attempting to double-initialize logging.
	if (log_file != NULL) {
		fprintf(log_file, "ERROR:    log_init() called more than once without "
			"calling log_close in between. Ignoring latest call.\n");
		return;
	}

	// Figure out what the filename should be
	sprintf_s(fname, sizeof(fname), "logs/%s_logfile.txt", filename_time());

	// Make sure error and warning flags are clear
	log_error_this_run = 0;
	log_warning_this_run = 0;


	// Open the actual log. If we're given a NULL filename, don't try to
	// open it and pretend we were passed "".
	if (fname != NULL) {
		// Requirements 10.1, 10.2
		log_file = NULL;
		fopen_s(&log_file,fname, "a");
	}
	else {
		fname[0] = '\0';
	}
	

	// If we failed to open the log file, print a message and use stderr
	// instead.
	if (log_file == NULL) {
        SetErrorBit(M_DCU, DCU_WRN_LOG_FILE_INIT);
/*		printf("WARNING: Cannot open log file %s. Using stderr instead.\n",
				fname);
		fprintf(stderr, "WARNING: Cannot open log file %s. Using stderr"
				"instead.\n", fname); */

		log_file = stderr;
	}

	log_printf(LOG_INFO, "====== Logging initialized ======\n");
}

/** @brief Closes the log file. Prints a warning if the log file is already
 *	closed.
 */
void log_close(void) {

	// If we attempt to double close, just print a message.
	if (log_file == NULL) {
		printf("WARNING: log_close() called without calling log_init.\n");
		fprintf(stderr, "WARNING: log_close() called without calling "
				"log_init.\n");
		return;
	}

	log_printf(LOG_INFO, "====== Logging ended ======\n");

	// Otherwise close the log, assuming it's an actual file.
	if (log_file != stderr && log_file != NULL) {
		fclose(log_file);
	}

	// Set it to NULL we that we know we actually closed it.
	log_file = NULL;
} 


/** @brief Returns a pointer to the string representation of the current time
 *	and date. Note that this string is allocated statically, so multiple calls
 *	to asctime, ctime, or this function will result in the string being
 *	overwritten.
 *
 *	@return Pointer to a string representation of the current date and time.
 */
char *str_time(void)
{
	time_t current_time;
	char *time_str = (char *)malloc(sizeof(char)*100);
	char *newline_pos;

	// Get the current time
	time(&current_time);

	// Convert it to a string
	ctime_s(time_str, 100, &current_time);

	// Remove the \n
	newline_pos = strstr(time_str, "\n");
	if (newline_pos != NULL) {
		*newline_pos = '\0';
	}


	// Return a pointer to the string representation of it
	return time_str;
}

/** @brief Returns a pointer to a string representation of the current date and 
 *	time that will be useful in a file name. For example, the time/date "Tue SepJ
 *	22 18:43:51 2009" would become "2009-09-22_18.43.51". The pointer to the
 *	string that is returned is statically allocated, so date should be copied
 *	from it before another call to the function.
 *
 *	@return Pointer to a formated string representation of the current date and
 *	time.
 */
char *filename_time(void)
{
	time_t current_time;

	// Get the current time
	time(&current_time);

	// Convert it to a properly formated string representation
	struct tm myTM;
	localtime_s(&myTM, &current_time);
	strftime(filename_time_str, sizeof(filename_time_str), "%Y-%m-%d_%H.%M.%S", &myTM);

	// Return a pointer to that string
	return filename_time_str;
}
     
                                      
     
        
      




