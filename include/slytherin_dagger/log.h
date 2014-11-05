/** @file log.h
 *	@brief Function prototypes for the log component (section10).
 *
 *	@author Cornell Wright
 */


#ifndef LOG_H
#define LOG_H
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#endif
#include <stdio.h>
#include "guided_mode_console.h"

/** @brief Used to specify the INFO log level */
#define LOG_INFO 0

/** @brief Used to specify the WARNING log level */
#define LOG_WARNING 1

/** @brief Used to specify the ERROR log level */
#define LOG_ERROR 2

/** @brief File pointer for the log file. This will be opened by log_init(). */
extern FILE *log_file;

// Function prototypes:
void log_init(void);
void log_close(void);
char *str_time(void);
char *filename_time(void);

/** @brief Set to nonzero of an error has been logged since the software was
 *	initialized.
 */
extern int log_error_this_run;

/** @brief Set to nonzero of a warning has been logged since the software was
 *	initialized.
 */
extern int log_warning_this_run;


/** @brief Print a message to the log.
 *
 *	@param level Log level - should be either LOG_INFO, LOG_WARNING, or
 *			LOG_ERROR.
 *	@param format_str printf style format string for the log entry.
 *	@param ... variables to be substituted into the format string, if necessary.
 */
#define log_printf(level, format_str, ...) 									\
	/* The whole body of the macro is in a do-while(0) loop in order to */	\
	/* "swallow the semicolon." */											\
	do {																	\
		char __log_line[200];                                               \
		char __log_msg[200];                                                \
		char __log_level_str[30];                                           \
		/* If the log has not been initialized, we will print an error	*/	\
		/* and call log_init("")										*/	\
		if (log_file == NULL) {												\
			printf("ERROR:    log_printf() called before log_init(). "		\
				"Calling log_init(\"\") now.\n");							\
			fprintf(stderr, "ERROR:    log_printf() called before "			\
				"log_init(). Calling log_init(\"\") now.\n");				\
			log_init();														\
		}																	\
																			\
		/* Print the time */												\
		char *myTime = str_time();											\
		sprintf_s(__log_line, sizeof(__log_line), "%s ", myTime);			\
		if (myTime) free(myTime);											\
																			\
		/* Prefix the entry into the log with INFO, WARNING, or ERROR	*/	\
		/* depending on log_level. If an invalid leveil is given, print */	\
		/* an error to the log and assume the log level is ERROR. 		*/	\
		switch (level) {													\
			case LOG_INFO:                                                  \
				strncpy_s(__log_level_str, sizeof(__log_level_str), "INFO:    ", sizeof(__log_level_str));\
				break;                                                      \
			case LOG_WARNING:                                               \
				log_warning_this_run = 1;                                   \
				strncpy_s(__log_level_str, sizeof(__log_level_str), "WARNING: ", sizeof(__log_level_str));\
				break;                                                      \
			case LOG_ERROR:                                                 \
				log_error_this_run = 1;                                     \
				strncpy_s(__log_level_str, sizeof(__log_level_str), "ERROR:   ", sizeof(__log_level_str));\
				break;                                                      \
			default:                                                        \
				log_error_this_run  = 1;                                    \
				fprintf(log_file,										    \
				"ERROR:   Invalid log level %d specified at __LINE__ in "	\
				"__FILE__. Assuming ERROR level for next log entry.\n"		\
				"ERROR:   ",												\
				level);														\
				break;														\
		}																	\
																			\
		/* Write the actual entry to the log and flush it in hopes that */	\
		/* it will be written more immediately.							*/	\
		/* Requirement 10.3	*/												\
		sprintf_s(__log_msg, sizeof(__log_msg), format_str, ##__VA_ARGS__);	\
		myTime = str_time();												\
		sprintf_s(__log_line, sizeof(__log_line), "%s %s%s", myTime,		\
				__log_level_str, __log_msg);                                \
		if (myTime) free(myTime);											\
                                                                            \
		/* Have the UI print the line */                                    \
		new_log_line(__log_line);                                           \
                                                                            \
		/* Now write it to the file */                                      \
		if (fprintf(log_file, __log_line) < 0 ||             \
				fflush(log_file) != 0) {                                    \
			new_log_line("UNABLE TO WRITE TO LOG FILE!\n");                 \
		}                                                                   \
                                                                            \
	} while (0)


#endif /* LOG_H */

