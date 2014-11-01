
#ifndef LOG_STATE_H
#define LOG_STATE_H

#include <stdio.h>
#include "../communication_interface/usbcomm.h"

FILE* init_state_log(char *fname);
void log_state_packet(FILE* state_log, sState *log_state);



#endif /* LOG_STATE_H */

