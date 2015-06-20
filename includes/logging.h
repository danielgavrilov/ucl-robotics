#ifndef LOGGING_H
#define LOGGING_H

#include "datatypes.h"

void init_logging();
void log_str(char*);
void log_msg(char*);
void log_msg_two_ints(char*, int, int);
void log_state(State, char *sensors);

#endif