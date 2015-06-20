#ifndef STATE_H
#define STATE_H

State state;
double wheelbase_adjustment;

void update_state(char *response);
int speeds_are_zero();

#endif