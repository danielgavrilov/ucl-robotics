#ifndef ASYNC_H
#define ASYNC_H

int start_async(int millis);
int async_update();
void stop_async();
void quit();

int send_command(char *command);
int command_single(char *format, int param);
int command_double(char *format, int param1, int param2);

int set_voltages(int left, int right);
int set_ir_angles(int left, int right);
int reset_origin();
int plot_trail();
int plot_point(int, int);

int ir_left_ready();
int ir_right_ready();
int ir_both_ready();

void wait_time(int ms);
void wait_ticks(int ticks);

#endif