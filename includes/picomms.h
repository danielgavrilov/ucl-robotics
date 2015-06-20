/*

Code below provided by UCL (as part of COMP105P Robotics Programming).
Not written or owned by me.

*/

#ifndef PICOMMS_H
#define PICOMMS_H

#define LEFT 0
#define RIGHT 1

// function prototypes for picomms.
int send_msg(char* msg, int len);
void initialize_robot();
int connect_to_robot();
const char* recv_msg(char *buf, int bufsize);
void clear_input_stream();
int gp2d12_to_dist(int ir);
int gp2d120_to_dist(int ir);
void set_motor(int side, int speed);
void set_motors(int speed_l, int speed_r);
void set_asr(int flag); /*set auto-speed regulation; 0=off, 1=on*/
void set_ir_angle(int side, int angle);
void send_text(char *text);
int one_sensor_read(char *sensorname, int *value);
int get_front_ir_dist(int side);
int get_side_ir_dist(int side);
int get_us_dist();
int two_sensor_read(char *sensornames, int *value1, int *value2);
int get_front_ir_dists(int *leftdist, int *rightdist);
int get_side_ir_dists(int *leftdist, int *rightdist);
int check_bump(int side);
int check_bumpers(int *lbump, int *rbump);
int get_voltage();
int get_motor_encoders(int *leftenc, int *rightenc);
void log_trail();
void set_origin();
void set_point(int x, int y);

#endif