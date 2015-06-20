#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifdef __arm__
  #define ROBOT
#else
  #define SIMULATOR
#endif

#ifdef ROBOT
  #define TICKS_PER_CM 11.68
  #define WHEELBASE 23.45
#else
  #define TICKS_PER_CM 12.058 // 12.058 most accurate on sim
  #define WHEELBASE 22.5
#endif

#define LOGGING 0 // send logs to laptop
#define SLOW_VOLTAGE 55 
#define FAST_VOLTAGE 127

#define SERVO_SPEED 300 // rough rotational speed (deg/sec) for front IR servos
#define MAX_TURN 1.0

// sensor locations

#define BUMPER_X 12
#define BUMPER_Y 12

// ultrasound offset supposed to be 5, but 8 is subtracted from original value 
// to simplify other aspects of the code
#define US_Y_OFFSET 13 
#define US_MAX_READING 50

#define IR_FRONT_X 8
#define IR_FRONT_Y 8
#define IR_FRONT_MAX_READING 42

#define IR_SIDE_X 11
#define IR_SIDE_Y -6.5
#define IR_SIDE_MAX_READING 36

#endif