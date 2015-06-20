#include <stdio.h>
#include <math.h>
#include "constants.h"
#include "logging.h"
#include "picomms.h"
#include "utils.h"
#include "datatypes.h"
#include "state.h"

double us_to_cm(int value) {
  return (double)value - 8.0; // sensor is about 8cm inside
}

double ir_front_to_cm(int value) {
  if (value < 35) return 200;
  return (6787 / (value - 3.0)) - 4;
}

double ir_side_to_cm(int value) {
  if (value < 80) return 40;
  return (2914 / (value + 5.0)) - 1;
}

double encoder_to_cm(int value) {
  return ((double)value) / TICKS_PER_CM;
}

// "8 10 12 14" -> [8, 10, 12, 14]
int extract_ints(int *ints, char *c) {
  int got_number = 0;
  int number = 0;
  int sign = 1;
  int len = 0;
  do {
    if (*c == '-' && !got_number && is_number(*(c+1))) {
      sign = -1;
    } else if (is_number(*c)) {
      number = (number * 10) + (*c - '0');
      got_number = 1;
    } else if (got_number) {
      ints[len++] = sign * number;
      got_number = 0;
      number = 0;
      sign = 1;
    }
    c++;
  } while (*c != '\0');
  return len;
}

Sensors structure_readings(int ints[]) {
  return (Sensors){
    .time     = millis(),
    .bumper   = {ints[0], ints[1]},
    .us       = us_to_cm(ints[2]),
    .ir_front = {ir_front_to_cm(ints[3]), ir_front_to_cm(ints[4])},
    .ir_side  = {ir_side_to_cm(ints[5]),  ir_side_to_cm(ints[6])},
    .encoder  = {encoder_to_cm(ints[7]),  encoder_to_cm(ints[8])}
  };
}

Sensors extract_readings(char *response) {
  int ints[9]; 
  extract_ints(ints, response+2);
  return structure_readings(ints);
}

double smooth(double previous, double current) {
  double diff = fabs(current - previous);
  double smoothing = min(1.05 / pow(1.16, diff), 0.99);
  return (current * (1 - smoothing)) + (previous * smoothing);
}

void update_state(char *response) {
  Sensors current = extract_readings(response);
  Sensors last = state.sensors;
  current.ir_front.left  = smooth(last.ir_front.left,  current.ir_front.left);
  current.ir_front.right = smooth(last.ir_front.right, current.ir_front.right);
  current.ir_side.left   = smooth(last.ir_side.left,   current.ir_side.left);
  current.ir_side.right  = smooth(last.ir_side.right,  current.ir_side.right);
  double dl = current.encoder.left - last.encoder.left;
  double dr = current.encoder.right - last.encoder.right;
  double angle_change = (dl - dr) / WHEELBASE;
  double half_angle_change = angle_change / 2.0;
  double r = dl / angle_change;
  double dh = (dl == dr) ? dl : (2*r - WHEELBASE) * sin(half_angle_change);
  double dt = (double)(current.time - last.time) / 1e3;
  state.pos.x += dh * sin(state.angle + half_angle_change);
  state.pos.y += dh * cos(state.angle + half_angle_change);
  state.angle = normalise_angle(state.angle + angle_change);
  state.wheelspeed.left  = dl / dt;
  state.wheelspeed.right = dr / dt;
  state.speed = (state.wheelspeed.left + state.wheelspeed.right) / 2.0;
  state.sensors = current;
  // StateList_add(state);
  log_state(state, response);
}

int speeds_are_zero() {
  return fabs(state.wheelspeed.left) < 1 && fabs(state.wheelspeed.right) < 1;
}