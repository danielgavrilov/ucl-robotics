#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "constants.h"
#include "datatypes.h"
#include "utils.h"
#include "async.h"
#include "state.h"
#include "logging.h"
#include "control.h"

// P(I?)D parameters
// -----------------------------------------------------------------------------

#define FOLLOW_IR_ANGLE 80
#define FOLLOW_PROPORTIONAL 0.03
#define FOLLOW_DERIVATIVE 0.015

#define DRIVETO_VOLTAGE 3.5
#define DRIVETO_PROPORTIONAL 0.6
#define DRIVETO_DERIVATIVE 0

#define DRIVETO_FAST_VOLTAGE 9.0
#define DRIVETO_FAST_PROPORTIONAL 0.43
#define DRIVETO_FAST_DERIVATIVE 0

// The maximum voltage for all the functions below.
// Can be changed from anywhere (included in header file).
int max_voltage = SLOW_VOLTAGE;

// Functions calculating turn
// =============================================================================

double follow_calc_turn(double left_dist, double right_dist) {
  static double last_error = 0;
  double error = left_dist - right_dist;
  double turn = error * -FOLLOW_PROPORTIONAL + (error - last_error) * -FOLLOW_DERIVATIVE;
  last_error = error;
  return normalise_turn(turn);
}

double driveto_calc_turn(double target, double current) {
  static double last_error = 0;
  double error = normalise_angle(target - current);
  double turn = error * DRIVETO_PROPORTIONAL + (error - last_error) * DRIVETO_DERIVATIVE; 
  last_error = error;
  return normalise_turn(turn);
}

double driveto_fast_calc_turn(double target, double current) {
  static double last_error = 0;
  double error = normalise_angle(target - current);
  double turn = error * DRIVETO_FAST_PROPORTIONAL + (error - last_error) * DRIVETO_FAST_DERIVATIVE; 
  last_error = error;
  return normalise_turn(turn);
}


// drive(turn, voltage)
// -----------------------------------------------------------------------------
// Drives straight or in a turn proportional to `turn`.
// Negative turn turns left, positive turns right, 0 is straight.

int drive(double turn, int voltage) {
  int left, right;
  turn = normalise_turn(turn);
  voltage = clamp(voltage, -max_voltage, max_voltage);
  // the wheel in direction of turn is slowed down
  if (turn < 0) {
    left = round(voltage * (turn * 2 + MAX_TURN));
    right = voltage;
  } else if (turn > 0) {
    left = voltage;
    right = round(voltage * (MAX_TURN - turn * 2));
  } else {
    left = right = voltage;
  }
  return set_voltages(left, right);
}

// drive_radius(side, radius)
// -----------------------------------------------------------------------------
// Drives at the given radius.

int drive_radius(int side, double radius) {
  double half_wb = WHEELBASE / 2;
  double ratio = (radius - half_wb) / (radius + half_wb);
  double turn = (MAX_TURN - ratio) / 2;
  if (side == LEFT) turn = -turn;
  return drive(turn, max_voltage);
}


// -----------------------------------------------------------------------------
// NOTE: functions ending with "_partial" adjust voltage and pass control back 
// to the rest of the program, which basically means they need to be called 
// repeatedly to have the intended effect. They return 1 when done, 0 otherwise.
// All _partial's have an impartial (lol!) buddy that does not pass control 
// until it's finished.
// -----------------------------------------------------------------------------


// driveto(mode, coordinates)
// -----------------------------------------------------------------------------
// Drives towards the specified coordinate.
// Assumes the robot can drive through walls (which it can), ignoring sensors.
// Accepts SLOW and FAST mode. FAST turns more smoothly (takes a wider curve)
// but stops abruptly, SLOW takes a straighter path (sharper turns) and brakes early.

int driveto_partial(enum DriveMode mode, Coords target) {
  double (*pd_func)(double,double) = (mode == FAST) ? driveto_fast_calc_turn : driveto_calc_turn;
  double dist = get_distance(target, state.pos);
  int voltage = round(dist * DRIVETO_VOLTAGE);
  if (dist < 1.0) {
    set_voltages(0,0);
    return speeds_are_zero();
  }
  double target_angle = get_angle(target, state.pos);
  double diff = normalise_angle(target_angle - state.angle);
  double turn;
  if (fabs(diff) > 3*M_PI/2 && dist < 10.0) {
    turn = pd_func(target_angle, state.angle+M_PI);
    drive(-turn, -voltage);
  } else {
    turn = pd_func(target_angle, state.angle);
    drive(turn, voltage);
  }
  return 0;
}

void driveto(enum DriveMode mode, Coords target) {
  while (async_update() && !driveto_partial(mode, target));
}


// driveto_straight(coordinates)
// -----------------------------------------------------------------------------
// Rotates on the spot to face the given coordinates, then drives to them in a 
// straight line. Ignores distance sensors.

void driveto_straight(Coords target) {
  double target_angle = get_angle(target, state.pos);
  rotateto(target_angle);
  driveto(SLOW, target);
}


// rotateto(angle)
// -----------------------------------------------------------------------------
// Rotates on the spot to the specified angle.
// The specified angle is the angle relative to the map, not the robot.

int rotateto_partial(double target_angle) {
  double error = normalise_angle(target_angle - state.angle);
  if (fabs(error) < 0.015) {
    set_voltages(0,0);
    return speeds_are_zero();
  }
  int voltage = abs((int)(error * 40.0)) + 1;
  int sign = get_sign(error);
  drive(sign, voltage);
  return 0;
}

void rotateto(double target_angle) {
  while (async_update() && !rotateto_partial(target_angle));
}


// follow(side, distance from wall, minimum front distance required to stop)
// -----------------------------------------------------------------------------
// Follows wall on the specified side, at the given distance, until it reaches
// the minimum front distance specified.

int follow_partial(int side, double dist_from_wall, double dist_before_stop) {

  int voltage;

  Sensors current = state.sensors;

  int left_ir_angle = 0;
  int right_ir_angle = 0;

  if (side == LEFT) {
    left_ir_angle = FOLLOW_IR_ANGLE;
  } else {
    right_ir_angle = FOLLOW_IR_ANGLE;
  }

  set_ir_angles(left_ir_angle, right_ir_angle);

  if (ir_both_ready()) {

    double left_front_side  = current.ir_front.left * sin(deg_to_rad(left_ir_angle));
    double left_front_front  = current.ir_front.left * cos(deg_to_rad(left_ir_angle));
    double right_front_side = current.ir_front.right * sin(deg_to_rad(right_ir_angle));
    double right_front_front = current.ir_front.right * cos(deg_to_rad(right_ir_angle));
    double avg_front_dist = (left_front_front + right_front_front) / 2.0;

    double turn = (side == LEFT) ? follow_calc_turn(left_front_side, dist_from_wall) :
                                   follow_calc_turn(dist_from_wall, right_front_side);

    if (current.us < 30) {
      voltage = current.us * 1.0;
    } else {
      voltage = max_voltage;
    }

    if (current.us < dist_before_stop || avg_front_dist < 5) {
      set_voltages(0,0);
      return speeds_are_zero();
    }

    drive(turn, voltage);
  }

  return 0;
}

void follow(int side, double dist_from_wall, double dist_before_stop) {
  while (async_update() && !follow_partial(side, dist_from_wall, dist_before_stop));
}


// correct_front_dist(target distance)
// -----------------------------------------------------------------------------
// Moves the robot forward/backwards so that the forward distance from a wall is
// the given target distance.

int correct_front_dist_partial(double target_distance) {
  double diff = state.sensors.us - target_distance;
  if (fabs(diff) < 1.0) {
    set_voltages(0,0);
    return speeds_are_zero();
  } else {
    int voltage = diff * 2.0;
    set_voltages(voltage, voltage);
    return 0;
  }
}

void correct_front_dist(double target_distance) {
  while (async_update() && !correct_front_dist_partial(target_distance));
}