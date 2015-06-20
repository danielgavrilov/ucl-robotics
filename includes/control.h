#ifndef CONTROL_H
#define CONTROL_H

#include "datatypes.h"

enum DriveMode {
  SLOW = 0,
  FAST = 1
};

int max_voltage;

int drive(double turn, int voltage);
int drive_radius(int side, double radius);

int driveto_partial(enum DriveMode, Coords);
void driveto(enum DriveMode, Coords);
void driveto_straight(Coords target);

int rotateto_partial(double);
void rotateto(double);
// void rotateto_unroll(double);

int follow_partial(int side, double dist_from_wall, double dist_before_stop);
void follow(int side, double dist_from_wall, double dist_before_stop);

int correct_front_dist_partial(double distance);
void correct_front_dist(double distance);

#endif