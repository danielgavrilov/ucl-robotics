#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "constants.h"
#include "datatypes.h"
#include "utils.h"


// Timekeeping

unsigned long startTime = 0;

unsigned long millis() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (startTime == 0) startTime = tv.tv_sec;
  return (tv.tv_sec - startTime) * 1000UL + (tv.tv_usec) / 1000UL;
}

unsigned long elapsed(unsigned long ref) {
  return millis() - ref;
}


// Angles

double deg_to_rad(double deg) {
  return deg * M_PI / 180;
}

double rad_to_deg(double rad) {
  return rad * 180 / M_PI;
}

double normalise_angle(double rad) {
  rad = fmod(rad, 2*M_PI);
  if (rad > M_PI) rad -= 2*M_PI;
  if (rad < -M_PI) rad += 2*M_PI;
  return rad;
}

double get_angle(Coords a, Coords b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return normalise_angle(atan2(dx, dy));
}


// Arrays

void insertion_sort(double values[], int len) {
  if (len < 2) return; 
  int i, j;
  double temp;
  for (i = 1; i < len; i++) {
    j = i;
    while (j > 0 && values[j] < values[j-1]) {
      temp = values[j];
      values[j] = values[j-1];
      values[j-1] = temp;
      j--;
    }
  }
}

double get_median(double values[], int len) {
  insertion_sort(values, len);
  if (len % 2) return values[len/2];
  else return (values[len/2] + values[len/2 - 1]) / 2.0;
}


// Numbers

int clamp(int value, int lower, int upper) {
  return max(min(value, upper), lower);
}

int is_number(char c) {
  return (c >= '0' && c <= '9');
}

int get_sign(double num) {
  return (num < 0) ? -1 : 1;
}

double is_between(double a, double b, double c) {
  return a <= b && b <= c;
}


// Misc

double normalise_turn(double turn) {
  if (turn < -MAX_TURN) return -MAX_TURN;
  if (turn > MAX_TURN) return MAX_TURN;
  return turn;
}

double get_distance(Coords a, Coords b) {
  double dx = fabs(a.x - b.x);
  double dy = fabs(a.y - b.y);
  return sqrt(dx*dx + dy*dy);
}
