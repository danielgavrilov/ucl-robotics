#ifndef UTILS_H
#define UTILS_H

#include "datatypes.h"

#ifndef max
  #define max(a, b) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
  #define min(a, b) ( ((a) < (b)) ? (a) : (b) )
#endif

unsigned long startTime;
unsigned long millis();
unsigned long elapsed(unsigned long ref);

double deg_to_rad(double);
double rad_to_deg(double);
double normalise_angle(double rad);
double get_angle(Coords a, Coords b);

void insertion_sort(double values[], int len);
double get_median(double values[], int len);

int clamp(int value, int min, int max);
int is_number(char);
int get_sign(double);
double is_between(double, double, double);

double normalise_turn(double turn);
double get_distance(Coords a, Coords b);

#endif