#ifndef DATATYPES_H
#define DATATYPES_H

#define LEFT 0
#define RIGHT 1

enum Direction {
  NORTH = 0,
  EAST,
  SOUTH,
  WEST
};

enum Direction rad_to_dir(double angle);
double dir_to_rad(enum Direction);

typedef struct Coords {
  double x;
  double y;
} Coords;

Coords Coords_diff(Coords, Coords);

typedef struct Sides {
  int left;
  int right;
} Sides;

int Sides_equal(Sides, Sides);

typedef struct Sides_f {
  double left;
  double right;
} Sides_f;

typedef struct Sensors {
  unsigned long time;
  Sides   bumper;
  double  us;
  Sides_f ir_front;
  Sides_f ir_side;
  Sides_f encoder;
  Sides   raw_encoder;
} Sensors;

typedef struct State {
  Coords  pos;
  double  angle;
  Sides   ir_angle;
  Sides   voltage;
  double  speed;
  Sides_f wheelspeed;
  Sensors sensors;
} State;

typedef struct Setpoint {
  Sides previous;
  unsigned long issued;
} Setpoint;

// typedef struct Obstacle {
//   Coords pos;
//   double probability;
// } Obstacle;

typedef struct PosList {
  Coords pos;
  struct PosList *next;
} PosList;

typedef struct StateList {
  State state;
  struct StateList *next;
  struct StateList *prev;
} StateList;

PosList *pos_list;
void PosList_add(Coords pos);
void PosList_reset();
PosList* PosList_closest(Coords);
PosList* PosList_skip(PosList*, int n);

StateList *statelist_head;
StateList *statelist_tail;
void StateList_add(State state);

#endif