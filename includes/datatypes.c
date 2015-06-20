#include <math.h>
#include <stdlib.h>
#include "utils.h"
#include "datatypes.h"

// Direction

// Converts an angle to closest direction (NORTH, EAST, WEST or SOUTH)
enum Direction rad_to_dir(double angle) {
  if (angle > (M_PI/4) && angle <= (3*M_PI/4)) return EAST;
  if (fabs(angle) >= (3*M_PI/4)) return SOUTH;
  if (angle < (-M_PI/4) && angle >= (-3*M_PI/4)) return WEST;
  return NORTH;
}

double dir_to_rad(enum Direction direction) {
  return direction * (M_PI/2);
}


// Coords

Coords Coords_diff(Coords a, Coords b) {
  return (Coords){a.x - b.x, a.y - b.y};
}


// Sides

int Sides_equal(Sides a, Sides b) {
  return (a.left == b.left) && (a.right == b.right);
}


// PosList

PosList *pos_list = NULL;

void PosList_add(Coords pos) {
  PosList *node = malloc(sizeof(PosList));
  node->pos = pos;
  node->next = pos_list;
  pos_list = node;
}

void PosList_destroy(PosList* list) {
  if (list->next != NULL) PosList_destroy(list->next);
  free(list);
}

void PosList_reset() {
  if (pos_list == NULL) return;
  PosList_destroy(pos_list);
  pos_list = NULL;
} 

PosList* PosList_closest(Coords pos) {
  PosList* current = pos_list;
  PosList* closest = pos_list;
  double closest_dist = get_distance(current->pos, pos);
  while (current != NULL) {
    double dist = get_distance(current->pos, pos);
    if (dist < closest_dist) {
      closest = current;
      closest_dist = dist;
    }
    current = current->next;
  }
  return closest;
} 

PosList* PosList_skip(PosList* current, int n) {
  while (current->next != NULL && n > 0) {
    current = current->next;
    n--;
  }
  return current;
}

// PosList* PosList_find_next(PosList* current, double min_path_dist, double min_dist) {
//   double path_dist = 0;
//   while (current->next != NULL && (path_dist < min_path_dist || get_distance() < min_dist)) {

//   }
// }

StateList *statelist_head = NULL;
StateList *statelist_tail = NULL;

void StateList_add(State state) {
  StateList *node = malloc(sizeof(StateList));
  node->state = state;
  node->prev = statelist_head;
  if (node->prev != NULL) {
    node->prev->next = node;
  } else {
    statelist_tail = node;
  }
  statelist_head = node;
}