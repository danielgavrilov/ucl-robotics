/*



*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "constants.h"
#include "datatypes.h"
#include "utils.h"
#include "picomms.h"
#include "async.h"
#include "state.h"
#include "logging.h"
#include "control.h"


// THE GOOD (ABSTRACT) PART
// =============================================================================

#define MAZE_SIZE 5     // 5x5 maze
#define BLOCK_SIZE 60.0 // in cm
#define TOTAL_NODES 25  // (SIZE)^2
#define MAX_DIST 999    // substitute for `infinity` in Dijkstra's

enum WallState {
  DOES_NOT_EXIST = -1,
  UNKNOWN = 0,
  EXISTS = 1
};

// Walls are determined as the average of a number of samples.
// `samples` stores the total samples
// `exists` stores the average probability there is a wall
struct Wall {
  int samples;
  double exists;
};

enum SensorType {
  // BUMPER_LEFT = 0,
  // BUMPER_RIGHT,
  ULTRASOUND = 0,
  IR_FRONT_LEFT,
  IR_FRONT_RIGHT,
  IR_SIDE_LEFT,
  IR_SIDE_RIGHT
};

int total_sensor_types = 5; // total obstruction types, used for looping

struct Node {
  struct Wall *walls[4]; // pointers to walls, since nodes share walls
  // fields below used for pathfinding (Dijkstra's)
  int through;     // the previous node index in the "shortest distance path"
  double distance; // total distance from designated node
  int done;        // whether `distance` is the minimum distance 
};

// All walls are stored in `walls` array. Nodes have pointers to this array.
struct Wall walls[2][MAZE_SIZE+1][MAZE_SIZE] = {};

// Nodes indices:
// +----+----+----+----+----+
// | 20 | 21 | 22 | 23 | 24 |
// +----+----+----+----+----+
// | 15 | 16 | 17 | 18 | 19 |
// +----+----+----+----+----+
// | 10 | 11 | 12 | 13 | 14 |
// +----+----+----+----+----+
// |  5 |  6 |  7 |  8 |  9 |
// +----+----+----+----+----+
// |  0 |  1 |  2 |  3 |  4 |
// +----+----+----+----+----+
struct Node nodes[TOTAL_NODES] = {};

int get_row(int i) {
  return i / MAZE_SIZE;
}

int get_col(int i) {
  return i % MAZE_SIZE;
}

int get_node_index(int row, int col) {
  return row*MAZE_SIZE + col;
}

Coords centre(int node) {
  int row = get_row(node);
  int col = get_col(node);
  return (Coords){BLOCK_SIZE*col, BLOCK_SIZE*row};
}

// weirdly named, but `wall_definitely(EXISTS, ...)` is quite readable
void wall_definitely(enum WallState value, struct Wall *wall) {
  wall->samples = 9999;
  wall->exists = value;
}

int exists(struct Wall wall) {
  return wall.samples >= 2 && round(wall.exists) == 1;
}

int does_not_exist(struct Wall wall) {
  return wall.samples >= 2 && round(wall.exists) == -1;
}

int is_unknown(struct Wall wall) {
  return !exists(wall) && !does_not_exist(wall);
}

// sets the known (outside) walls for the maze
void init_walls() {
  int i;
  for (i = 0; i < 4; i++) {
    wall_definitely(EXISTS, &walls[0][1][i]); // horizontal south walls
    wall_definitely(EXISTS, &walls[0][5][i]); // horizontal north walls
    wall_definitely(EXISTS, &walls[1][0][i+1]); // vertical west walls
    wall_definitely(EXISTS, &walls[1][4][i+1]); // vertical east walls
  }
  // the "box" around the starting node
  wall_definitely(DOES_NOT_EXIST, &walls[0][1][0]);
  wall_definitely(EXISTS, &walls[1][0][0]);
  wall_definitely(EXISTS, &walls[1][1][0]); 
  wall_definitely(EXISTS, &walls[0][0][0]); 
}

void init_nodes() {
  int i;
  for (i = 0; i < TOTAL_NODES; i++) {
    int col = get_col(i);
    int row = get_row(i);
    nodes[i] = (struct Node){
      .walls = {},
      .through = -1,
      .distance = MAX_DIST,
      .done = 0
    };
    nodes[i].walls[NORTH] = &walls[0][row+1][col];
    nodes[i].walls[EAST]  = &walls[1][col+1][row];
    nodes[i].walls[SOUTH] = &walls[0][row][col];
    nodes[i].walls[WEST]  = &walls[1][col][row];
  }
}

// Incomprehensible mess
void print_maze() {
  int i, col;
  for (i = 2*MAZE_SIZE+1; i > 0; i--) {
    for (col = 0; col < MAZE_SIZE; col++) {
      int row = (i / 2) - 1;
      if (i % 2) {
        printf("+");
        printf(
          exists(walls[0][row+1][col]) ? "---" :
          is_unknown(walls[0][row+1][col]) ? " · " :
          "   "
        );
        if (col == MAZE_SIZE-1) printf("+");
      } else {
        int current = get_node_index(row, col);
        double dist = nodes[current].distance;
        printf(
          exists(walls[1][col][row]) ? "|" : 
          is_unknown(walls[1][col][row]) ? "·" : 
          " "
        );
        if (dist < MAX_DIST) printf("%.1f", dist);
        else printf("   ");
        if (col == MAZE_SIZE-1) printf(
          exists(walls[1][col+1][row]) ? "|" : 
          is_unknown(walls[1][col+1][row]) ? "·" : 
          " "
        );
      }
    }
    printf("\n");
  }
}

// Given a node index and direction, returns the adjacent node in the direction.
// Returns -1 if there is no adjacent node.
int get_adjacent(int current, enum Direction direction) {
  if (direction == EAST && (current+1) % MAZE_SIZE == 0) return -1;
  if (direction == WEST && current % MAZE_SIZE == 0) return -1;
  switch (direction) {
    case NORTH: current += MAZE_SIZE; break;
    case EAST:  current += 1;         break;
    case SOUTH: current -= MAZE_SIZE; break;
    case WEST:  current -= 1;         break;
    default: return -1;
  }
  if (current < 0 || current > TOTAL_NODES-1) return -1;
  return current;
}

// Returns the direction between two adjacent nodes.
// Returns -1 if they are not adjacent.
int direction_between(int from, int to) {
  if (get_row(from) == get_row(to)) {
    if (from < to) return NORTH;
    if (from > to) return SOUTH;
  } else if (get_col(from) == get_col(to)) {
    if (from < to) return EAST;
    if (from > to) return WEST;
  }
  return -1;
}

// Resets everything set by Dijkstra's
void reset_distances() {
  int i;
  for (i = 0; i < TOTAL_NODES; i++) {
    nodes[i].done = 0;
    nodes[i].distance = MAX_DIST;
    nodes[i].through = -1;
  }
}

// Checks whether distances to all nodes are calculated
int all_done() {
  int i;
  for (i = 0; i < TOTAL_NODES; i++) {
    if (!nodes[i].done) return 0;
  }
  return 1;
}

int get_closest_pending_node() {
  int i;
  int min_dist = MAX_DIST+1;
  int min_index;
  for (i = 0; i < TOTAL_NODES; i++) {
    if (nodes[i].distance < min_dist && !nodes[i].done) {
      min_dist = nodes[i].distance;
      min_index = i;
    }
  }
  return min_index;
}

// Calculates the shortest distance to a node. Essentially it counts the minimum
// number of squares to get from one to the other, but punishes a change in
// direction. Doesn't punish again if the direction is then changed back to 
// initial, the idea being it can be cut across. 
double calc_distance(int c, int d) {
  int b = nodes[c].through;
  if (b != -1) {
    if (direction_between(b, c) != direction_between(c, d)) {
      int a = nodes[b].through;
      if (a != -1 && direction_between(a, b) == direction_between(c, d)) {
        return 1.0;
      } else {
        return 1.5; // 0.5 punishment (sort of randomly chosen)
      }
    }
  }
  return 1.0;
}

// Populates the distance to all the nodes adjacent to the given node.
void populate_adjacent(int current) {
  double current_dist = nodes[current].distance;
  int dir; // direction
  for (dir = 0; dir < 4; dir++) {
    struct Wall wall = *nodes[current].walls[dir];
    if (does_not_exist(wall) || is_unknown(wall)) {
      int adjacent = get_adjacent(current, dir);
      if (adjacent < 0) continue;
      double dist = current_dist + calc_distance(current, adjacent);
      if (nodes[adjacent].distance > dist) {
        nodes[adjacent].distance = dist;
        nodes[adjacent].through = current;
      }
    }
  }
}

// Populates distances from the given to all other nodes. This is the function 
// you're looking for to run Dijkstra's.
void populate_distances_from(int current) {
  reset_distances();
  nodes[current].distance = 0;
  while (!all_done()) {
    nodes[current].done = 1;
    populate_adjacent(current);
    current = get_closest_pending_node();
  }
}

int find_next_node_in_path(int current, int target) {
  if (nodes[target].distance >= MAX_DIST) {
    fprintf(stderr, "Node %d is unreachable\n", target);
    return -1;
  }
  while (target != current && nodes[target].through != -1 && nodes[target].through != current) {
    target = nodes[target].through;
  }
  return target;
}

// Returns the direction of the first occurring unknown wall.
// Returns -1 if all walls are known.
int get_unknown_wall(int node) {
  int i;
  for (i = 0; i < 4; i++) {
    if (is_unknown(*nodes[node].walls[i])) {
      return i;
    }
  }
  return -1;
}

// Checks whether all walls are known in the given node.
int knows_all_walls(int node) {
  return get_unknown_wall(node) == -1;
}


// THE BAD (PRACTICAL) PART
// =============================================================================

double target_dist = 17.0;

// Takes `n` ultrasound samples and returns the median. 
double get_accurate_us() {
  int n = 6;
  int i;
  double samples[n];
  for (i = 0; i < n && async_update(); i++) {
    samples[i] = state.sensors.us;
  }
  return get_median(samples, n);
}

// Returns the index of the closest node to the given coordinates.
int get_closest_node(Coords coords) {
  int i;
  double min_dist = MAX_DIST+1;
  double min_index;
  for (i = 0; i < TOTAL_NODES; i++) {
    double dist = get_distance(centre(i), coords);
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }
  return min_index;
}

void rotateto_direction(enum Direction direction) {
  rotateto(dir_to_rad(direction));
}

// Tries to rotate to the closest right angle relative to the maze.
void rotateto_right_angle() {
  rotateto_direction(rad_to_dir(state.angle));
}

void rotate_left() {
  rotateto_direction((rad_to_dir(state.angle) + 3) % 4);
}

void rotate_right() {
  rotateto_direction((rad_to_dir(state.angle) + 1) % 4);
}

int wall_in_front() {
  int node = get_closest_node(state.pos);
  int direction = rad_to_dir(state.angle);
  return exists(*nodes[node].walls[direction]);
}

int wall_on_left() {
  int node = get_closest_node(state.pos);
  int direction = (rad_to_dir(state.angle) + 3) % 4;
  return exists(*nodes[node].walls[direction]);
}

int wall_on_right() {
  int node = get_closest_node(state.pos);
  int direction = (rad_to_dir(state.angle) + 1) % 4;
  return exists(*nodes[node].walls[direction]);
}

void record_wall(struct Wall *wall, enum WallState value) {
  int samples = wall->samples;
  wall->exists = (wall->exists * samples + value) / (samples + 1);
  wall->samples = samples + 1;
}

Coords rotate_coords(Coords d, Coords a, double angle) {
  a.x +=  d.x * cos(angle) + d.y * sin(angle);
  a.y += -d.x * sin(angle) + d.y * cos(angle);
  return a;
}

// Returns the first wall that the given line intersects with.
// Line is specified by a coordinate and an angle.
enum Direction get_intersecting_wall(Coords offset, double angle) {
  angle = normalise_angle(angle);
  double t = deg_to_rad(15); // how close (angle) the reading needs to be to the corner to be ignored
  double d = BLOCK_SIZE / 2.0;
  double NW = get_angle((Coords){-d,  d}, offset);
  double NE = get_angle((Coords){ d,  d}, offset);
  double SW = get_angle((Coords){-d, -d}, offset);
  double SE = get_angle((Coords){ d, -d}, offset);
  if (angle > NE+t && angle <= SE-t) return EAST;
  if (angle > NW+t && angle <= NE-t) return NORTH;
  if (angle > SW+t && angle <= NW-t) return WEST;
  if (angle > SE+t || angle <= SW-t) return SOUTH;
  else return -1;
}

double get_wall_distance(enum Direction dir, Coords offset) {
  double centre_dist = BLOCK_SIZE / 2.0;
  switch (dir) {
    case NORTH: return centre_dist - offset.y;
    case EAST:  return centre_dist - offset.x;
    case SOUTH: return centre_dist + offset.y;
    case WEST:  return centre_dist + offset.x;
  }
  return 9999;
}

double get_expected_dist(enum Direction dir, Coords offset, double angle) {
  // printf("dir: %d; offset x: %.1f y: %.1f; angle: %.1f\n", dir, offset.x, offset.y, rad_to_deg(angle));
  double normal_dist = get_wall_distance(dir, offset);
  angle = normalise_angle(angle - dir_to_rad(dir));
  return normal_dist / cos(angle);
}

void record_reading(Coords pos, double angle, double actual_dist, double max_dist, double tolerance) {
  int node = get_closest_node(pos);
  Coords offset = Coords_diff(pos, centre(node));
  enum Direction dir = get_intersecting_wall(offset, angle);
  if (dir == -1) return;
  double expected_dist = get_expected_dist(dir, offset, angle);
  // printf("expected: %.1f actual: %.1f\n", expected_dist, actual_dist);
  enum WallState wall_state;
  if (actual_dist <= max_dist && fabs(actual_dist - expected_dist) <= tolerance)
    wall_state = EXISTS;
  else if (actual_dist > expected_dist + tolerance)
    wall_state = DOES_NOT_EXIST;
  else
    wall_state = UNKNOWN;
  if (wall_state != UNKNOWN) {
    record_wall(nodes[node].walls[dir], wall_state);
  } 
}


void record_sensor(enum SensorType type) {
  Coords offset;
  double angle = 0;
  double distance = 0;
  double max_distance = 0;
  double tolerance = 8.0; // TODO expose this maybe
  switch (type) {
    // case BUMPER_LEFT:
    // case BUMPER_RIGHT:
    //   offset.x = type == BUMPER_LEFT ? -BUMPER_X : BUMPER_X;
    //   offset.y = BUMPER_Y;
    //   break;
    case ULTRASOUND:
      offset.x = 0;
      offset.y = US_Y_OFFSET;
      distance = state.sensors.us;
      max_distance = US_MAX_READING;
      break;
    case IR_FRONT_LEFT:
      offset.x = -IR_FRONT_X;
      offset.y = IR_FRONT_Y;
      angle = deg_to_rad(-state.ir_angle.left);
      distance = state.sensors.ir_front.left;
      max_distance = IR_FRONT_MAX_READING;
      break;
    case IR_FRONT_RIGHT:
      offset.x = IR_FRONT_X;
      offset.y = IR_FRONT_Y;
      angle = deg_to_rad(state.ir_angle.right);
      distance = state.sensors.ir_front.right;
      max_distance = IR_FRONT_MAX_READING;
      break;
    case IR_SIDE_LEFT:
      offset.x = -IR_SIDE_X;
      offset.y = IR_SIDE_Y;
      angle = deg_to_rad(-90);
      distance = state.sensors.ir_side.left;
      max_distance = IR_SIDE_MAX_READING;
      break;
    case IR_SIDE_RIGHT:
      offset.x = IR_SIDE_X;
      offset.y = IR_SIDE_Y;
      angle = deg_to_rad(90);
      distance = state.sensors.ir_side.right;
      max_distance = IR_SIDE_MAX_READING;
      break;
  }
  angle = normalise_angle(angle + state.angle);
  Coords pos = rotate_coords(offset, state.pos, state.angle);
  record_reading(pos, angle, distance, max_distance, tolerance);
}

void record_walls() {
  // record only if robot is not driving fast and not rotating
  if (state.speed < 25 && fabs(state.wheelspeed.left - state.wheelspeed.right) < 5.0) {
    int sensor;
    for (sensor = 0; sensor < total_sensor_types; sensor++) {
      record_sensor(sensor);      
    }
  }
}

void plot_centres() {
  int i;
  for (i = 0; i < TOTAL_NODES; i++) {
    plot_point(centre(i).x, centre(i).y - 17.0);
  }
}

void find_starting_position() {

  double left_offset;
  double bottom_offset;
  double right_offset;

  rotateto_direction(WEST);
  left_offset   = get_accurate_us() - state.pos.x;

  rotateto_direction(SOUTH);
  bottom_offset = get_accurate_us() - state.pos.y;

  rotateto_direction(EAST);
  right_offset  = get_accurate_us() + state.pos.x;

  target_dist = (left_offset + right_offset) / 2.0;
  printf("target dist: %.1f\n", target_dist);
  rotateto_direction(NORTH);

  state.pos.x = left_offset - right_offset;
  state.pos.y = bottom_offset - target_dist;
}

// Calibrates the position of the robot in the current axis.
// e.g. if facing NORTH/SOUTH calibrates y axis.
void calibrate_current_axis() {

  int closest = get_closest_node(state.pos);
  int direction = rad_to_dir(state.angle);
  double offset = get_accurate_us() - target_dist;

  Coords c = centre(closest);

  switch (direction) {
    case NORTH: state.pos.y = c.y - offset; break;
    case EAST:  state.pos.x = c.x - offset; break;
    case SOUTH: state.pos.y = c.y + offset; break;
    case WEST:  state.pos.x = c.x + offset; break;
  }
}

void calibrate_angle() {

  double ir_distance = IR_FRONT_X * 2;
  set_ir_angles(0,0);
  wait_time(500);

  while (async_update()) {
    double right_angle = normalise_angle(dir_to_rad(rad_to_dir(state.angle)));
    double front_diff = state.sensors.ir_front.left - state.sensors.ir_front.right;
    double actual_offset = atan2(front_diff, ir_distance);
    double expected_offset = normalise_angle(state.angle - right_angle);
    double angle_diff = expected_offset - actual_offset;
    if (fabs(angle_diff) < deg_to_rad(1) && speeds_are_zero()) {
      set_voltages(0,0);
      state.angle = right_angle - angle_diff * 0.5;
      // printf("state angle: %.1f\n", rad_to_deg(state.angle));
      break;
    } else {
      // printf("expected: %.2f; actual: %.2f\n", expected_offset, actual_offset);
      rotateto_partial(state.angle - angle_diff);
    }
  }

  set_ir_angles(85,85);
  wait_time(300);
}

void try_calibrate() {

  rotateto_right_angle();

  if (wall_in_front()) {

    correct_front_dist(target_dist);
    calibrate_angle();
    calibrate_current_axis();

    if (wall_on_left() || wall_on_right()) {
      if (wall_on_left()) {
        rotate_left();
      } else {
        rotate_right();
      }
      correct_front_dist(target_dist);
      calibrate_current_axis();
    }
  }
}


// THE UGLY
// =============================================================================

int main() {

  if (LOGGING) init_logging();

  init_walls();
  init_nodes();
  reset_distances();
  populate_distances_from(0);
  print_maze();

  if (start_async(30)) {

    reset_origin();
    set_ir_angles(85,85);
    wait_time(300);

    // phase 1 - mapping the maze
    // -------------------------------------------------------------------------

    find_starting_position();

    int current;
    int next;
    int target = get_node_index(4,0);

    while (1) {
      current = get_closest_node(state.pos);
      if (current == target) {
        if (target == get_node_index(4,0)) target = get_node_index(4,3);
        else if (target == get_node_index(4,3)) target = get_node_index(1,3);
        else if (target == get_node_index(1,3)) target = get_node_index(0,0);
        else break;
        continue;
      } 
      populate_distances_from(current);
      print_maze();
      next = find_next_node_in_path(current, target);
      printf("current: %d, next: %d, target: %d\n", current, next, target);
      rotateto(get_angle(centre(next), state.pos));
      while (async_update() && !driveto_partial(SLOW, centre(next))) {
        record_walls();
        if (state.sensors.us <= target_dist+2) wait_time(80); // an elegant way to wait for a new sensor reading
        if (state.sensors.us <= target_dist) {
          set_voltages(0,0);
          break;
        }
        if (state.sensors.ir_front.left < 11.0 && wall_on_left()) {
          rotate_left();
          correct_front_dist(target_dist);
          calibrate_current_axis();
          rotate_right();
        } else if (state.sensors.ir_front.right < 11.0 && wall_on_right()) {
          rotate_right();
          correct_front_dist(target_dist);
          calibrate_current_axis();
          rotate_left();
        }
      }
      rotateto_right_angle();
      record_walls();
      try_calibrate();
      if (!knows_all_walls(current)) {
        printf("cannot determine wall %d in node %d.\n", get_unknown_wall(current), current);
        rotateto_direction(get_unknown_wall(current));
        record_walls();
      }
    }

    // phase 2 - race to end
    // -------------------------------------------------------------------------

    rotateto_direction(NORTH);

    current = get_closest_node(state.pos);
    target = get_node_index(4,3);
    populate_distances_from(current);
    next = find_next_node_in_path(current, target);
    print_maze();

    set_ir_angles(-45,-45);
    wait_time(1000);
    set_ir_angles(45,45);
    wait_time(1000);

    max_voltage = FAST_VOLTAGE;
    unsigned long started = millis();

    while (current != target) {
      while (async_update() && get_distance(centre(next), state.pos) > 56.0) {
        driveto_partial(FAST, centre(next));
      }
      int temp;
      temp = next;
      next = find_next_node_in_path(current, target);
      current = temp;
    }

    while (async_update() && get_distance(centre(target), state.pos) > 4.0) {
      driveto_partial(FAST, centre(target));
    }

    set_voltages(0,0);
    printf("finished in %.2f seconds.\n", (double)elapsed(started) / 1000.0);

  } else {
    fprintf(stderr, "Failed to start async mode.");
  }

  quit();
  return 0;
}