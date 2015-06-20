#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include "constants.h"
#include "picomms.h"
#include "datatypes.h"
#include "utils.h"
#include "state.h"
#include "logging.h"
#include "async.h"

#define COMMAND_LENGTH 50
#define RESPONSE_LENGTH 50
#define MAX_WAIT 200 // milliseconds to wait for ACK or ASYNC reply

Setpoint voltage;
Setpoint irangle;

static int socket;

typedef enum MessageType {
  REPLY = 1,
  ACK = 2,
  ASYNC = 3
} MessageType;

const char* async_read(char *response) {
  char recvbuf[RESPONSE_LENGTH];
  const char *reply_msg;
  memset(recvbuf, 0, RESPONSE_LENGTH);
  reply_msg = 0;
  while (reply_msg == 0) {
    reply_msg = recv_msg(recvbuf, RESPONSE_LENGTH);
  }
  strcpy(response, reply_msg);
  return reply_msg;
}

void quit(int sig) {
  set_voltages(0,0);
  stop_async();
  close(socket);
  printf("\nQuitting...\n");
  exit(0);
}

int start_async(int rate) {
  signal(SIGINT, quit);
  socket = connect_to_robot();
  int success = command_single("C ASYNC %d US IFLR ISLR MELR", rate);
  if (success) {
    state.ir_angle.left = state.ir_angle.right = -1;
    state.voltage.left  = state.voltage.right  = -1;
    set_voltages(0, 0);
    set_ir_angles(45, 45);
  }
  return success;
}

int receive_next(MessageType);

int async_update() {
  return receive_next(ASYNC);
}

void stop_async() {
  command_single("C ASYNC %d US IFLR ISLR MELR", 600000); // set rate to 10min
}

int valid_async_response(char *c) {

  int was_number = 0;
  int count = 0;

  if (c[0] != 'S' || c[1] != ' ') return 0;

  c += 2;

  do {
    if (is_number(*c) || (*c == '-' && !was_number && is_number(*(c+1))) ) {
      was_number = 1;
    } else if (was_number && (*c == ' ' || *c == '\n')) {
      was_number = 0;
      count++;
    } else {
      return 0;
    }
    c++;
  } while (*c != '\0');

  if (count == 9) return 1;
  else return 0;

}

// Control/communications

int send_command(char *command) {
  char buffer[COMMAND_LENGTH];
  // printf("sent: %s ... \n", command);
  sprintf(buffer, "%s\n", command);
  if (send_msg(buffer, strlen(buffer))) {
    // unsigned long sent = millis();
    int success = receive_next(ACK);
    // printf("%ldms\n", elapsed(sent));
    return success;
  }
  return 0;
}

int command_single(char *format, int param) {
  char buffer[COMMAND_LENGTH];
  sprintf(buffer, format, param);
  return send_command(buffer);
}

int command_double(char *format, int param1, int param2) {
  char buffer[COMMAND_LENGTH];
  sprintf(buffer, format, param1, param2);
  return send_command(buffer);
}

int receive_next(MessageType type_needed) {

  unsigned long begin = millis();
  MessageType type = 0;
  char response[RESPONSE_LENGTH];

  while (type != type_needed && elapsed(begin) < MAX_WAIT) {

    async_read(response);

    if (response[0] == '.') {
      type = ACK;
    } else if (valid_async_response(response)) {
      type = ASYNC;
      update_state(response);
    } else {
      type = REPLY;
      printf("Unexpected reply: %s\n", response);
      // TODO: not much you can do with the message now, is there?
    }
  }

  return (type == type_needed);
}

// Public control functions

int set_voltages(int left, int right) {
  left = clamp(left, -127, 127);
  right = clamp(right, -127, 127);
  Sides request = {left, right};
  if (!Sides_equal(state.voltage, request) ||
     (!Sides_equal(state.voltage, (Sides){0,0}) && elapsed(voltage.issued) > 1000)) {
    int success = command_double("M LR %d %d", left, right);
    if (success) {
      voltage.issued = millis();
      voltage.previous = state.voltage;
      state.voltage = request;
      log_msg_two_ints("VOLTAGE %d %d", left, right);
    }
    return success;
  }
  return 1;
}

int set_ir_angles(int left, int right) {
  Sides request = {left, right};
  if (!Sides_equal(state.ir_angle, request)) {
    int success = command_double("I LR %d %d", -left+45, right-45);
    if (success) {
      irangle.issued = millis();
      irangle.previous = state.ir_angle;
      state.ir_angle = request;
      log_msg_two_ints("IR_ANGLE %d %d", left, right);
    }
    return success;
  }
  return 1;
}


int ir_left_ready() {
  int diff = abs(state.ir_angle.left - irangle.previous.left);
  return elapsed(irangle.issued) > ((diff * 1000) / SERVO_SPEED);
}

int ir_right_ready() {
  int diff = abs(state.ir_angle.right - irangle.previous.right);
  return elapsed(irangle.issued) > ((diff * 1000) / SERVO_SPEED);
}

int ir_both_ready() {
  return ir_left_ready() && ir_right_ready();
}


int reset_origin() {
  #ifdef ROBOT
    return 1;
  #endif
  return send_command("C ORIGIN");
}

int plot_trail() {
  #ifdef ROBOT
    return 1;
  #endif
  return send_command("C TRAIL");
}

int plot_point(int x, int y) {
  #ifdef ROBOT
    return 1;
  #endif
  return command_double("C POINT %d %d", x, y);
}

// waiting

void wait_time(int ms) {
  unsigned int t = millis();
  while (async_update() && elapsed(t) < ms);
}

void wait_ticks(int ticks) {
  while (ticks-- > 0 && async_update());
}
