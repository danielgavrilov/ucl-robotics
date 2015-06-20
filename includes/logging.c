#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include "constants.h"
#include "utils.h"
#include "datatypes.h"
#include "logging.h"

#define FIFO_NAME "log"
#define BUFFER_SIZE 1024 

static int log_fifo;
static int logging = 0;
static int pipe_broken = 0;

void init_logging() {
  signal(SIGPIPE, SIG_IGN);
  mknod(FIFO_NAME, S_IFIFO | 0666, 0);
  system("nc -v -l 55444 <" FIFO_NAME " &");
  log_fifo = open(FIFO_NAME, O_WRONLY);
  logging = 1;
}

void log_str(char *c) {
  if (!logging) return;
  int count = strlen(c);
  int written;
  while (count > 0 && !pipe_broken) {
    written = write(log_fifo, c, count);
    if (written < 1) pipe_broken = 1;
    count -= written;
  }
}

void log_msg(char *msg) {
  char buf[BUFFER_SIZE];
  snprintf(buf, BUFFER_SIZE,
    "TIME %ld\n"
    "%s\n"
    "END\n",
    millis(),
    msg
  );
  log_str(buf);
}

void log_msg_two_ints(char *format, int a, int b) {
  char msg[40]; // 40 bytes ought to be enough for everyone
  snprintf(msg, 40, format, a, b);
  log_msg(msg);
}

void log_state(State state, char *sensors) {
  char buf[BUFFER_SIZE];
  snprintf(buf, BUFFER_SIZE, 
    "TIME %ld\n"
    "POS %f %f\n"
    "ANGLE %f\n"
    // "IR_ANGLES %d %d\n"
    // "VOLTAGES %d %d\n"
    "SENSORS %s"
    "END\n",
    state.sensors.time, 
    state.pos.x, state.pos.y, 
    rad_to_deg(state.angle), 
    // state.ir_angle.left, state.ir_angle.right,
    // state.voltage.left, state.voltage.right,
    sensors+2
  );
  log_str(buf);
}