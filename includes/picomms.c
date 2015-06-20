/*

Code below provided by UCL (as part of COMP105P Robotics Programming).
Not written or owned by me.

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <assert.h>
#include <sys/select.h>
#include "picomms.h"

#define LEFT 0
#define RIGHT 1
int ir_angle[2] = {0,0};
static int sock = -1;
int error_state = 0;
#define READBUFLEN 80

void re_initialize_robot();
void check_errors();

int send_msg(char* msg, int len) {
  if (write(sock, msg, len) <= 0) {
    /* the write failed - likely the robot was switched off - attempt
       to reconnect and reinitialize */
    printf("send failed - reconnecting\n");
    connect_to_robot();
    re_initialize_robot();
    return 0;
  } else {
    return 1;
  }
}

/* strcpy isn't safe to use with overlapping data */
/* this one is safe, so long as you're copying to the left*/
void safestrcpy(char *s1, char *s2, int len) {
  int i;
  assert(s1 < s2 || s1 > s2+len);
  /* copy up to len bytes */
  for(i = 0; i < len; i++) {
    /* don't copy past the end of s2 */
    if(*s2 == '\0')
      break;
    *s1++ = *s2++;
  }
  *s1 = '\0';
}

/* add received bytes to a buffer */
/* received bytes are in tmpbuf and are null terminated */
/* we want to add them to permbuf.  The number of bytes already in
   permbuf is passed in using permbuflen, and we modify this with new
   number of bytes */
void add_to_buffer(char *permbuf, unsigned int *permbuflen, char *tmpbuf) {
  char *end = permbuf + *permbuflen;
  int tmpbuflen = strlen(tmpbuf);
  assert(tmpbuflen < READBUFLEN);

  /* is there space?*/
  if (tmpbuflen + *permbuflen + 1 >= READBUFLEN) {
    /* we've no more space for the commands - looks like someone forgot some newlines! */
    /* clear the old stuff out of permbuf - it's no use to us */
    printf("cleaing old stuff\n");
    strcpy(permbuf, tmpbuf);
    return;
  }
  
  safestrcpy(end, tmpbuf, tmpbuflen);
  *permbuflen += tmpbuflen;
}

/* extract a newline-terminated reply from a buffer */
const char* extract_reply(char *buf) {
  static char replybuf[READBUFLEN];
  int len;
  char *newline = strchr(buf, '\n');
  
  if (newline == NULL) {
    /* we've not got a newline, so no finished command here */
    return NULL;
  }
  *newline++ = '\0';
  strcpy(replybuf, buf);
  len = strlen(replybuf);
  if (replybuf[len-1] == '\r') {
    replybuf[len-1] = 0;
    len--;
  }
  if (replybuf[len-1] != '\n') {
    replybuf[len] = '\n';
    replybuf[len+1] = '\0';
    len++;
  }
  safestrcpy(buf, newline, strlen(newline));
  return replybuf;
}

/* receive a message from the robot.  This will return the first full
   line from the response stream - no guarantee it's the response
   you're expecting though */
/* The message is returned in a static buffer, so don't call this
   again until you're done with the previous response or it will be overwritten. */
const char* recv_msg(char *buf, int bufsize) {
  int val;
  fd_set read_fdset;
  fd_set except_fdset;
  struct timeval tv;
  char readbuf[bufsize];
  unsigned int totalbytes;

  /* Is there already a reply in the buffer from before? (can happen if
     we get multiple lines in one read) */
  const char *reply_msg = extract_reply(buf);
  if (reply_msg != NULL)
    return reply_msg;

  while (1) {
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    FD_ZERO(&read_fdset);
    FD_ZERO(&except_fdset);
    FD_SET(sock, &read_fdset);
    FD_SET(sock, &except_fdset);
    if (select(sock+1, &read_fdset, NULL, &except_fdset, &tv) == 0) {
      /* we've waited 1 second and got no response - too long - conclude
   the socket is dead */
      printf("timed out waiting response\n");
      //connect_to_robot();
      //initialize_robot();
      return 0;
    }
    if (FD_ISSET(sock, &except_fdset)) {
      connect_to_robot();
      re_initialize_robot();
      return 0;
    }
  
    assert(FD_ISSET(sock, &read_fdset));
    val = read(sock, readbuf, bufsize-1);
    if (val > 0) {
    } else {
      /* the read failed - likely the robot was switched off - attempt
   to reconnect and reinitialize */
      connect_to_robot();
      re_initialize_robot();
      buf[0]='\0';
      return 0;
    }
    /* ensure readbuf is null terminated */
    readbuf[val] = '\0';
    totalbytes = strlen(buf);
    add_to_buffer(buf, &totalbytes, readbuf);
    reply_msg = extract_reply(buf);
    if (reply_msg != NULL)
      return reply_msg;
  }
}

void clear_input_stream() {
  // If we're out of sync, we read until there's no more input
  int val;
  fd_set read_fdset;
  struct timeval tv;
  char buf[1024];
  error_state = 1;
  usleep(500000);
  while (1) {
    /* check if there's any data to read */
    /* we want to return immediately if there's nothing to read */
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&read_fdset);
    FD_SET(sock, &read_fdset);
    if (select(sock+1, &read_fdset, NULL, NULL, &tv) == 0) {
      /* nothing to read - we're done */
      return;
    }
    assert(FD_ISSET(sock, &read_fdset));
    val = read(sock, buf, 1024);
    if (val < 0) {
      /* we got an error; leave cleaning up to elsewhere */
      printf("read failed in clear_input_stream\n");
      return;
    }
  }
}


int gp2d12_to_dist(int ir) {
  int dist;
  if (ir > 35)
    dist = (6787 / (ir - 3)) - 4;
  else
    dist=200;
  return dist;
}

int gp2d120_to_dist(int ir) {
  int dist;
  if (ir > 80)
    dist = (2914 / (ir + 5)) - 1;
  else
    dist = 40;
  return dist;
}

void recv_ack() {
  char  buf[READBUFLEN];
  const char *reply;
  memset(buf, 0, READBUFLEN);
  reply = recv_msg(buf, READBUFLEN);
  while (reply && reply[0] != '.') {
    printf("Unexpected reply: >>%s<<\n", reply);
    reply = recv_msg(buf, READBUFLEN);
  }
}

void reset_motor_encoders() {
  char  buf[READBUFLEN];
  sprintf(buf, "C RME\n");
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

void set_motor(int side, int speed) {
  char  buf[READBUFLEN];
  if (speed > 127)
    speed = 127;
  if (speed < -127)
    speed = -127;
  switch(side) {
  case LEFT:
    sprintf(buf, "M L %d\n", speed);
    break;
  case RIGHT:
    sprintf(buf, "M R %d\n", speed);
    break;
  }
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

void set_motors(int speed_l, int speed_r) {
  char  buf[READBUFLEN];
  sprintf(buf, "M LR %d %d\n", speed_l, speed_r);
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}  

void set_ir_angle(int side, int angle) {
  char  buf[READBUFLEN];
  switch(side) {
  case LEFT:
    sprintf(buf, "I L %d\n", angle);
    ir_angle[LEFT] = angle;
    break;
  case RIGHT:
    sprintf(buf, "I R %d\n", angle);
    ir_angle[RIGHT] = angle;
    break;
  }
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

int one_sensor_read(char *sensorname, int *value) {
  char  sendbuf[READBUFLEN];
  char  recvbuf[READBUFLEN];
  const char *arg;
  const char *reply_msg;
  sprintf(sendbuf, "S %s\n", sensorname);
  if (send_msg(sendbuf, strlen(sendbuf))) {
    /* now we loop, reading until we get the answer to the request we just asked. */
    memset(recvbuf, 0, READBUFLEN);
    while (1) {
      reply_msg = 0;
      while (reply_msg == 0) {
        reply_msg = recv_msg(recvbuf, READBUFLEN);
        if (reply_msg == 0) {
          printf("one sensors read failed - retrying\n");
        }
      }

      /* compare request string to response */
      if (strncmp(reply_msg, sendbuf, strlen(sendbuf)-1)==0) {
        /* skip over the fixed part of the response to get to the
           returned value */
        arg = reply_msg + strlen(sendbuf);
        *value = atoi(arg);
        /* all done */
        check_errors();
        return 0;
      } else if (reply_msg[0] == 'W') {
        /* response is an asynchronous warning - we'll print it and ignore it. */
        printf("Warning: %s", reply_msg+2);
        /* now go round the loop again and re-read */
      } else {
        /* what we got back wasn't the response we expected or a warning. */
        printf("got an error >>%s<<\n", reply_msg);
        /* we don't really know what happened - just clear any
           remaining input in the hope we can get back in sync. */
        clear_input_stream();
        return -1;
      }
    }
  } else {
    /* the send failed - retry the whole request; it should auto-reconnect */
    printf("send failed!!!\n");
    return one_sensor_read(sensorname, value);
  }
}

int two_sensor_read(char *sensornames, int *value1, int *value2) {
  char  sendbuf[READBUFLEN];
  char  recvbuf[READBUFLEN];
  const char *reply_msg;
  const char *arg;
  sprintf(sendbuf, "S %s\n", sensornames);
  if (send_msg(sendbuf, strlen(sendbuf))) {
    /* now we loop, reading until we get the answer to the request we just asked. */
    while (1) {
      memset(recvbuf, 0, READBUFLEN);
      reply_msg = 0;
      while (reply_msg == 0) {
        reply_msg = recv_msg(recvbuf, READBUFLEN);
        if (reply_msg == 0) {
          printf("two sensor read failed - retrying\n");
        }
      }

      /* remove the trailing newline from the request string so we can
   compare it to the response */
      sendbuf[strlen(sendbuf)-1]='\0';
      if (strncmp(reply_msg, sendbuf, strlen(sendbuf))==0) {

        /* skip over the fixed part of the response to get to the
           first returned value */
        arg = reply_msg + strlen(sendbuf) + 1;
        *value1 = atoi(arg);

        /* skip to next space character that should be before the second 
           returned value */
        arg = strchr(arg, ' ');
        if (arg == NULL) {
          printf("got an incomplete response >>%s<<\n", reply_msg);
          return -1;
        }

        /* skip any spaces (shouldn't really be any though) */
        while(*arg==' ') arg++;

        *value2 = atoi(arg); 
        /* all done */
        check_errors();
        return 0;
      } else if (reply_msg[0] == 'W') {
        /* response is an asynchronous warning - we'll print it and ignore it. */
        printf("Warning: %s\n", reply_msg+2);
        /* now go round the loop again and re-read */
      } else {
        /* what we got back wasn't the response we expected or a warning. */
        printf("got an error >>%s<<\n", reply_msg);
        /* we don't really know what happened - just clear any
           remaining input in the hope we can get back in sync. */
        clear_input_stream();
        return -1;
      }
    }
  } else {
    /* the send failed - retry the whole request; it should auto-reconnect */
    return two_sensor_read(sensornames, value1, value2);
  }
}

int get_front_ir_dist(int side) {
  int result = 0, value;
  switch(side) {
  case LEFT:
    result = one_sensor_read("IFL", &value);
    break;
  case RIGHT:
    result = one_sensor_read("IFR", &value);
    break;
  default:
    fprintf(stderr, "get_front_ir_dist: invalid side number %d\n", side);
    exit(1);
  }
  if (result < 0)
    /* something broke - return 200cm */
    return 200;
  return gp2d12_to_dist(value); 
}

int get_side_ir_dist(int side) {
  int result = 0, value;
  switch(side) {
  case LEFT:
    result = one_sensor_read("ISL", &value);
    break;
  case RIGHT:
    result = one_sensor_read("ISR", &value);
    break;
  default:
    fprintf(stderr, "get_front_ir_dist: invalid side number %d\n", side);
    exit(1);
  }
  if (result < 0)
    /* something broke - return 50cm */
    return 50;
  return gp2d120_to_dist(value); 
}

int get_us_dist() {
  int result, value;
  result = one_sensor_read("US", &value);
  if (result < 0)
    /* something broke - return 5 meters */
    return 500;
  return value;
}

int get_front_ir_dists(int *leftdist, int *rightdist) {
  int leftrawdist, rightrawdist;
  int result;
  result = two_sensor_read("IFLR", &leftrawdist, &rightrawdist);
  if (result < 0)
    return result;
  *leftdist = gp2d12_to_dist(leftrawdist); 
  *rightdist = gp2d12_to_dist(rightrawdist); 
  return 0;
}

int get_side_ir_dists(int *leftdist, int *rightdist) {
  int leftrawdist, rightrawdist;
  int result;
  result = two_sensor_read("ISLR", &leftrawdist, &rightrawdist);
  if (result < 0)
    return result;
  *leftdist = gp2d120_to_dist(leftrawdist); 
  *rightdist = gp2d120_to_dist(rightrawdist); 
  return 0;
}

int get_motor_encoders(int *leftenc, int *rightenc) {
  int result;
  result = two_sensor_read("MELR", leftenc, rightenc);
  if (result < 0)
    return result;
  return 0;
}

int check_bump(int side) {
  int result = 0, value;
  switch(side) {
  case LEFT:
    result = one_sensor_read("BFL", &value);
    break;
  case RIGHT:
    result = one_sensor_read("BFR", &value);
    break;
  default:
    fprintf(stderr, "get_front_ir_dist: invalid side number %d\n", side);
    exit(1);
  }
  if (result < 0)
    return result;
  return value; 
}

int check_bumpers(int *lbump, int *rbump) {
  return two_sensor_read("BFLR", lbump, rbump);
}

int get_voltage() {
  int result, value;
  result = one_sensor_read("V", &value);
  if (result < 0)
    return result;
  return value; 
}

void log_trail() {
  char  buf[READBUFLEN];
  sprintf(buf, "C TRAIL\n");
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

void set_origin() {
  char  buf[READBUFLEN];
  sprintf(buf, "C ORIGIN\n");
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

void set_point(int x, int y) {
  char  buf[READBUFLEN];
  sprintf(buf, "C POINT %d %d\n", x, y);
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

void send_text(char *txt) {
  char  buf[READBUFLEN];
  sprintf(buf, "T %s\n", txt);
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

void set_asr(int flag) {
  char  buf[READBUFLEN];
  sprintf(buf, "C ASR %d\n", flag);
  if (send_msg(buf, strlen(buf))) {
    recv_ack();
  }
}

int connect_to_robot() {
  int volts;
  printf("connecting...");
  struct sockaddr_in s_addr;
  if (sock != -1) {
    close(sock);
    sock = -1;
  }

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
    fprintf(stderr, "Failed to create socket\n");
    exit(1);
  }

  while (1) {
    s_addr.sin_family = AF_INET;
    s_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    s_addr.sin_port = htons(55443);

    if (connect(sock, (struct sockaddr *) &s_addr, sizeof(s_addr)) >= 0) {
      /* connection succeeded */
      printf("done\n");
      sleep(1);
      volts = get_voltage();
      printf("Battery state %2.1f volts\n", volts/10.0);
      re_initialize_robot();
      return sock;
    }
    sleep(1);
    printf(".");
    fflush(stdout);
  }
}


void initialize_robot() {
  printf("initializing\n");
  set_ir_angle(LEFT, 0);
  set_ir_angle(RIGHT, 0);
  reset_motor_encoders();
}

void re_initialize_robot() {
  printf("re-initializing\n");
  set_ir_angle(LEFT, ir_angle[LEFT]);
  set_ir_angle(RIGHT, ir_angle[RIGHT]);
}

void check_errors() {
  if (error_state == 1) {
    error_state = 0;
    re_initialize_robot();
  }
}
