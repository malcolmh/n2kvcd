/*
 * n2kvcd.c - userspace daemon to interface NGT1/YDNU devices to SocketCAN
 *
 * Copyright (c) 2018 Malcolm Herring
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <termios.h>
#include <linux/serial.h>
#include <stdarg.h>
#include <linux/can.h>

#include <mnd/mnd.h>

#define DAEMON_NAME "n2kvcd"
#define RUN_AS_USER "root"

static void fake_syslog(int priority, const char *format, ...) {
	va_list ap;

	printf("[%d] ", priority);
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	printf("\n");
}

typedef void (*syslog_t)(int priority, const char *format, ...);
static syslog_t syslogger = syslog;

static bool running;  // Main loop flag
static int exit_code; // Signal handler return value
static int dev;       // FD for device
static int vci;       // FD for VCAN

static void child_handler(int signum) {
	switch (signum) {
	case SIGUSR1:
		exit(EXIT_SUCCESS);
		break;
	case SIGALRM:
	case SIGCHLD:
		syslogger(LOG_NOTICE, "received signal %i", signum);
		exit_code = EXIT_FAILURE;
		running = false;
		break;
	case SIGINT:
	case SIGTERM:
		syslogger(LOG_NOTICE, "received signal %i", signum);
		exit_code = EXIT_SUCCESS;
		running = false;
		break;
	}
}

void* dev2vci(void* x) {
  uint8_t buf[1000];
  S_2000 frames[32];
  E_2000 enc2k;
  uint8_t ch;
  int seq = 0;
  enum {
    NGT_CC0, NGT_CC1, NGT_CC2, NGT_LEN, NGT_BUF, NGT_ESC
  } state = NGT_CC0;
  int chk = 0;
  int len = 0;
  int idx = 0;
  while (running) {
    int siz = read(dev, buf, 500);
    if (siz < 0) {
      syslogger(LOG_ERR, "Device read error: %s", strerror(errno));
    } else {
      for (int i = 0; i < siz; i++) {
        ch = buf[i];
        switch (state) {
        case NGT_CC0:
          if (ch == 0x10)
            state = NGT_CC1;
          break;
        case NGT_CC1:
          if (ch == 0x02)
            state = NGT_CC2;
          else
            state = NGT_CC0;
          break;
        case NGT_CC2:
          if (ch == 0x93) {
            chk = ch;
            state = NGT_LEN;
          } else
            state = NGT_CC0;
          break;
        case NGT_LEN:
          chk += ch;
          len = ch;
          if ((len < 12) || (len > 234)) {
            state = NGT_CC0;
          } else {
            idx = 0;
            state = NGT_BUF;
          }
          break;
        case NGT_BUF:
          if (len > 0) {
            if (ch == 0x10) {
              state = NGT_ESC;
            } else {
              chk += ch;
              enc2k.msg[idx++] = ch;
            }
            len--;
          } else {
            chk += ch;
            if ((chk & 0xff) == 0) {
              enc2k.pgn = enc2k.msg[1] + ((enc2k.msg[2] + (enc2k.msg[3] << 8)) << 8);
              enc2k.pri = enc2k.msg[0];
              enc2k.dst = enc2k.msg[4];
              enc2k.src = enc2k.msg[5];
              enc2k.len = idx - 11;
              enc2k.fpp = (enc2k.len > 8);
              for (int i = 0; i < enc2k.len; i++) {
                enc2k.msg[i] = enc2k.msg[i + 11];
              }
              int nf = enframeN2000(&enc2k, (enc2k.fpp ? seq++ : 0), frames);
              for (int i = 0; i < nf; i++) {
                write(vci, &frames[i], sizeof(S_2000));
              }
            } else {
              syslogger(LOG_ERR, "Message checksum error");
            }
            state = NGT_CC0;
          }
          break;
        case NGT_ESC:
          chk += ch;
          enc2k.msg[idx++] = ch;
          state = NGT_BUF;
          break;
        }
      }
    }
  }
  return NULL;
}

void* vci2dev(void* x) {
#define esc(x,y) x=y;if(y==0x10)x=y;chk+=y
  S_2000 frame;
  E_2000 enc2k;
  uint8_t buf[250];
  bool err = false;
  while (running) {
    int siz = read(vci, &frame, sizeof(struct can_frame));
    if (siz < 0) {
      syslogger(LOG_ERR, "VCAN read error: %s", strerror(errno));
    } else {
      switch (deframeN2000(&frame, &enc2k)) {
      case -1:
        if (!err) {
          syslogger(LOG_ERR, "FPP sequence error");
          err = true;
        }
        break;
      case 1:
        err = false;
        int idx = 0;
        buf[idx++] = 0x10;
        buf[idx++] = 0x02;
        buf[idx++] = 0x94;
        int chk = 0x94;
        esc(buf[idx++], (enc2k.len + 6));
        esc(buf[idx++], (enc2k.pri & 0x7));
        esc(buf[idx++], (enc2k.pgn & 0xff));
        esc(buf[idx++], ((enc2k.pgn >> 8) & 0xff));
        esc(buf[idx++], ((enc2k.pgn >> 16) & 0xff));
        esc(buf[idx++], (enc2k.dst & 0xff));
        esc(buf[idx++], (enc2k.len & 0xff));
        for (int i = 0; i < enc2k.len; i++) {
          esc(buf[idx++], enc2k.msg[i]);
        }
        esc(buf[idx++], (-chk & 0xff));
        buf[idx++] = 0x10;
        buf[idx++] = 0x03;
        if (write(dev, buf, idx) < 0) {
          syslogger(LOG_ERR, "Device write error: %s", strerror(errno));
        }
        break;
      default:
        break;
      }
    }
  }
  return NULL;
}

int main(int argc, char *argv[]) {
	struct termios attr;
  int run_as_daemon = ((argc > 3) && (*argv[3] == 'D')) ? 0 : 1;
  running = true;

  if (argc >= 3) {
    if (!run_as_daemon) {
      syslogger = fake_syslog;
    }
    openlog(DAEMON_NAME, LOG_PID, LOG_LOCAL5);

    char* tty = argv[1];
    char* can = argv[2];

    if (run_as_daemon) {
      if (daemon(0, 0)) {
        syslogger(LOG_ERR, "failed to daemonize");
        exit(EXIT_FAILURE);
      }
    } else {
      signal(SIGINT, child_handler);
      signal(SIGTERM, child_handler);
    }

    /* Open device */

    if ((dev = open(tty, O_RDWR | O_NOCTTY)) < 0) {
      syslogger(LOG_ERR, "Failed to open device %s\n", tty);
      perror(tty);
      exit(EXIT_FAILURE);
    }
    memset(&attr, 0, sizeof(attr));
    cfsetispeed(&attr, B115200);
    cfsetospeed(&attr, B115200);
    attr.c_cflag |= CS8 | CLOCAL | CREAD;
    attr.c_iflag |= IGNPAR;
    attr.c_cc[VMIN] = 1;
    attr.c_cc[VTIME] = 0;
    tcflush(dev, TCIOFLUSH);
    tcsetattr(dev, TCSANOW, &attr);
    uint8_t cmd[] = {0x10, 0x02, 0xA1, 0x03, 0x11, 0x02, 0x00, 0x49, 0x10, 0x03};
    write(dev, cmd, sizeof(cmd));
    sleep(2);

    /* Open VCAN */

    struct sockaddr_can addr;
    struct ifreq ifr;

    vci = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, argv[2]);
    if (ioctl(vci, SIOCGIFINDEX, &ifr) < 0) {
      syslogger(LOG_ERR, "Failed to set attributes for socket \"%s\": %s!\n",
          can, strerror(errno));
      exit(EXIT_FAILURE);
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(vci, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
      syslogger(LOG_ERR, "Failed to bind socket \"%s\": %s!\n", can, strerror(errno));
      exit(EXIT_FAILURE);
    }

    /* Launch RX & TX threads */

    pthread_t rxthread, txthread;

    pthread_create(&rxthread, NULL, dev2vci, NULL);
    pthread_create(&txthread, NULL, vci2dev, NULL);

    /* Idle until end */

    while (running) {
      sleep(1);
    }

    syslogger(LOG_NOTICE, "terminated on %s", tty);
    closelog();
    return exit_code;
  }
}
