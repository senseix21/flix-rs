/* R2-05: the UDP reply endpoint moves only on a frame the owner accepted.
 *
 * Builds the real radio.c against host stubs and real loopback sockets. The owner is
 * replaced by a stand-in that accepts a datagram whose first byte is 'A'; the Rust tests
 * (apps/esp32/tests/radio_adversarial.rs) establish that only fresh, correctly signed
 * frames are accepted. Run: sh scripts/radio-endpoint-test.sh */
#include "../main/board.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static uint64_t epoch = 1;
static int accepted;
int32_t flix_board_radio(const uint8_t *bytes, uint32_t count, uint64_t now) {
  (void)now;
  if (count > 0 && bytes[0] == 'A') {
    accepted++;
    return 0;
  }
  return -4;
}
uint64_t flix_board_radio_epoch(void) { return epoch; }
const char *const WIFI_EVENT = "WIFI_EVENT";

static int failures;
#define CHECK(condition, what)                                                 \
  do {                                                                         \
    if (!(condition)) {                                                        \
      fprintf(stderr, "FAIL %s\n", what);                                      \
      failures++;                                                              \
    } else {                                                                   \
      printf("ok   %s\n", what);                                               \
    }                                                                          \
  } while (0)

static int peer(void) {
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in local = {.sin_family = AF_INET,
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
  struct timeval wait = {.tv_sec = 0, .tv_usec = 50000};
  if (fd < 0 || bind(fd, (struct sockaddr *)&local, sizeof(local)) < 0 ||
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &wait, sizeof(wait)) < 0) {
    perror("peer");
    exit(2);
  }
  return fd;
}
static void send_to_drone(int fd, const char *bytes, size_t count) {
  struct sockaddr_in drone = {.sin_family = AF_INET,
                              .sin_port = htons(14550),
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
  sendto(fd, bytes, count, 0, (struct sockaddr *)&drone, sizeof(drone));
  usleep(10000);
  flix_radio_poll(0);
}
/* True when fd receives the drone's next reply. */
static int hears_reply(int fd) {
  static const uint8_t reply[] = "reply";
  char buffer[16];
  /* Earlier checks may have left a reply queued here; it must not count now. */
  while (recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT) > 0) {
  }
  flix_radio_send(reply, sizeof(reply));
  return recv(fd, buffer, sizeof(buffer), 0) > 0;
}

int main(void) {
  int probe = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in port = {.sin_family = AF_INET, .sin_port = htons(14550)};
  if (bind(probe, (struct sockaddr *)&port, sizeof(port)) < 0) {
    fprintf(stderr, "UDP 14550 is busy\n");
    return 2;
  }
  close(probe);
  flix_radio_init();
  int operator = peer(), attacker = peer(), roamed = peer();

  CHECK(!hears_reply(operator) && !hears_reply(attacker),
        "no endpoint before any accepted frame");
  send_to_drone(attacker, "X", 1);
  CHECK(!hears_reply(attacker), "refused frame does not set the endpoint");
  send_to_drone(operator, "A", 1);
  CHECK(hears_reply(operator), "accepted frame sets the endpoint");
  send_to_drone(attacker, "X", 1);
  CHECK(hears_reply(operator) && !hears_reply(attacker),
        "refused frame from another address does not move it");
  char oversize[281];
  memset(oversize, 'A', sizeof(oversize));
  send_to_drone(attacker, oversize, sizeof(oversize));
  CHECK(hears_reply(operator) && !hears_reply(attacker),
        "oversize datagram is not offered to the owner and does not move it");
  send_to_drone(roamed, "A", 1);
  CHECK(hears_reply(roamed) && !hears_reply(operator),
        "accepted frame from a new address moves it (key holder roamed)");
  epoch++;
  flix_radio_poll(0);
  CHECK(!hears_reply(roamed) && !hears_reply(operator),
        "new session epoch (enrollment, rotation, revoke) forgets it");
  send_to_drone(operator, "A", 1);
  CHECK(hears_reply(operator), "next accepted frame learns it again");
  CHECK(accepted == 3, "owner saw exactly the three accepted datagrams");
  return failures ? 1 : 0;
}
