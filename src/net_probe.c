// net_probe.c — 한 인터페이스의 수신 트래픽을 송신 IP별로 세는 작은 도우미.
//
// 언매니지드 스위치에는 포트별 카운터가 없으므로, PC의 업링크 포트에서
// 들어오는 프레임을 AF_PACKET으로 받아 src IP별 바이트/패킷을 누적한다.
// 장치 하나 = 스위치 포트 하나이므로 사실상 포트별 통계와 같다.
// MSG_TRUNC로 앞 64바이트만 복사받고 실제 길이만 취하므로 400 MB/s급 트래픽에서도
// 부담이 작다.
//
// 권한: AF_PACKET은 CAP_NET_RAW가 필요. 한 번만:
//   sudo setcap cap_net_raw+ep <설치경로>/net_probe
//
// 출력: 1초마다 JSON 한 줄 (누적값 — 소비자가 차분을 낸다)
//   {"dt":1.000,"total":[bytes,pkts],"ips":{"192.168.1.3":[bytes,pkts],...}}

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define MAX_IPS 256

#define MAX_FLOWS 4

// src IP별 "어디로 무슨 포트로 보내는가" 상위 몇 개 — 정체 식별용
struct flow {
  uint8_t proto;      // 6 tcp / 17 udp / 기타
  uint16_t dport;
  uint8_t kind;       // 0 unicast, 1 broadcast(255.255.255.255 또는 .255), 2 multicast
  uint64_t pkts;
};

struct entry {
  uint32_t ip;
  uint64_t bytes, pkts;
  struct flow flows[MAX_FLOWS];
  int n_flows;
};

static struct entry table[MAX_IPS];
static int n_entries = 0;

static struct entry * lookup(uint32_t ip)
{
  for (int i = 0; i < n_entries; ++i) {
    if (table[i].ip == ip) {return &table[i];}
  }
  if (n_entries >= MAX_IPS) {return NULL;}
  table[n_entries].ip = ip;
  table[n_entries].bytes = table[n_entries].pkts = 0;
  return &table[n_entries++];
}

static void note_flow(struct entry * e, uint8_t proto, uint16_t dport, uint8_t kind)
{
  for (int i = 0; i < e->n_flows; ++i) {
    struct flow * f = &e->flows[i];
    if (f->proto == proto && f->dport == dport && f->kind == kind) {
      f->pkts++;
      return;
    }
  }
  if (e->n_flows < MAX_FLOWS) {
    e->flows[e->n_flows++] = (struct flow){proto, dport, kind, 1};
  }
}

static double elapsed(const struct timespec * a, const struct timespec * b)
{
  return (double)(b->tv_sec - a->tv_sec) + (double)(b->tv_nsec - a->tv_nsec) / 1e9;
}

int main(int argc, char ** argv)
{
  if (argc < 2) {
    fprintf(stderr, "usage: net_probe <interface>\n");
    return 2;
  }
  int fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_IP));
  if (fd < 0) {
    fprintf(stderr, "PERMISSION: socket(AF_PACKET): %s\n", strerror(errno));
    fprintf(stderr, "run once: sudo setcap cap_net_raw+ep %s\n", argv[0]);
    return 1;
  }
  struct sockaddr_ll sll;
  memset(&sll, 0, sizeof sll);
  sll.sll_family = AF_PACKET;
  sll.sll_protocol = htons(ETH_P_IP);
  sll.sll_ifindex = (int)if_nametoindex(argv[1]);
  if (sll.sll_ifindex == 0) {
    fprintf(stderr, "no such interface: %s\n", argv[1]);
    return 1;
  }
  if (bind(fd, (struct sockaddr *)&sll, sizeof sll) < 0) {
    fprintf(stderr, "bind: %s\n", strerror(errno));
    return 1;
  }
  int rcvbuf = 64 << 20;
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);

  unsigned char buf[64];
  uint64_t total_bytes = 0, total_pkts = 0;
  struct timespec last, now;
  clock_gettime(CLOCK_MONOTONIC, &last);

  for (;;) {
    struct pollfd pfd = {fd, POLLIN, 0};
    if (poll(&pfd, 1, 200) > 0) {
      for (;;) {
        struct sockaddr_ll from;
        socklen_t fl = sizeof from;
        ssize_t n = recvfrom(fd, buf, sizeof buf, MSG_TRUNC | MSG_DONTWAIT,
                             (struct sockaddr *)&from, &fl);
        if (n < 0) {break;}                        // EAGAIN: 큐 비움
        if (from.sll_pkttype == PACKET_OUTGOING || n < 34) {continue;}
        uint32_t src, dst;
        memcpy(&src, buf + 26, 4);                 // eth(14) + ip src 오프셋 12
        memcpy(&dst, buf + 30, 4);
        struct entry * e = lookup(src);
        if (e) {
          e->bytes += (uint64_t)n;
          e->pkts += 1;
          const int ihl = (buf[14] & 0x0f) * 4;
          const uint8_t proto = buf[23];
          uint16_t dport = 0;
          if ((proto == 17 || proto == 6) && n >= 14 + ihl + 4 &&
              14 + ihl + 4 <= (int)sizeof buf)
          {
            dport = (uint16_t)((buf[14 + ihl + 2] << 8) | buf[14 + ihl + 3]);
          }
          const uint8_t d0 = (uint8_t)(ntohl(dst) >> 24);
          const uint8_t kind = (dst == 0xFFFFFFFFu || (ntohl(dst) & 0xFF) == 0xFF) ? 1 :
            (d0 >= 224 && d0 <= 239) ? 2 : 0;
          note_flow(e, proto, dport, kind);
        }
        total_bytes += (uint64_t)n;
        total_pkts += 1;
      }
    }
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = elapsed(&last, &now);
    if (dt >= 1.0) {
      printf("{\"dt\":%.3f,\"total\":[%llu,%llu],\"ips\":{", dt,
             (unsigned long long)total_bytes, (unsigned long long)total_pkts);
      for (int i = 0; i < n_entries; ++i) {
        struct in_addr a;
        a.s_addr = table[i].ip;
        printf("%s\"%s\":[%llu,%llu,[", i ? "," : "", inet_ntoa(a),
               (unsigned long long)table[i].bytes,
               (unsigned long long)table[i].pkts);
        for (int k = 0; k < table[i].n_flows; ++k) {
          const struct flow * f = &table[i].flows[k];
          printf("%s[%u,%u,%u,%llu]", k ? "," : "", f->proto, f->dport, f->kind,
                 (unsigned long long)f->pkts);
        }
        printf("]]");
      }
      printf("}}\n");
      fflush(stdout);
      last = now;
    }
  }
}
