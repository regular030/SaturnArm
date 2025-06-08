#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// Basic Configuration
#define NO_SYS                          1
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (16 * 1024)
#define MEMP_NUM_PBUF                   16
#define PBUF_POOL_SIZE                  16
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_DHCP                       1
#define LWIP_IPV4                       1
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define TCP_MSS                         (1460)
#define TCP_WND                         (4 * TCP_MSS)
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                (2 * TCP_SND_BUF/TCP_MSS)

// DHCP Configuration
#define LWIP_DHCP                       1
#define DHCP_DOES_ARP_CHECK             0

// Debugging
#define LWIP_DEBUG                      0
#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0

// Threading
#define TCPIP_THREAD_STACKSIZE          2048
#define DEFAULT_THREAD_STACKSIZE        1024
#define DEFAULT_RAW_RECVMBOX_SIZE       8
#define TCPIP_MBOX_SIZE                 8
#define SLIPIF_THREAD_STACKSIZE         1024
#define PPP_THREAD_STACKSIZE            1024

// Memory Options
#define MEM_LIBC_MALLOC                 0
#define MEMP_MEM_MALLOC                 0
#define MEMP_NUM_SYS_TIMEOUT            8

// PBUF Options
#define PBUF_POOL_BUFSIZE               LWIP_MEM_ALIGN_SIZE(TCP_MSS+40+PBUF_LINK_HLEN)

#endif /* LWIPOPTS_H */