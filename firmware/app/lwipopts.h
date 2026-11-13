/*
 * lwIP options for gnss_smart: NO_SYS, used through
 * pico_cyw43_arch_lwip_threadsafe_background on core 0 (plans/web-config-server.md).
 * Sized from pico-examples' lwipopts_examples_common.h, with a smaller TCP
 * window: the pages are a few KB and one or two browsers poll at 1 Hz.
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/* ---- Core ---------------------------------------------------------------- */
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (16 * 1024)
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              16
#define MEMP_NUM_SYS_TIMEOUT        (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 8)
#define LWIP_CHKSUM_ALGORITHM       3

/* ---- Protocols ----------------------------------------------------------- */
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define LWIP_UDP                    1
#define LWIP_TCP                    1
#define LWIP_DHCP                   1
#define LWIP_DNS                    1
#define LWIP_IGMP                   1  /* mDNS multicast */
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0
#define LWIP_TCP_KEEPALIVE          1

#define TCP_MSS                     1460
#define TCP_WND                     (4 * TCP_MSS)
#define TCP_SND_BUF                 (4 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define MEMP_NUM_TCP_PCB            6

/* ---- Netif --------------------------------------------------------------- */
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_EXT_STATUS_CALLBACK 1 /* mDNS re-announces on IP change */
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_NUM_NETIF_CLIENT_DATA  1  /* mDNS */

/* ---- mDNS ---------------------------------------------------------------- */
#define LWIP_MDNS_RESPONDER         1
#define MDNS_MAX_SERVICES           1

/* ---- HTTP server --------------------------------------------------------- */
#define HTTPD_FSDATA_FILE           "pico_fsdata.inc"
#define LWIP_HTTPD_CUSTOM_FILES     1  /* /status.json, see app/web/web.c */
#define LWIP_HTTPD_DYNAMIC_HEADERS  1  /* headers for the custom files */
#define LWIP_HTTPD_SUPPORT_POST     1  /* POST /config, see app/web/config_web.c */
/* Always copy into the TCP send buffer. A custom file's buffer is reused once
 * httpd closes the file, which can be before the data is acknowledged. */
#define HTTP_IS_DATA_VOLATILE(hs)   TCP_WRITE_FLAG_COPY

/* ---- Stats and debug off ------------------------------------------------- */
#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0
#define LWIP_DEBUG                  0

#endif /* LWIPOPTS_H */
