/*
 * Wi-Fi station and web server (plans/web-config-server.md, Phase 2).
 *
 * Core 0 only. lwIP runs in the background (pico_cyw43_arch_lwip_threadsafe_
 * background); this module never blocks the main loop except in web_init().
 *
 * Pages: / (index.html, compiled into flash) and /status.json, which serves
 * the last JSON text handed in with web_set_status_json().
 */
#ifndef WEB_H
#define WEB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    WEB_NET_AUTO = 0,     /* station; open AP if not joined within ap_fallback_s of boot */
    WEB_NET_STA,          /* station only, retry forever */
    WEB_NET_AP,           /* open AP only, from boot */
} web_net_mode_t;

typedef struct {
    web_net_mode_t mode;
    const char *ssid;     /* station network */
    const char *password; /* NULL or "" = open network */
    const char *ap_ssid;  /* open access point, 192.168.4.1 */
    uint32_t ap_fallback_s;
    const char *hostname; /* DHCP and mDNS name */
} web_config_t;

typedef enum {
    WEB_WIFI_OFF = 0,     /* web_init() not called or failed */
    WEB_WIFI_JOINING,     /* associating or waiting for DHCP */
    WEB_WIFI_UP,          /* associated with an IP address */
    WEB_WIFI_WAIT_RETRY,  /* failed or link lost; next try at a backoff */
    WEB_WIFI_AP,          /* open access point; stays until reboot */
} web_wifi_state_t;

typedef struct {
    web_wifi_state_t state;
    int link_status;      /* last cyw43_tcpip_link_status() */
    uint32_t ip;          /* network byte order, 0 if none (AP: 192.168.4.1) */
    int32_t rssi_dbm;     /* valid when state is UP */
    uint32_t ap_clients;  /* AP: stations associated now */
    uint32_t joins;       /* times the link came up */
    uint32_t drops;       /* times it went down or a join failed */
    uint32_t retry_in_s;  /* WAIT_RETRY only */
    uint32_t http_busy;   /* /status.json requests refused, no free buffer */
} web_info_t;

/* Brings up the CYW43439 (loads its firmware, a few hundred ms, blocking),
 * starts station mode, httpd and mDNS. Call before the sensor UARTs start, so
 * the blocking part costs no samples. Returns false if the chip didn't start;
 * the rest of the firmware runs on without Wi-Fi. */
bool web_init(const web_config_t *cfg);

/* The SSID in use: the station network, or the AP once it runs. */
const char *web_ssid(void);

/* Join / retry state machine, AP fallback and the LED. Call from the main
 * loop. */
void web_poll(uint64_t now_us);

/* JSON documents served from RAM. Each request gets its own copy, so a
 * transfer in progress never sees a half-new text. */
typedef enum {
    WEB_DOC_STATUS = 0,   /* /status.json */
    WEB_DOC_CONFIG,       /* /config.json */
    WEB_DOC_COUNT
} web_doc_t;

#define WEB_DOC_MAX  2048

/* The text served for doc until the next call. Copied. Main loop only. */
void web_set_doc(web_doc_t doc, const char *json, size_t len);
static inline void web_set_status_json(const char *json, size_t len)
{
    web_set_doc(WEB_DOC_STATUS, json, len);
}

/* POST handler. Runs in lwIP context (core 0, background IRQ), so it must
 * not block or write flash: parse, validate, queue for the main loop, and
 * write a short JSON reply into reply (at most cap bytes, length in
 * *reply_len). The body is at most WEB_POST_MAX bytes. */
#define WEB_POST_MAX 1024
typedef void (*web_post_fn)(const char *uri, const char *body, size_t len, char *reply, size_t cap,
                            size_t *reply_len);
void web_set_post_handler(web_post_fn fn);

/* Run fn inside the lwIP lock, for data shared with a POST handler. */
void web_locked(void (*fn)(void *), void *arg);

void web_get_info(web_info_t *out);
const char *web_state_name(web_wifi_state_t s);

#endif /* WEB_H */
