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

typedef struct {
    const char *ssid;
    const char *password; /* NULL or "" = open network */
    const char *hostname; /* DHCP and mDNS name */
} web_config_t;

typedef enum {
    WEB_WIFI_OFF = 0,     /* web_init() not called or failed */
    WEB_WIFI_JOINING,     /* associating or waiting for DHCP */
    WEB_WIFI_UP,          /* associated with an IP address */
    WEB_WIFI_WAIT_RETRY,  /* failed or link lost; next try at a backoff */
} web_wifi_state_t;

typedef struct {
    web_wifi_state_t state;
    int link_status;      /* last cyw43_tcpip_link_status() */
    uint32_t ip;          /* network byte order, 0 if none */
    int32_t rssi_dbm;     /* valid when state is UP */
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

/* Join / retry state machine and the LED. Call from the main loop. */
void web_poll(uint64_t now_us);

/* The text /status.json serves until the next call. Copied. */
void web_set_status_json(const char *json, size_t len);

void web_get_info(web_info_t *out);
const char *web_state_name(web_wifi_state_t s);

#endif /* WEB_H */
