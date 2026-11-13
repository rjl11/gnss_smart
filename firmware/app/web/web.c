#include "web.h"

#include <stdio.h>
#include <string.h>

#include "pico/cyw43_arch.h"

#include "lwip/apps/fs.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/mdns.h"
#include "lwip/netif.h"

#define POLL_PERIOD_US      50000u
#define JOIN_TIMEOUT_US     30000000u  /* association plus DHCP */
#define RETRY_FIRST_US      5000000u
#define RETRY_MAX_US        30000000u
#define RSSI_PERIOD_US      2000000u
#define LED_BLINK_US        500000u

#define JSON_MAX            2048
#define JSON_SLOTS          3          /* concurrent /status.json transfers */
#define STATUS_URI          "/status.json"

static web_config_t s_cfg;
static bool s_started;
static web_info_t s_info;
static uint64_t s_next_poll, s_join_t0, s_retry_at, s_next_rssi, s_next_led;
static uint32_t s_backoff_us = RETRY_FIRST_US;
static bool s_led;

/* /status.json: the main loop writes s_json under the lwIP lock; each request
 * gets its own copy, so a transfer in progress never sees a half-new text. */
static char s_json[JSON_MAX];
static size_t s_json_len;
static char s_slot[JSON_SLOTS][JSON_MAX];
static bool s_slot_used[JSON_SLOTS];
static volatile uint32_t s_http_busy;

static struct netif *sta_netif(void) { return &cyw43_state.netif[CYW43_ITF_STA]; }

/* ---- httpd custom file (lwIP context) ----------------------------------- */

int fs_open_custom(struct fs_file *file, const char *name)
{
    if (strcmp(name, STATUS_URI) != 0) {
        return 0;
    }
    for (int i = 0; i < JSON_SLOTS; i++) {
        if (!s_slot_used[i]) {
            s_slot_used[i] = true;
            memcpy(s_slot[i], s_json, s_json_len);
            memset(file, 0, sizeof *file);
            file->data = s_slot[i];
            file->len = (int)s_json_len;
            file->index = (int)s_json_len;
            file->flags = 0; /* httpd adds the headers (Content-Type from .json) */
            return 1;
        }
    }
    s_http_busy++;
    return 0; /* answered with 404 */
}

void fs_close_custom(struct fs_file *file)
{
    for (int i = 0; i < JSON_SLOTS; i++) {
        if (file->data == s_slot[i]) {
            s_slot_used[i] = false;
        }
    }
}

/* ---- mDNS ----------------------------------------------------------------- */

static void mdns_txt(struct mdns_service *service, void *userdata)
{
    (void)userdata;
    mdns_resp_add_service_txtitem(service, "path=/", 6);
}

/* ---- Station ------------------------------------------------------------- */

static void start_join(uint64_t now)
{
    const char *pw = (s_cfg.password && s_cfg.password[0]) ? s_cfg.password : NULL;
    uint32_t auth = pw ? CYW43_AUTH_WPA2_AES_PSK : CYW43_AUTH_OPEN;
    int rc = cyw43_arch_wifi_connect_async(s_cfg.ssid, pw, auth);
    printf("[web] joining %s (%s)%s\n", s_cfg.ssid, pw ? "WPA2" : "open",
           rc ? " - start failed" : "");
    s_info.state = WEB_WIFI_JOINING;
    s_join_t0 = now;
}

static void schedule_retry(uint64_t now, const char *why)
{
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    s_info.drops++;
    s_info.ip = 0;
    s_info.state = WEB_WIFI_WAIT_RETRY;
    s_retry_at = now + s_backoff_us;
    printf("[web] %s, retry in %lu s\n", why, (unsigned long)(s_backoff_us / 1000000u));
    s_backoff_us *= 2;
    if (s_backoff_us > RETRY_MAX_US) {
        s_backoff_us = RETRY_MAX_US;
    }
}

static const char *link_name(int st)
{
    switch (st) {
    case CYW43_LINK_DOWN:    return "link down";
    case CYW43_LINK_JOIN:    return "joined";
    case CYW43_LINK_NOIP:    return "no IP";
    case CYW43_LINK_UP:      return "up";
    case CYW43_LINK_FAIL:    return "join failed";
    case CYW43_LINK_NONET:   return "network not found";
    case CYW43_LINK_BADAUTH: return "authentication failed";
    default:                 return "unknown";
    }
}

bool web_init(const web_config_t *cfg)
{
    s_cfg = *cfg;
    if (cyw43_arch_init() != 0) {
        printf("[web] CYW43439 init failed, running without Wi-Fi\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    cyw43_arch_lwip_begin();
    netif_set_hostname(sta_netif(), s_cfg.hostname);
    httpd_init();
    mdns_resp_init();
    mdns_resp_add_netif(sta_netif(), s_cfg.hostname);
    mdns_resp_add_service(sta_netif(), s_cfg.hostname, "_http", DNSSD_PROTO_TCP, 80, mdns_txt, NULL);
    cyw43_arch_lwip_end();

    web_set_status_json("{}", 2);
    s_started = true;
    start_join(time_us_64());
    return true;
}

static void led_set(bool on)
{
    if (on != s_led) {
        s_led = on;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    }
}

void web_poll(uint64_t now)
{
    if (!s_started || now < s_next_poll) {
        return;
    }
    s_next_poll = now + POLL_PERIOD_US;

    int st = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    s_info.link_status = st;

    switch (s_info.state) {
    case WEB_WIFI_JOINING:
        if (st == CYW43_LINK_UP) {
            cyw43_arch_lwip_begin();
            s_info.ip = netif_ip4_addr(sta_netif())->addr;
            mdns_resp_announce(sta_netif());
            cyw43_arch_lwip_end();
            s_info.state = WEB_WIFI_UP;
            s_info.joins++;
            s_backoff_us = RETRY_FIRST_US;
            s_next_rssi = now;
            const uint8_t *b = (const uint8_t *)&s_info.ip;
            printf("[web] up: http://%u.%u.%u.%u/  (http://%s.local/)\n", b[0], b[1], b[2], b[3],
                   s_cfg.hostname);
        } else if (st < 0) {
            schedule_retry(now, link_name(st));
        } else if (now - s_join_t0 > JOIN_TIMEOUT_US) {
            schedule_retry(now, st == CYW43_LINK_NOIP ? "no DHCP answer" : "join timed out");
        }
        break;
    case WEB_WIFI_UP:
        if (st != CYW43_LINK_UP) {
            schedule_retry(now, link_name(st));
        } else if (now >= s_next_rssi) {
            int32_t rssi;
            if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0) {
                s_info.rssi_dbm = rssi;
            }
            s_next_rssi = now + RSSI_PERIOD_US;
        }
        break;
    case WEB_WIFI_WAIT_RETRY:
        if (now >= s_retry_at) {
            start_join(now);
        }
        break;
    default:
        break;
    }

    /* LED: solid with an IP, slow blink otherwise. */
    if (s_info.state == WEB_WIFI_UP) {
        led_set(true);
    } else if (now >= s_next_led) {
        led_set(!s_led);
        s_next_led = now + LED_BLINK_US;
    }
}

void web_set_status_json(const char *json, size_t len)
{
    if (len > JSON_MAX) {
        len = JSON_MAX;
    }
    if (s_started) {
        cyw43_arch_lwip_begin();
    }
    memcpy(s_json, json, len);
    s_json_len = len;
    if (s_started) {
        cyw43_arch_lwip_end();
    }
}

void web_get_info(web_info_t *out)
{
    *out = s_info;
    out->http_busy = s_http_busy;
    if (s_info.state == WEB_WIFI_WAIT_RETRY) {
        uint64_t now = time_us_64();
        out->retry_in_s = s_retry_at > now ? (uint32_t)((s_retry_at - now + 999999u) / 1000000u) : 0;
    }
}

const char *web_state_name(web_wifi_state_t s)
{
    switch (s) {
    case WEB_WIFI_OFF:        return "off";
    case WEB_WIFI_JOINING:    return "joining";
    case WEB_WIFI_UP:         return "up";
    case WEB_WIFI_WAIT_RETRY: return "retry";
    default:                  return "?";
    }
}
