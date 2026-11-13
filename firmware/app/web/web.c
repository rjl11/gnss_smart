#include "web.h"

#include <stdio.h>
#include <string.h>

#include "pico/cyw43_arch.h"

#include "lwip/apps/fs.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/mdns.h"
#include "lwip/netif.h"

#include "dhcpserver.h"
#include "dnsserver.h"

#define POLL_PERIOD_US      50000u
#define JOIN_TIMEOUT_US     30000000u  /* association plus DHCP */
#define RETRY_FIRST_US      5000000u
#define RETRY_MAX_US        30000000u
#define RSSI_PERIOD_US      2000000u
#define LED_BLINK_US        500000u
#define LED_BLINK_AP_US     125000u

#define JSON_MAX            WEB_DOC_MAX
#define JSON_SLOTS          3          /* concurrent JSON transfers */
#define POST_REPLY_URI      "/post_reply.json"
#define POST_REPLY_MAX      256

static web_config_t s_cfg;
static bool s_started;
static web_info_t s_info;
static uint64_t s_next_poll, s_join_t0, s_retry_at, s_next_rssi, s_next_led;
static uint32_t s_backoff_us = RETRY_FIRST_US;
static bool s_led;
static uint64_t s_ap_deadline;     /* AUTO: start the AP if not joined by then */
static dhcp_server_t s_dhcp;
static dns_server_t s_dns;

/* JSON documents: the main loop writes them under the lwIP lock; each request
 * gets its own copy, so a transfer in progress never sees a half-new text. */
static const char *const k_doc_uri[WEB_DOC_COUNT] = { "/status.json", "/config.json" };
static char s_doc[WEB_DOC_COUNT][JSON_MAX];
static size_t s_doc_len[WEB_DOC_COUNT];
static char s_slot[JSON_SLOTS][JSON_MAX];
static bool s_slot_used[JSON_SLOTS];
static volatile uint32_t s_http_busy;

/* POST: one at a time; the reply is served as POST_REPLY_URI right after
 * httpd_post_finished() returns, in the same lwIP context. */
static web_post_fn s_post_fn;
static void *s_post_conn;
static char s_post_uri[32];
static char s_post_body[WEB_POST_MAX + 1];
static size_t s_post_len;
static bool s_post_overflow;
static char s_post_reply[POST_REPLY_MAX];
static size_t s_post_reply_len;

static struct netif *sta_netif(void) { return &cyw43_state.netif[CYW43_ITF_STA]; }
static struct netif *ap_netif(void) { return &cyw43_state.netif[CYW43_ITF_AP]; }

/* ---- httpd custom file (lwIP context) ----------------------------------- */

int fs_open_custom(struct fs_file *file, const char *name)
{
    const char *src = NULL;
    size_t len = 0;
    for (int d = 0; d < WEB_DOC_COUNT; d++) {
        if (strcmp(name, k_doc_uri[d]) == 0) {
            src = s_doc[d];
            len = s_doc_len[d];
        }
    }
    if (strcmp(name, POST_REPLY_URI) == 0) {
        src = s_post_reply;
        len = s_post_reply_len;
    }
    if (src == NULL) {
        return 0;
    }
    for (int i = 0; i < JSON_SLOTS; i++) {
        if (!s_slot_used[i]) {
            s_slot_used[i] = true;
            memcpy(s_slot[i], src, len);
            memset(file, 0, sizeof *file);
            file->data = s_slot[i];
            file->len = (int)len;
            file->index = (int)len;
            file->flags = 0; /* httpd adds the headers (Content-Type from .json) */
            return 1;
        }
    }
    s_http_busy++;
    return 0; /* answered with 404 */
}

/* ---- httpd POST (lwIP context) ------------------------------------------ */

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd)
{
    (void)http_request;
    (void)http_request_len;
    (void)response_uri;
    (void)response_uri_len;
    if (s_post_fn == NULL || s_post_conn != NULL || content_len < 0 ||
        content_len > WEB_POST_MAX || strlen(uri) >= sizeof s_post_uri) {
        return ERR_VAL; /* 400 */
    }
    s_post_conn = connection;
    snprintf(s_post_uri, sizeof s_post_uri, "%s", uri);
    s_post_len = 0;
    s_post_overflow = false;
    *post_auto_wnd = 1;
    return ERR_OK;
}

err_t httpd_post_receive_data(void *connection, struct pbuf *p)
{
    if (connection == s_post_conn) {
        if (s_post_len + p->tot_len <= WEB_POST_MAX) {
            pbuf_copy_partial(p, s_post_body + s_post_len, p->tot_len, 0);
            s_post_len += p->tot_len;
        } else {
            s_post_overflow = true;
        }
    }
    pbuf_free(p);
    return ERR_OK;
}

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
    if (connection != s_post_conn) {
        return; /* 404 */
    }
    s_post_conn = NULL;
    if (s_post_overflow) {
        static const char k_too_long[] = "{\"ok\":false,\"error\":\"request body too long\"}";
        memcpy(s_post_reply, k_too_long, sizeof k_too_long - 1);
        s_post_reply_len = sizeof k_too_long - 1;
    } else {
        s_post_body[s_post_len] = '\0';
        s_post_reply_len = 0;
        s_post_fn(s_post_uri, s_post_body, s_post_len, s_post_reply, sizeof s_post_reply,
                  &s_post_reply_len);
    }
    snprintf(response_uri, response_uri_len, "%s", POST_REPLY_URI);
}

void web_set_post_handler(web_post_fn fn) { s_post_fn = fn; }

void web_locked(void (*fn)(void *), void *arg)
{
    if (s_started) {
        cyw43_arch_lwip_begin();
    }
    fn(arg);
    if (s_started) {
        cyw43_arch_lwip_end();
    }
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

/* Open AP at 192.168.4.1 with DHCP, catch-all DNS and mDNS. Once it runs it
 * stays until reboot, so no station join moves its channel under connected
 * phones (plans/web-config-server.md, "Open access point"). */
static void start_ap(uint64_t now, const char *why)
{
    (void)now;
    if (s_info.state != WEB_WIFI_OFF) {
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        cyw43_arch_disable_sta_mode();
    }
    cyw43_arch_enable_ap_mode(s_cfg.ap_ssid, NULL, CYW43_AUTH_OPEN);

    ip_addr_t ip, mask;
    ip_addr_set_ip4_u32(&ip, PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS));
    ip_addr_set_ip4_u32(&mask, PP_HTONL(CYW43_DEFAULT_IP_MASK));
    cyw43_arch_lwip_begin();
    dhcp_server_init(&s_dhcp, ap_netif(), &ip, &mask);
    dns_server_init(&s_dns, ap_netif(), &ip);
    mdns_resp_add_netif(ap_netif(), s_cfg.hostname);
    mdns_resp_add_service(ap_netif(), s_cfg.hostname, "_http", DNSSD_PROTO_TCP, 80, mdns_txt, NULL);
    cyw43_arch_lwip_end();

    s_info.state = WEB_WIFI_AP;
    s_info.ip = ip_addr_get_ip4_u32(&ip);
    printf("[web] %s: open AP \"%s\", http://192.168.4.1/ until reboot\n", why, s_cfg.ap_ssid);
}

bool web_init(const web_config_t *cfg)
{
    s_cfg = *cfg;
    if (cyw43_arch_init() != 0) {
        printf("[web] CYW43439 init failed, running without Wi-Fi\n");
        return false;
    }

    cyw43_arch_lwip_begin();
    httpd_init();
    mdns_resp_init();
    cyw43_arch_lwip_end();
    web_set_doc(WEB_DOC_STATUS, "{}", 2);
    web_set_doc(WEB_DOC_CONFIG, "{}", 2);
    s_started = true;

    uint64_t now = time_us_64();
    if (s_cfg.mode == WEB_NET_AP) {
        start_ap(now, "AP mode set at build time");
        return true;
    }

    cyw43_arch_enable_sta_mode();
    cyw43_arch_lwip_begin();
    netif_set_hostname(sta_netif(), s_cfg.hostname);
    mdns_resp_add_netif(sta_netif(), s_cfg.hostname);
    mdns_resp_add_service(sta_netif(), s_cfg.hostname, "_http", DNSSD_PROTO_TCP, 80, mdns_txt, NULL);
    cyw43_arch_lwip_end();

    s_ap_deadline = now + (uint64_t)s_cfg.ap_fallback_s * 1000000u;
    start_join(now);
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

    if (s_info.state == WEB_WIFI_AP) {
        if (now >= s_next_rssi) { /* reuse the 2 s timer for the client count */
            /* With macs == NULL the driver returns the maximum, not the
             * count, so it needs a buffer. */
            static uint8_t macs[10 * 6];
            int n = (int)(sizeof macs / 6);
            cyw43_wifi_ap_get_stas(&cyw43_state, &n, macs);
            s_info.ap_clients = (uint32_t)n;
            s_next_rssi = now + RSSI_PERIOD_US;
        }
        if (now >= s_next_led) { /* fast blink */
            led_set(!s_led);
            s_next_led = now + LED_BLINK_AP_US;
        }
        return;
    }

    /* AUTO: no station link since boot within the fallback time. A link that
     * came up once and dropped later keeps retrying the station. */
    if (s_cfg.mode == WEB_NET_AUTO && s_info.joins == 0 && now >= s_ap_deadline) {
        start_ap(now, "no station link");
        return;
    }

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

void web_set_doc(web_doc_t doc, const char *json, size_t len)
{
    if ((unsigned)doc >= WEB_DOC_COUNT) {
        return;
    }
    if (len > JSON_MAX) {
        len = JSON_MAX;
    }
    if (s_started) {
        cyw43_arch_lwip_begin();
    }
    memcpy(s_doc[doc], json, len);
    s_doc_len[doc] = len;
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
    case WEB_WIFI_AP:         return "ap";
    default:                  return "?";
    }
}

const char *web_ssid(void)
{
    return s_info.state == WEB_WIFI_AP ? s_cfg.ap_ssid : s_cfg.ssid;
}
