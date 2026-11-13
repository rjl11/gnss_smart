#include "jsonw.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

void jw_init(jw_t *w, char *buf, size_t cap)
{
    w->buf = buf;
    w->cap = cap;
    w->len = 0;
    w->overflow = cap == 0;
    w->depth = 0;
    w->need_comma[0] = false;
    if (cap) {
        buf[0] = '\0';
    }
}

bool jw_ok(const jw_t *w) { return !w->overflow && w->depth == 0; }

static void out(jw_t *w, const char *fmt, ...)
{
    if (w->overflow) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(w->buf + w->len, w->cap - w->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= w->cap - w->len) {
        w->overflow = true;
        w->buf[w->len] = '\0';
        return;
    }
    w->len += (size_t)n;
}

static void out_str(jw_t *w, const char *s)
{
    out(w, "\"");
    for (; s && *s; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') {
            out(w, "\\%c", c);
        } else if (c < 0x20) {
            out(w, "\\u%04x", c);
        } else {
            out(w, "%c", c);
        }
    }
    out(w, "\"");
}

static void prefix(jw_t *w, const char *key)
{
    if (w->need_comma[w->depth]) {
        out(w, ",");
    }
    w->need_comma[w->depth] = true;
    if (key) {
        out_str(w, key);
        out(w, ":");
    }
}

static void open_container(jw_t *w, const char *key, char c)
{
    prefix(w, key);
    out(w, "%c", c);
    if (w->depth + 1 < JW_MAX_DEPTH) {
        w->depth++;
        w->need_comma[w->depth] = false;
    } else {
        w->overflow = true;
    }
}

void jw_obj(jw_t *w, const char *key) { open_container(w, key, '{'); }
void jw_arr(jw_t *w, const char *key) { open_container(w, key, '['); }

void jw_end(jw_t *w, char close)
{
    out(w, "%c", close);
    if (w->depth > 0) {
        w->depth--;
    }
}

void jw_str(jw_t *w, const char *key, const char *v)
{
    prefix(w, key);
    out_str(w, v);
}

void jw_num(jw_t *w, const char *key, double v, int decimals)
{
    prefix(w, key);
    if (isfinite(v)) {
        out(w, "%.*f", decimals, v);
    } else {
        out(w, "null");
    }
}

void jw_int(jw_t *w, const char *key, long long v)
{
    prefix(w, key);
    out(w, "%lld", v);
}

void jw_bool(jw_t *w, const char *key, bool v)
{
    prefix(w, key);
    out(w, v ? "true" : "false");
}

void jw_null(jw_t *w, const char *key)
{
    prefix(w, key);
    out(w, "null");
}
