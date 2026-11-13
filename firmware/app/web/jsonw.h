/*
 * Minimal JSON writer into a fixed buffer. Commas are inserted for you;
 * non-finite numbers are written as null, so the output stays valid JSON.
 * On overflow the text is cut and jw_ok() returns false.
 */
#ifndef JSONW_H
#define JSONW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define JW_MAX_DEPTH 8

typedef struct {
    char *buf;
    size_t cap, len;
    bool overflow;
    int depth;
    bool need_comma[JW_MAX_DEPTH];
} jw_t;

void jw_init(jw_t *w, char *buf, size_t cap);
bool jw_ok(const jw_t *w);

/* key is NULL inside an array. */
void jw_obj(jw_t *w, const char *key);
void jw_arr(jw_t *w, const char *key);
void jw_end(jw_t *w, char close); /* '}' or ']' */

void jw_str(jw_t *w, const char *key, const char *v);
void jw_num(jw_t *w, const char *key, double v, int decimals);
void jw_int(jw_t *w, const char *key, long long v);
void jw_bool(jw_t *w, const char *key, bool v);
void jw_null(jw_t *w, const char *key);

#endif /* JSONW_H */
