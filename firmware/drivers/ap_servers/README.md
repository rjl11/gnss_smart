# ap_servers

The small DHCP and DNS servers the open access point `gnss_smart` needs ([plans/web-config-server.md](../../../plans/web-config-server.md), Phase 3).

Copied unchanged from pico-examples, `pico_w/wifi/access_point/` at commit `0d62f75` (2026-09-03):

| File | Origin | Licence |
|---|---|---|
| `dhcpserver.c`, `dhcpserver.h` | MicroPython, Damien P. George | MIT, see `LICENSE` |
| `dnsserver.c`, `dnsserver.h` | Raspberry Pi (Trading) Ltd. | BSD-3-Clause (SPDX header in each file) |

- The DHCP server hands out up to 8 addresses from 192.168.4.16, with the AP's own address as router and DNS server.
- The DNS server answers every name with the AP's address, so a phone's captive-portal check lands on the status page.

Both are used from `firmware/app/web/web.c` only, inside the lwIP lock.
