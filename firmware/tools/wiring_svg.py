"""Generate docs/wiring.svg (Pico 2 W wiring) from the pin map in firmware/app/board_pins.h.

The pins are hard-coded below; update both when board_pins.h changes.
Run from the repo root: python firmware/tools/wiring_svg.py docs/wiring.svg
"""
import sys

W, H = 1200, 1140
OX = 50
P = 30                      # header pin pitch in the drawing
Y0 = 190                    # y of pin 1 / pin 40
BX0, BX1 = 540, 700         # Pico board left/right edge
LPAD, RPAD = BX0 + 12, BX1 - 12

C = dict(
    tm="#2563eb", bmp="#15803d", um="#9333ea", v5="#dc2626", v3="#ea580c",
    gnd="#111827", swd_c="#f97316", swd_d="#ca8a04", ink="#1f2937",
    mute="#6b7280", board="#0b6b3a", board_edge="#064e2a", pad="#e5c55a",
    box="#f8fafc", res="#f59e0b",
)

out = []
a = out.append


def ly(pin):  # left-side pin 1..20
    return Y0 + (pin - 1) * P


def ry(pin):  # right-side pin 21..40
    return Y0 + (40 - pin) * P


def text(x, y, s, size=12, anchor="start", fill=C["ink"], weight="normal", extra=""):
    a(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" '
      f'fill="{fill}" font-weight="{weight}" {extra}>{s}</text>')


def wire(points, color, width=2.6, dash=None):
    d = " ".join(f"{x},{y}" for x, y in points)
    ds = f' stroke-dasharray="{dash}"' if dash else ""
    a(f'<polyline points="{d}" fill="none" stroke="{color}" stroke-width="{width}" '
      f'stroke-linecap="round" stroke-linejoin="round"{ds}/>')


def arrow(x, y, color, direction):  # small direction chevron on a horizontal wire
    s = 6 if direction == "r" else -6
    a(f'<path d="M{x - s},{y - 5} L{x + s},{y} L{x - s},{y + 5}" fill="none" '
      f'stroke="{color}" stroke-width="2.2" stroke-linejoin="round"/>')


def flag(x, y, label, color, side="l"):
    """Power net flag; x,y = wire end. side 'l' puts the flag to the left."""
    w = 9 + 7.2 * len(label)
    x0 = x - w - 8 if side == "l" else x + 8
    tip = x - 8 if side == "l" else x + 8
    if side == "l":
        a(f'<path d="M{x0},{y - 10} H{tip} L{x},{y} L{tip},{y + 10} H{x0} Z" fill="{color}"/>')
    else:
        a(f'<path d="M{x},{y} L{tip},{y - 10} H{tip + w} V{y + 10} H{tip} Z" fill="{color}"/>')
    text(x0 + w / 2 if side == "l" else tip + w / 2, y + 4, label, 11.5, "middle", "#fff", "bold")


def box(x0, y0, x1, y1, title, sub, color):
    a(f'<rect x="{x0}" y="{y0}" width="{x1 - x0}" height="{y1 - y0}" rx="8" '
      f'fill="{C["box"]}" stroke="{color}" stroke-width="2"/>')
    a(f'<rect x="{x0}" y="{y0}" width="{x1 - x0}" height="26" rx="8" fill="{color}"/>')
    a(f'<rect x="{x0}" y="{y0 + 16}" width="{x1 - x0}" height="10" fill="{color}"/>')
    text((x0 + x1) / 2, y0 + 18, title, 13, "middle", "#fff", "bold")
    if sub:
        for i, line in enumerate(sub):
            text((x0 + x1) / 2, y1 - 10 - 14 * (len(sub) - 1 - i), line, 10.5, "middle", C["mute"])


def pinlabel(x, y, s, anchor):
    text(x, y + 4, s, 11, anchor, C["ink"], "bold")


a(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
  f'font-family="Segoe UI, Helvetica, Arial, sans-serif">')
a(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
a(f'<g transform="translate({OX},0)">')
text(-20, 42, "gnss_smart: wiring to the Pico 2 W", 22, weight="bold")
text(-20, 64, "Pin map from firmware/app/board_pins.h (suggested assignments). Board viewed from the top, USB at the top. "
     "Physical header pin numbers in grey.", 12, fill=C["mute"])

# ---------------- Pico board ----------------
BY0, BY1 = Y0 - 40, ly(20) + 60
a(f'<rect x="{BX0}" y="{BY0}" width="{BX1 - BX0}" height="{BY1 - BY0}" rx="10" '
  f'fill="{C["board"]}" stroke="{C["board_edge"]}" stroke-width="2"/>')
a(f'<rect x="{(BX0 + BX1) / 2 - 26}" y="{BY0 - 14}" width="52" height="30" rx="4" fill="#c7ccd1" stroke="#8a9096"/>')
text((BX0 + BX1) / 2, BY0 + 34, "Pico 2 W", 14, "middle", "#ffffff", "bold")

NUMS = []
left = {1: "GP0", 2: "GP1", 3: "GND", 4: "GP2", 5: "GP3", 6: "GP4", 7: "GP5", 8: "GND", 9: "GP6",
        10: "GP7", 11: "GP8", 12: "GP9", 13: "GND", 14: "GP10", 15: "GP11", 16: "GP12", 17: "GP13",
        18: "GND", 19: "GP14", 20: "GP15"}
right = {40: "VBUS", 39: "VSYS", 38: "GND", 37: "3V3_EN", 36: "3V3(OUT)", 35: "ADC_VREF", 34: "GP28",
         33: "GND", 32: "GP27", 31: "GP26", 30: "RUN", 29: "GP22", 28: "GND", 27: "GP21", 26: "GP20",
         25: "GP19", 24: "GP18", 23: "GND", 22: "GP17", 21: "GP16"}
used_l = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
used_r = {40, 38, 36}
reserved = {27, 26, 25, 24, 22, 21}

for pin, name in left.items():
    y = ly(pin)
    on = pin in used_l
    a(f'<circle cx="{LPAD}" cy="{y}" r="6.5" fill="{C["pad"] if on else "#9fb5a8"}" stroke="#7a6420" stroke-width="1"/>')
    NUMS.append((BX0 - 8, y + 4, str(pin), "end"))
    text(LPAD + 12, y + 4, name, 11, "start", "#ffffff" if on else "#a7c4b3", "bold" if on else "normal")
for pin, name in right.items():
    y = ry(pin)
    on = pin in used_r
    fill = C["pad"] if on else ("#f2c46d" if pin in reserved or pin == 29 else "#9fb5a8")
    a(f'<circle cx="{RPAD}" cy="{y}" r="6.5" fill="{fill}" stroke="#7a6420" stroke-width="1"/>')
    NUMS.append((BX1 + 8, y + 4, str(pin), "start"))
    col = "#ffffff" if on else ("#fde68a" if pin in reserved or pin == 29 else "#a7c4b3")
    text(RPAD - 12, y + 4, name, 11, "end", col, "bold" if on or pin in reserved or pin == 29 else "normal")

# Debug pads on the bottom edge (SWCLK, GND, SWDIO, left to right)
DPY = BY1 - 16
dbg = [("SWCLK", (BX0 + BX1) / 2 - 30, C["swd_c"]), ("GND", (BX0 + BX1) / 2, C["gnd"]),
       ("SWDIO", (BX0 + BX1) / 2 + 30, C["swd_d"])]
for name, x, _ in dbg:
    a(f'<circle cx="{x}" cy="{DPY}" r="5.5" fill="{C["pad"]}" stroke="#7a6420"/>')
    text(x, DPY - 11, name, 8.5, "middle", "#ffffff", "bold")

# ---------------- TM210 IMU ----------------
TX0, TX1 = 170, 360
box(TX0, ly(1) - 40, TX1, ly(3) + 22, "TM210 IMU", None, C["tm"])
text((TX0 + TX1) / 2, ly(3) + 16, "PH2.0-5 connector", 10, "middle", C["mute"])
for pin, lab, d in [(1, "RXD", "r2l"), (2, "TXD", "l2r"), (3, "GND", None)]:
    y = ly(pin)
    col = C["gnd"] if lab == "GND" else C["tm"]
    wire([(TX1, y), (LPAD, y)], col)
    pinlabel(TX1 - 8, y, lab, "end")
    if d:
        arrow((TX1 + BX0) / 2, y, col, "l" if d == "r2l" else "r")
text((TX1 + BX0) / 2, ly(1) - 7, "UART0 TX", 10, "middle", C["tm"])
text((TX1 + BX0) / 2, ly(2) - 7, "UART0 RX  · 921600 baud", 10, "middle", C["tm"])
y5 = ly(2)
wire([(TX0, y5), (TX0 - 28, y5)], C["v5"])
pinlabel(TX0 + 8, y5, "5V", "start")
flag(TX0 - 28, y5, "5V", C["v5"])

# ---------------- BMP581 barometer (outer loop) ----------------
BMX0, BMX1 = 40, 200
BMY0, BMY1 = ly(4) - 40, ly(10) + 80
box(BMX0, BMY0, BMX1, BMY1, "BMP581 baro", ["I2C addr 0x46", "(SDO to GND)"], C["bmp"])
for pin, lab, d in [(4, "SDA", "both"), (5, "SCL", "l"), (10, "INT", "r")]:
    y = ly(pin)
    wire([(BMX1, y), (LPAD, y)], C["bmp"])
    pinlabel(BMX1 - 8, y, lab, "end")
    if d == "l":
        arrow(255, y, C["bmp"], "l")
    elif d == "r":
        arrow(255, y, C["bmp"], "r")
    else:
        arrow(249, y, C["bmp"], "l")
        arrow(261, y, C["bmp"], "r")
text(470, ly(4) - 7, "I2C1 SDA", 10, "middle", C["bmp"])
text(470, ly(5) - 7, "I2C1 SCL · 400 kHz", 10, "middle", C["bmp"])
text(480, ly(10) + 16, "INT (edge timestamp)", 10, "middle", C["bmp"])
for y, lab, col in [(ly(6) + 10, "VDD", C["v3"]), (ly(7) + 10, "VDDIO", C["v3"]), (ly(8) + 10, "GND", C["gnd"])]:
    wire([(BMX0, y), (BMX0 - 4, y)], col)
    pinlabel(BMX0 + 8, y, lab, "start")
# flags just outside the BMP box's left edge
for y, lab, col in [(ly(6) + 10, "3V3", C["v3"]), (ly(7) + 10, "3V3", C["v3"]), (ly(8) + 10, "GND", C["gnd"])]:
    flag(BMX0 - 4, y, lab, col)

# ---------------- UM980 / UM982 (nested inside the BMP loop) ----------------
UX0, UX1 = 268, 420
UY0, UY1 = ly(6) - 40, ly(9) + 18
a(f'<rect x="{UX0}" y="{UY0}" width="{UX1 - UX0}" height="{UY1 - UY0}" rx="8" fill="{C["box"]}" '
  f'stroke="{C["um"]}" stroke-width="2"/>')
a(f'<rect x="{UX0}" y="{UY0}" width="{UX1 - UX0}" height="26" rx="8" fill="{C["um"]}"/>')
a(f'<rect x="{UX0}" y="{UY0 + 16}" width="{UX1 - UX0}" height="10" fill="{C["um"]}"/>')
text((UX0 + UX1) / 2, UY0 + 18, "UM980 / UM982 module", 12, "middle", "#fff", "bold")
for pin, lab, d in [(6, "COM1 RX", "l"), (7, "COM1 TX", "r"), (8, "GND", None), (9, "PPS", "r")]:
    y = ly(pin)
    col = C["gnd"] if lab == "GND" else C["um"]
    wire([(UX1, y), (LPAD, y)], col)
    pinlabel(UX1 - 8, y, lab, "end")
    if d:
        arrow(485, y, col, d)
text(485, ly(6) - 7, "UART1 TX", 10, "middle", C["um"])
text(485, ly(7) - 7, "UART1 RX · 460800", 10, "middle", C["um"])
text(485, ly(9) - 7, "GPIO in", 10, "middle", C["um"])
yv = ly(8)
wire([(UX0, yv), (UX0 - 14, yv)], C["v5"])
pinlabel(UX0 + 8, yv, "5V", "start")
flag(UX0 - 14, yv, "5V", C["v5"])

# ---------------- Power pins on the right ----------------
for pin, lab, col in [(40, "5V", C["v5"]), (38, "GND", C["gnd"])]:
    y = ry(pin)
    wire([(RPAD, y), (BX1 + 50, y)], col)
    flag(BX1 + 50, y, lab, col, side="r")
y36 = ry(36)
wire([(RPAD, y36), (BX1 + 50, y36)], C["v3"])
flag(BX1 + 50, y36, "3V3", C["v3"], side="r")
text(BX1 + 112, ry(40) + 4, "VBUS: 5 V from the Pico's USB → TM210, GNSS module", 11, fill=C["v5"])
text(BX1 + 112, ry(38) + 4, "GND: any Pico GND pin (3, 8, 13, 18, 23, 28, 33, 38)", 11, fill=C["gnd"])
text(BX1 + 112, ry(36) + 4, "3V3(OUT): 3.3 V regulator → BMP581 only", 11, fill=C["v3"])

# ---------------- GP22 web-mode switch (production build) ----------------
y29, y28 = ry(29), ry(28)
sx = BX1 + 60
wire([(RPAD, y29), (sx, y29), (sx, y29 + 6)], C["mute"], 2, "5 4")
wire([(sx, y28 - 6), (sx, y28), (RPAD, y28)], C["mute"], 2, "5 4")
a(f'<line x1="{sx}" y1="{y29 + 6}" x2="{sx + 12}" y2="{y28 - 8}" stroke="{C["mute"]}" stroke-width="2"/>')
a(f'<circle cx="{sx}" cy="{y29 + 6}" r="2.5" fill="{C["mute"]}"/><circle cx="{sx}" cy="{y28 - 6}" r="2.5" fill="{C["mute"]}"/>')
text(sx + 26, y29 + 2, "Web-mode switch GP22 → GND (optional)", 11, fill=C["ink"], weight="bold")
text(sx + 26, y29 + 17, "Only read by a -DWEB_MODE=switch build: closed at boot starts Wi-Fi.", 10.5, fill=C["mute"])
text(sx + 26, y29 + 31, "Development builds (WEB_MODE=always) ignore it.", 10.5, fill=C["mute"])

# Reserved bracket GP16-21 (pins 27..21)
yt, yb = ry(27) - 10, ry(21) + 10
bx = BX1 + 40
a(f'<path d="M{bx},{yt} h10 V{yb} h-10" fill="none" stroke="{C["res"]}" stroke-width="2.5"/>')
text(bx + 22, (yt + yb) / 2 - 6, "Reserved: GP16–GP21 (SPI0)", 11.5, fill="#b45309", weight="bold")
text(bx + 22, (yt + yb) / 2 + 10, "W5500 Ethernet, on hold. Leave unconnected.", 10.5, fill=C["mute"])

# ---------------- Debug Probe (optional) ----------------
PX0, PX1 = (BX0 + BX1) / 2 - 120, (BX0 + BX1) / 2 + 120
PY0 = BY1 + 60
for name, x, col in dbg:
    wire([(x, DPY), (x, PY0)], col)
box(PX0, PY0, PX1, PY0 + 66, "Debug Probe, D port (optional)", None, "#64748b")
for (name, x, col), plab in zip(dbg, ["SC", "GND", "SD"]):
    text(x, PY0 + 44, plab, 11, "middle", C["ink"], "bold")
text((PX0 + PX1) / 2, PY0 + 60, "orange · black · yellow", 9.5, "middle", C["mute"])
text(PX1 + 14, PY0 + 24, "SWD: solder a 3-pin male header to", 10.5, fill=C["mute"])
text(PX1 + 14, PY0 + 38, "SWCLK / GND / SWDIO on the board edge.", 10.5, fill=C["mute"])

for x, y, t, anc in NUMS:
    text(x, y, t, 10.5, anc, C["mute"], "bold", 'stroke="#ffffff" stroke-width="4" paint-order="stroke"')

# ---------------- Notes ----------------
NY = PY0 + 100
a(f'<rect x="-20" y="{NY}" width="{W - 60}" height="{H - NY - 24}" rx="8" fill="#fffbeb" stroke="#fcd34d"/>')
text(-2, NY + 24, "Before you power it up", 13.5, weight="bold")
notes = [
    "TX goes to RX: Pico GP0 (UART0 TX) → TM210 RXD, GP4 (UART1 TX) → GNSS COM1 RX, and the other way round for the RX pins.",
    "The GNSS module and the TM210 run from 5 V (VBUS). Their UART outputs and PPS must be 3.3 V logic: check the GNSS module's COM1 TX and PPS "
    "on the bench first.",
    "VBUS carries 5 V only while the Pico is powered from its USB port. Budget: GNSS module + antenna (not yet measured) + TM210 80 mA.",
    "Connect every ground: TM210 GND → pin 3, GNSS GND → pin 8, BMP581 GND → any GND pin. Keep the TM210 and UART wires short at 921600 baud.",
    "GNSS module: only COM1 goes to the Pico. COM2 stays unwired (reserved for the Zero 2 W companion); the USB-C port is COM3, free for a PC.",
]
for i, n in enumerate(notes):
    text(-2, NY + 48 + i * 19, "• " + n, 11.5)

a("</g></svg>")
open(sys.argv[1], "w", encoding="utf-8").write("\n".join(out) + "\n")
