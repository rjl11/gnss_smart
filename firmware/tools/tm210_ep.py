"""EasyProtocol packet code shared by the TM210 bench tools.

The layout follows plans/imu-tm210-config.md ("Facts that shape the design"),
which records what the TM210 really does where SYD's documents disagree:

    AA 55 | len | 4-byte header | payload | CRC16 Modbus (LE, over len..payload)
    header, LSB first: CMD (7), 3 ignored bits, fromId (11), toId (11)

Also used for the TM171, whose Setting layout matches (AiO firmware cross-check
in the same plan).
"""

import struct

SYNC = b"\xAA\x55"

CMD_REQUEST = 12
CMD_ACK = 13
CMD_SETTING = 21
CMD_STATUS = 22
CMD_RAW = 41

ID_BROADCAST = 0
ID_HOST = 2

RAW_LEN = 40
STATUS_LEN = 12
SETTING_LEN = 20

# Setting object, TM210 layout (firmware/drivers/tm210/tm210.c).
SW_STATUS_OUT = 1 << 5
SW_RAW_OUT = 1 << 6
SW_QUAT_OUT = 1 << 8
SW_EULER_OUT = 1 << 10
SW_RPY_OUT = 1 << 11
SW_GRAVITY_OUT = 1 << 12
SW_RESERVED3 = 1 << 13
SW_RESERVED4_MASK = 7 << 14
SW_RESERVED4_VALUE = 2 << 14
SW_REQUEST_ACK = 1 << 17
SW_SAVE_FLASH = 1 << 18
SW_UART_OUT = 1 << 19
SW_USB_OUT = 1 << 20
SW_TEMP_CALIB = 1 << 23
SW_BOOT_MODE_MASK = 3 << 24
SW_CMD42_OUT = 1 << 26
SW_COMBO_OUT = 1 << 27
SW_SLOW_OUT = 1 << 28

# The logging target: raw only on USB. Boot mode is left alone (it only
# matters after a power cycle, from flash, and this tool writes RAM only).
SW_LOG_MUST_BE_0 = (SW_STATUS_OUT | SW_QUAT_OUT | SW_EULER_OUT | SW_RPY_OUT |
                    SW_GRAVITY_OUT | SW_TEMP_CALIB | SW_CMD42_OUT |
                    SW_COMBO_OUT | SW_SLOW_OUT)
SW_LOG_MUST_BE_1 = SW_RAW_OUT | SW_USB_OUT

OFS_SWITCHES = 0
OFS_BAUD = 6         # uint16, unit 100 bps
OFS_SILENT_TIME = 16  # uint16, seconds
OFS_ODR = 18         # uint16, Hz

TARGET_ODR_HZ = 800


def _crc_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
        t.append(c)
    return t


_CRC_TABLE = _crc_table()


def crc16(data):
    """CRC-16 Modbus."""
    c = 0xFFFF
    t = _CRC_TABLE
    for b in data:
        c = (c >> 8) ^ t[(c ^ b) & 0xFF]
    return c


def encode(cmd, to_id, payload):
    body = bytes([4 + len(payload)]) + struct.pack(
        "<I", cmd | (ID_HOST << 10) | (to_id << 21)) + bytes(payload)
    return SYNC + body + struct.pack("<H", crc16(body))


def request(cmd):
    return encode(CMD_REQUEST, ID_BROADCAST, bytes([cmd, 0, 0, 0]))


class Parser:
    """Byte-stream parser. feed() returns a list of (cmd, from_id, payload)."""

    MAX_LEN = 255 + 5

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.skipped = 0

    def reset(self):
        self.buf.clear()

    def feed(self, data):
        self.buf += data
        out = []
        b = self.buf
        i = 0
        n = len(b)
        while True:
            j = b.find(SYNC, i)
            if j < 0:
                # Keep a trailing 0xAA, it may start the next packet.
                keep = n - 1 if n and b[-1] == 0xAA else n
                self.skipped += max(0, keep - i)
                i = keep
                break
            self.skipped += j - i
            if j + 3 > n:
                i = j
                break
            plen = b[j + 2]
            if plen < 4:
                self.skipped += 1
                i = j + 1
                continue
            end = j + plen + 5
            if end > n:
                i = j
                break
            if crc16(b[j + 2:end - 2]) != (b[end - 2] | (b[end - 1] << 8)):
                self.crc_errors += 1
                i = j + 1
                continue
            header = b[j + 3] | (b[j + 4] << 8) | (b[j + 5] << 16) | (b[j + 6] << 24)
            out.append((header & 0x7F, (header >> 10) & 0x7FF, bytes(b[j + 7:end - 2])))
            i = end
        del b[:i]
        return out


def parse_status(p):
    """(temperature degC, internal rate Hz, QoS) from a Status payload."""
    temp = struct.unpack_from("<f", p, 4)[0]
    rate, qos = struct.unpack_from("<HH", p, 8)
    return temp, rate, qos & 7


def parse_raw(p):
    """(timestamp us uint32, gyr xyz rad/s, acc xyz g, mag xyz) in the TM210's
    own FLU frame, as sent."""
    v = struct.unpack_from("<I9f", p)
    return v[0], v[1:4], v[4:7], v[7:10]


def switches(s):
    return struct.unpack_from("<I", s, OFS_SWITCHES)[0]


def describe_setting(s):
    sw = switches(s)
    baud, = struct.unpack_from("<H", s, OFS_BAUD)
    silent, odr = struct.unpack_from("<HH", s, OFS_SILENT_TIME)
    return (f"switches 0x{sw:08x}, baud {baud * 100}, silent {silent} s, ODR {odr} Hz, "
            f"raw {int(bool(sw & SW_RAW_OUT))}, USB {int(bool(sw & SW_USB_OUT))}, "
            f"UART {int(bool(sw & SW_UART_OUT))}")


def layout_recognised(s):
    sw = switches(s)
    return bool(sw & SW_RESERVED3) and (sw & SW_RESERVED4_MASK) == SW_RESERVED4_VALUE


def log_target_matches(s):
    sw = switches(s)
    _, odr = struct.unpack_from("<HH", s, OFS_SILENT_TIME)
    return (sw & SW_LOG_MUST_BE_0) == 0 and (sw & SW_LOG_MUST_BE_1) == SW_LOG_MUST_BE_1 \
        and odr == TARGET_ODR_HZ


def log_target(s):
    """The read-back with the logging target applied, RAM only, ack requested.
    Baud and every other field are copied."""
    s = bytearray(s)
    sw = switches(s)
    sw &= ~(SW_LOG_MUST_BE_0 | SW_SAVE_FLASH)
    sw |= SW_LOG_MUST_BE_1 | SW_REQUEST_ACK
    struct.pack_into("<I", s, OFS_SWITCHES, sw)
    struct.pack_into("<HH", s, OFS_SILENT_TIME, 0, TARGET_ODR_HZ)
    return bytes(s)
