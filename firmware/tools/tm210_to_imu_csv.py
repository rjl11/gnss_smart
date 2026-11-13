#!/usr/bin/env python3
"""Convert a tm210_allan_log.py recording to INSLIB's replay imu.csv.

Output rows (external/INSLIB/datasets/replay_format.py, IMU_HEADER_TEMP):
    t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2], imu_temp_c [degC]
which external/INSLIB/tools/allan_variance.py reads directly.

- Starts at the record start marked in meta.json (after the warm-up and
  QoS), unless --from-start.
- Frame: TM210 FLU to FRD, gyro (gx, -gy, -gz), accelerometer
  (-ax, ay, az) x 9.8158 because the raw object is the gravity vector
  (plans/imu-tm210.md, bench results). --acc-specific-force for an IMU that
  reports specific force instead: (ax, -ay, -az) x g.
- Time is the IMU's own timestamp (uint32 us, wraps every 71.6 min),
  unwrapped. The TM210 clock runs a few tenths of a percent fast against the
  host, so tau is off by that much; it does not matter for the Allan fit.
- allan_variance.py assumes one uniform sample period, so gaps are filled:
  missing samples (USB drops whole packets) are linearly interpolated, up to
  --max-fill-ms. A longer gap, a backward step or a timestamp restart (power
  cycle) ends the segment. With one segment the output is <out>/imu.csv;
  with several, each goes to <out>/segNN/imu.csv and the longest is named.
- Temperature is the last Status reply before each sample (1 Hz).
- --average N writes the mean of each N samples (8 gives 100 Hz). A 12 h
  record at 800 Hz is about 35 M rows, 3.5 GB of CSV, which allan_variance.py
  loads with np.loadtxt; block means leave the Allan deviation unchanged at
  tau >= N samples, so --average 8 loses only tau < 10 ms.

Usage: tm210_to_imu_csv.py <log dir> [--out DIR] [--average N] ...
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tm210_ep as ep  # noqa: E402

G = 9.8158
CHUNK = 1 << 20


class Writer:
    """Writes segments, with optional block averaging."""

    def __init__(self, out, average, min_seconds):
        self.out = out
        self.average = average
        self.min_us = min_seconds * 1e6
        self.segments = []      # dicts
        self.f = None
        self.path = None
        self.acc = None
        self.k = 0

    def begin(self, t_us):
        self.close()
        idx = len(self.segments)
        self.path = os.path.join(self.out, f"seg{idx:02d}.tmp.csv")
        self.f = open(self.path, "w", newline="\n")
        self.f.write("# t_us, gyr_frd_x [rad/s], gyr_frd_y [rad/s], gyr_frd_z [rad/s],"
                     " acc_frd_x [m/s^2], acc_frd_y [m/s^2], acc_frd_z [m/s^2],"
                     " imu_temp_c [degC]\n")
        self.segments.append({"path": self.path, "t_first_us": t_us, "t_last_us": t_us,
                              "samples_in": 0, "filled": 0, "rows": 0})
        self.acc = [0.0] * 8
        self.k = 0

    def sample(self, t_us, v, temp, filled=False):
        seg = self.segments[-1]
        seg["samples_in"] += 1
        seg["filled"] += int(filled)
        seg["t_last_us"] = t_us
        if self.average == 1:
            self._row(t_us, v, temp)
            return
        a = self.acc
        if self.k == 0:
            a[7] = t_us
        for i in range(6):
            a[i] += v[i]
        a[6] += temp
        self.k += 1
        if self.k == self.average:
            n = self.average
            t_mid = a[7] + (t_us - a[7]) / 2
            self._row(round(t_mid), [x / n for x in a[:6]], a[6] / n)
            self.acc = [0.0] * 8
            self.k = 0

    def _row(self, t_us, v, temp):
        self.f.write(f"{t_us},{v[0]:.9g},{v[1]:.9g},{v[2]:.9g},"
                     f"{v[3]:.9g},{v[4]:.9g},{v[5]:.9g},{temp:.3f}\n")
        self.segments[-1]["rows"] += 1

    def close(self):
        if self.f:
            self.f.close()
            self.f = None

    def finalize(self):
        """Drop short segments, give the rest their final names."""
        self.close()
        keep = []
        for s in self.segments:
            s["duration_s"] = (s["t_last_us"] - s["t_first_us"]) / 1e6
            if s["t_last_us"] - s["t_first_us"] < self.min_us:
                os.remove(s["path"])
                s["dropped"] = True
            else:
                keep.append(s)
        for i, s in enumerate(keep):
            if len(keep) == 1:
                final = os.path.join(self.out, "imu.csv")
            else:
                d = os.path.join(self.out, f"seg{i:02d}")
                os.makedirs(d, exist_ok=True)
                final = os.path.join(d, "imu.csv")
            os.replace(s["path"], final)
            s["path"] = final
        return keep


def convert(args):
    with open(os.path.join(args.log_dir, "meta.json")) as f:
        meta = json.load(f)
    step = int(meta.get("step_us", 1250))
    start = 0 if args.from_start else meta.get("record_start_offset")
    if start is None:
        sys.exit("meta.json has no record start (the log stopped during the warm-up);"
                 " use --from-start")
    out = args.out or args.log_dir
    os.makedirs(out, exist_ok=True)
    max_fill_us = args.max_fill_ms * 1000

    acc_sign = (1, -1, -1) if args.acc_specific_force else (-1, 1, 1)
    w = Writer(out, args.average, args.min_minutes * 60)
    parser = ep.Parser()
    temp = float("nan")
    prev = None         # (t_us, values)
    last_raw_ts = None
    t_us = 0
    stats = {"gaps_filled": 0, "breaks": [], "duplicates": 0}

    raw_path = os.path.join(args.log_dir, "raw.bin")
    size = os.path.getsize(raw_path)
    done = 0
    with open(raw_path, "rb") as f:
        f.seek(start)
        while True:
            data = f.read(CHUNK)
            if not data:
                break
            done += len(data)
            for cmd, _, p in parser.feed(data):
                if cmd == ep.CMD_STATUS and len(p) >= ep.STATUS_LEN:
                    temp = ep.parse_status(p)[0]
                    continue
                if cmd != ep.CMD_RAW or len(p) != ep.RAW_LEN:
                    continue
                ts, g, a, _ = ep.parse_raw(p)
                v = (g[0], -g[1], -g[2],
                     acc_sign[0] * a[0] * G, acc_sign[1] * a[1] * G, acc_sign[2] * a[2] * G)
                if last_raw_ts is None:
                    t_us = ts
                    w.begin(t_us)
                else:
                    d = (ts - last_raw_ts) & 0xFFFFFFFF
                    if d == 0:
                        stats["duplicates"] += 1
                        continue
                    if d >= 0x80000000 or d > max_fill_us + step:
                        why = "backward step or restart" if d >= 0x80000000 \
                            else f"gap of {d / 1000:.1f} ms"
                        stats["breaks"].append({"after_t_us": t_us, "why": why})
                        t_us = ts if d >= 0x80000000 else t_us + d
                        w.begin(t_us)
                        prev = None
                    else:
                        n_miss = round(d / step) - 1
                        if n_miss > 0 and prev is not None:
                            stats["gaps_filled"] += 1
                            pt, pv = prev
                            for k in range(1, n_miss + 1):
                                fr = k / (n_miss + 1)
                                w.sample(pt + round(d * fr),
                                         [pv[i] + (v[i] - pv[i]) * fr for i in range(6)],
                                         temp, filled=True)
                        t_us += d
                w.sample(t_us, v, temp)
                prev = (t_us, v)
                last_raw_ts = ts
            if sys.stderr.isatty():
                sys.stderr.write(f"\r  {done / 1e6:.0f} / {(size - start) / 1e6:.0f} MB")
    if sys.stderr.isatty():
        sys.stderr.write("\n")

    segs = w.finalize()
    all_segs = w.segments
    summary = {"log_dir": os.path.abspath(args.log_dir), "from_offset": start,
               "average": args.average, "max_fill_ms": args.max_fill_ms,
               "acc": "specific force" if args.acc_specific_force else "gravity vector",
               "crc_errors": parser.crc_errors, "skipped_bytes": parser.skipped,
               **stats, "segments": all_segs}
    with open(os.path.join(out, "convert.json"), "w") as f:
        json.dump(summary, f, indent=2)

    print(f"CRC errors {parser.crc_errors}, skipped bytes {parser.skipped}, "
          f"gaps filled {stats['gaps_filled']}, duplicates {stats['duplicates']}, "
          f"segment breaks {len(stats['breaks'])}")
    for s in all_segs:
        state = "dropped (too short)" if s.get("dropped") else s["path"]
        print(f"  {s['duration_s'] / 3600:6.2f} h, {s['samples_in']} samples "
              f"({s['filled']} filled), {s['rows']} rows -> {state}")
    if not segs:
        sys.exit("no segment long enough")
    best = max(segs, key=lambda s: s["duration_s"])
    print(f"next: python external/INSLIB/tools/allan_variance.py {best['path']} "
          f"--plot {os.path.join(os.path.dirname(best['path']), 'allan.pdf')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index("- Starts"):])
    ap.add_argument("log_dir", help="directory written by tm210_allan_log.py")
    ap.add_argument("--out", help="output directory (default: the log directory)")
    ap.add_argument("--from-start", action="store_true",
                    help="include the warm-up, from the first byte of raw.bin")
    ap.add_argument("--average", type=int, default=1,
                    help="write the mean of each N samples (default 1)")
    ap.add_argument("--max-fill-ms", type=float, default=200,
                    help="longest gap to interpolate across (default 200 ms)")
    ap.add_argument("--min-minutes", type=float, default=10,
                    help="drop segments shorter than this (default 10)")
    ap.add_argument("--acc-specific-force", action="store_true",
                    help="the IMU reports specific force, not the gravity vector")
    args = ap.parse_args()
    if args.average < 1:
        ap.error("--average must be >= 1")
    convert(args)


if __name__ == "__main__":
    main()
