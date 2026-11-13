#!/usr/bin/env python3
"""Check that the timing-critical code never runs from flash (XIP).

Reads the disassembly the build writes (build/app.dis), follows direct calls
and branches from the roots below, and lists every reachable function whose
address is in flash (0x10000000-0x13ffffff). Long-branch veneers are followed
to the function they jump to. Calls through function pointers are not seen.

Usage: python tools/check_ram_path.py [build/app.dis] [root ...]
Exit status 1 if anything reachable is in flash.

See plans/web-config-server.md, "Memory layout".
"""
import re
import sys
from collections import deque

DEFAULT_ROOTS = ["core1_main", "pps_irq", "bmp581_irq0", "bmp581_irq1"]

FUNC_RE = re.compile(r"^([0-9a-f]{8}) <([^>]+)>:$")
BRANCH_RE = re.compile(r"\t(?:bl|blx|b|b\.w|b\.n|b[a-z]{2}(?:\.[wn])?)\s+[0-9a-f]+ <([^>+]+)(?:\+0x[0-9a-f]+)?>")
VENEER_RE = re.compile(r"^__(.+)_veneer$")


def is_flash(addr):
    return 0x10000000 <= addr < 0x14000000


def parse(path):
    addr = {}
    calls = {}
    cur = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            m = FUNC_RE.match(line)
            if m:
                cur = m.group(2)
                addr.setdefault(cur, int(m.group(1), 16))
                calls.setdefault(cur, set())
                continue
            if cur is None:
                continue
            m = BRANCH_RE.search(line)
            if m and m.group(1) != cur:
                calls[cur].add(m.group(1))
    return addr, calls


def main():
    args = sys.argv[1:]
    dis = args.pop(0) if args and args[0].endswith(".dis") else "build/app.dis"
    roots = args or DEFAULT_ROOTS
    addr, calls = parse(dis)

    seen = {}
    todo = deque()
    for r in roots:
        if r not in addr:
            print(f"root not found: {r}")
            continue
        seen[r] = None
        todo.append(r)
    while todo:
        fn = todo.popleft()
        targets = set(calls.get(fn, ()))
        v = VENEER_RE.match(fn)
        if v:
            targets.add(v.group(1))
        for t in targets:
            if t in addr and t not in seen:
                seen[t] = fn
                todo.append(t)

    bad = sorted((f for f in seen if is_flash(addr[f]) and not VENEER_RE.match(f)), key=lambda f: addr[f])
    print(f"{len(seen)} functions reachable from {', '.join(roots)}")
    if not bad:
        print("none in flash")
        return 0
    print(f"{len(bad)} in flash:")
    for f in bad:
        chain = []
        p = f
        while p is not None:
            chain.append(p)
            p = seen[p]
        print(f"  {addr[f]:08x} {f}   via {' <- '.join(chain[1:])}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
