#!/usr/bin/env python3
"""
Super-simple live TMF8829 depth visualiser over SWD.

Polls the `frame_buf` global in the running firmware via STM32CubeProgrammer
CLI and renders the 48x32 distance grid as a refreshing ANSI heatmap in the
terminal.  Throwaway tool — delete when the real host protocol exists.

Usage:  python3 tof_viz.py
Ctrl-C to quit.
"""
import os, re, subprocess, sys, time

CLI  = "/opt/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI"
NM   = "/opt/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin/arm-none-eabi-nm"
ELF  = os.path.join(os.path.dirname(__file__), "Debug", "dome_v2.elf")

# The firmware assembles both sub-frames into a stable full-resolution grid
# `tmf8829_grid` (1536 x uint16, distance in mm).  We read that directly.
GRID_W    = 48
GRID_H    = 32
GRID_N    = GRID_W * GRID_H
MAX_MM    = 2000   # clamp for the colour scale

def addr_of(sym):
    out = subprocess.check_output([NM, ELF], text=True)
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == sym:
            return int(parts[0], 16)
    raise SystemExit(f"symbol {sym} not found in {ELF}")

ANSI = re.compile(r"\x1b\[[0-9;]*m")

def read_mem(addr, nbytes):
    """Read nbytes from target RAM, return bytes. Uses one CLI invocation.

    The ST-Link can only serve one host at a time, so if another tool (e.g. the
    debugger or a flashing CLI) holds it, the read fails transiently.  Retry a
    few times instead of crashing the viewer."""
    out = None
    for _ in range(5):
        try:
            out = subprocess.check_output(
                [CLI, "-c", "port=SWD", "mode=HOTPLUG", "-r8", hex(addr), hex(nbytes)],
                text=True, stderr=subprocess.DEVNULL)
            break
        except subprocess.CalledProcessError:
            time.sleep(0.2)
    if out is None:
        return b""
    out = ANSI.sub("", out)
    data = bytearray()
    for line in out.splitlines():
        m = re.match(r"\s*0x[0-9A-Fa-f]{8}\s*:\s*(.+)", line)
        if m:
            for tok in m.group(1).split():
                if re.fullmatch(r"[0-9A-Fa-f]{2}", tok):
                    data.append(int(tok, 16))
    return bytes(data)

# 24-colour greyscale->turbo-ish ramp using ANSI 256 colours
RAMP = [16, 17, 18, 19, 20, 21, 27, 33, 39, 45, 51, 50, 49, 48, 47, 46,
        82, 118, 154, 190, 226, 220, 214, 208, 202, 196]

def cell(mm):
    if mm <= 0 or mm > MAX_MM*2:        # no/!invalid target
        return "\033[48;5;232m  \033[0m"
    f = min(mm, MAX_MM) / MAX_MM
    c = RAMP[int((1.0 - f) * (len(RAMP) - 1))]   # near=warm, far=cool
    return f"\033[48;5;{c}m  \033[0m"

def main():
    grid = addr_of("tmf8829_grid")
    gseq = addr_of("tmf8829_grid_seq")
    print(f"tmf8829_grid=0x{grid:08x}  grid_seq=0x{gseq:08x}")
    time.sleep(0.5)
    sys.stdout.write("\033[2J")
    while True:
        raw = read_mem(grid, GRID_N * 2)       # 1536 x uint16
        if len(raw) < GRID_N * 2:
            time.sleep(0.2); continue
        seq = int.from_bytes(read_mem(gseq, 4)[0:4], "little")
        dist = [raw[2*i] | (raw[2*i+1] << 8) for i in range(GRID_N)]

        out = ["\033[H"]   # cursor home
        out.append(f"TMF8829 48x32 full-res  snapshot#{seq}   (Ctrl-C quit)\n")
        for r in range(GRID_H):
            row = []
            for c in range(GRID_W):
                row.append(cell(dist[r*GRID_W + c]))
            out.append("".join(row) + "\n")
        out.append("near ")
        for mm in range(0, MAX_MM+1, MAX_MM//12):
            out.append(cell(mm if mm > 0 else 1))
        out.append(" far\n")
        sys.stdout.write("".join(out))
        sys.stdout.flush()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.stdout.write("\033[0m\n")
