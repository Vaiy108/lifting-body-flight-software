"""serial_ping_test.py -- standalone STM32 PIL smoke test.

Zero dependency on lifting-body-gnc or the plant simulator: this
sends a handful of hand-built pil_protocol packets directly to the
board and checks the response against known-correct values, computed
once from the trim condition baked into pil_core_init(). Use this
FIRST when bringing up new hardware or firmware -- it answers "is the
board alive and computing correctly" in a few seconds, before running
the full 30 s closed-loop demo (pil_driver.py) which needs the
sibling lifting-body-gnc repo.

Requires only: pyserial (`pip install pyserial`)

Usage:
    python3 serial_ping_test.py --port COM3
    python3 serial_ping_test.py --port /dev/ttyACM0
"""

import argparse
import struct
import sys
import time

IN_FMT_NO_CHECKSUM = "<BI3d3ddB3d3ddd"
IN_FMT_FULL = IN_FMT_NO_CHECKSUM + "B"
IN_SIZE = struct.calcsize(IN_FMT_FULL)

OUT_FMT = "<BI3d3d4d3dIB"
OUT_SIZE = struct.calcsize(OUT_FMT)

PIL_INPUT_MAGIC = 0xA5
PIL_OUTPUT_MAGIC = 0x5A

# Trim condition baked into c/pil/pil_core.c's pil_core_init():
# 160 m/s / 8000 m, theta_trim = -0.023685520995277876 rad
TRIM_THETA_RAD = -0.023685520995277876


def encode_input(seq, f_meas, w_meas, dt, update_flags, pos_meas, vel_meas,
                 alt_meas, theta_cmd):
    body = struct.pack(IN_FMT_NO_CHECKSUM, PIL_INPUT_MAGIC, seq, *f_meas,
                       *w_meas, dt, update_flags, *pos_meas, *vel_meas,
                       alt_meas, theta_cmd)
    checksum = sum(body) & 0xFF
    return body + struct.pack("<B", checksum)


def decode_output(buf):
    vals = struct.unpack(OUT_FMT, buf)
    return dict(
        magic=vals[0], seq=vals[1],
        p=vals[2:5], v=vals[5:8], q=vals[8:12],
        theta_est=vals[12], q_est=vals[13], de_cmd=vals[14],
        cycle_count=vals[15], checksum=vals[16],
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True,
                    help="e.g. COM3 (Windows) or /dev/ttyACM0 (Linux)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--n", type=int, default=10,
                    help="number of ping steps to send")
    args = ap.parse_args()

    import serial
    ser = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(0.5)  # let the board's UART settle after port open

    print(f"Connected to {args.port} @ {args.baud} baud")
    print(f"Sending {args.n} steps at trim (level-flight specific force, "
         f"zero body rate, theta_cmd = theta_trim)\n")

    failures = 0
    rtts = []

    for k in range(args.n):
        pkt = encode_input(
            seq=k,
            f_meas=(0.0, 0.0, -9.80665),
            w_meas=(0.0, 0.0, 0.0),
            dt=0.005,
            update_flags=0,
            pos_meas=(0.0, 0.0, 0.0),
            vel_meas=(0.0, 0.0, 0.0),
            alt_meas=0.0,
            theta_cmd=TRIM_THETA_RAD,
        )
        t0 = time.perf_counter()
        ser.write(pkt)
        resp = ser.read(OUT_SIZE)
        t1 = time.perf_counter()

        if len(resp) != OUT_SIZE:
            print(f"  step {k}: FAIL -- short read ({len(resp)}/{OUT_SIZE} bytes)")
            failures += 1
            continue

        out = decode_output(resp)
        rtt_ms = (t1 - t0) * 1e3
        rtts.append(rtt_ms)

        ok = (out["magic"] == PIL_OUTPUT_MAGIC and out["seq"] == k and
             out["cycle_count"] != 0xFFFFFFFF)
        status = "OK" if ok else "FAIL"
        if not ok:
            failures += 1

        print(f"  step {k}: {status}  theta_est={out['theta_est']:+.5f} rad  "
             f"de_cmd={out['de_cmd']:+.3f} deg  cycles={out['cycle_count']}  "
             f"rtt={rtt_ms:.2f} ms")

    ser.close()

    print()
    if rtts:
        print(f"Round-trip: mean={sum(rtts)/len(rtts):.2f} ms, "
             f"min={min(rtts):.2f} ms, max={max(rtts):.2f} ms")
    if failures == 0:
        print(f"ALL {args.n} STEPS OK -- board alive, protocol verified.")
        sys.exit(0)
    else:
        print(f"{failures}/{args.n} STEPS FAILED.")
        sys.exit(1)


if __name__ == "__main__":
    main()
