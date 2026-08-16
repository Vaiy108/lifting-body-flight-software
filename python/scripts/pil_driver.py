"""PIL driver: runs the 30 s trimmed-glide closed-loop scenario against
an external process speaking the pil_protocol wire format, exactly as
the STM32 firmware will (Phase 4b). Two backends:

VENDORED, UNMODIFIED, from lifting-body-gnc (Phase 4a). Source of
truth: https://github.com/Vaiy108/lifting-body-gnc -- see that repo's
python/hlgnc/ for the plant model (aero, dynamics, sensors) this
script drives. Do not edit here; make changes upstream and re-vendor.

DEPENDENCY NOTE: this script imports `hlgnc`, which is NOT part of
this repo -- it runs the full 6DOF plant simulation from
lifting-body-gnc and only delegates the navigation/control step to
this repo's firmware/host-sim. To run it, clone lifting-body-gnc as a
sibling directory:

    lifting-body-flight-software/
    lifting-body-gnc/

and run this script with lifting-body-gnc/python on PYTHONPATH, e.g.:

    PYTHONPATH=../lifting-body-gnc/python python3 pil_driver.py --backend serial --port COM3

For a quick hardware smoke test with NO cross-repo dependency (just
pyserial), use serial_ping_test.py in this same folder instead.

  --backend hostsim  (default) : spawns c/build/host_sim as a
                                  subprocess, talks over its stdin/stdout
                                  pipes. No hardware required -- this is
                                  the "PIL without hardware yet" mode
                                  that validates the full protocol.
  --backend serial   : opens a real serial port (pyserial) to a
                        flashed STM32 Nucleo. Same wire format, same
                        driver logic -- only the transport changes.

The scenario matches python/scripts/demo_closed_loop.py: IMU at the
simulation rate, GNSS at 5 Hz, baro at 20 Hz, a 2 deg pitch-attitude
step commanded from the 160 m/s / 8 km trim point. The plant (HL20Vehicle)
still runs in Python; only the navigation+control step is delegated to
the external process.

Usage:
  python scripts/pil_driver.py                    # host_sim backend
  python scripts/pil_driver.py --backend serial --port /dev/ttyACM0
"""

import argparse
import struct
import subprocess
import sys
import pathlib
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hlgnc.sim import HL20Vehicle
from hlgnc.actuators import allocate
from hlgnc.sensors import Imu, Gnss, Baro
from hlgnc.dynamics import quat_to_euler, rk4_step, quat_to_dcm
from hlgnc.aero import ControlDeflections

# -- wire format, must match c/include/pil_protocol.h exactly --------
IN_FMT_NO_CHECKSUM = "<BI3d3ddB3d3ddd"
IN_SIZE_NO_CHECKSUM = struct.calcsize(IN_FMT_NO_CHECKSUM)
IN_FMT_FULL = IN_FMT_NO_CHECKSUM + "B"
IN_SIZE = struct.calcsize(IN_FMT_FULL)

OUT_FMT = "<BI3d3d4d3dIB"
OUT_SIZE = struct.calcsize(OUT_FMT)

PIL_UPDATE_GNSS = 0x01
PIL_UPDATE_BARO = 0x02


def encode_input(seq, f_meas, w_meas, dt, update_flags, pos_meas, vel_meas,
                 alt_meas, theta_cmd):
    body = struct.pack(IN_FMT_NO_CHECKSUM, 0xA5, seq, *f_meas, *w_meas, dt,
                       update_flags, *pos_meas, *vel_meas, alt_meas, theta_cmd)
    checksum = sum(body) & 0xFF
    return body + struct.pack("<B", checksum)


def decode_output(buf):
    vals = struct.unpack(OUT_FMT, buf)
    return dict(
        magic=vals[0], seq=vals[1],
        p=np.array(vals[2:5]), v=np.array(vals[5:8]), q=np.array(vals[8:12]),
        theta_est=vals[12], q_est=vals[13], de_cmd=vals[14],
        cycle_count=vals[15], checksum=vals[16],
    )


class HostSimBackend:
    def __init__(self, exe_path):
        self.proc = subprocess.Popen([str(exe_path)], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, bufsize=0)

    def step(self, packet_bytes):
        self.proc.stdin.write(packet_bytes)
        self.proc.stdin.flush()
        buf = self.proc.stdout.read(OUT_SIZE)
        if len(buf) != OUT_SIZE:
            raise RuntimeError(f"short read from host_sim: got {len(buf)}, "
                              f"expected {OUT_SIZE}")
        return buf

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=2)


class SerialBackend:
    def __init__(self, port, baud=115200):
        import serial  # pyserial; only imported if this backend is used
        self.ser = serial.Serial(port, baud, timeout=2)

    def step(self, packet_bytes):
        self.ser.write(packet_bytes)
        buf = self.ser.read(OUT_SIZE)
        if len(buf) != OUT_SIZE:
            raise RuntimeError(f"short read from serial: got {len(buf)}, "
                              f"expected {OUT_SIZE}")
        return buf

    def close(self):
        self.ser.close()


def run(backend, t_end=30.0, dt=0.005, seed=7):
    rng = np.random.default_rng(seed)
    veh = HL20Vehicle()
    V0, alt0 = 160.0, 8000.0
    alpha_deg, de_deg, gamma = veh.trim_glide(V=V0, alt=alt0)
    theta_trim = gamma + np.deg2rad(alpha_deg)
    x = veh.initial_state(V0, alt0, alpha_deg, gamma)
    veh.actuators.pos = allocate(np.array([0, de_deg, 0, 0, 0, 0], float))

    imu, gnss, baro = Imu(), Gnss(), Baro()
    theta_cmd = theta_trim + np.deg2rad(2.0)

    gnss_period = round(1 / gnss.cfg.rate_hz / dt)
    baro_period = round(1 / 20.0 / dt)
    n = int(t_end / dt)

    t_hist = np.zeros(n)
    theta_true = np.zeros(n)
    theta_est_hist = np.zeros(n)
    de_hist = np.zeros(n)
    pos_err = np.zeros((n, 3))
    round_trip_us = np.zeros(n)
    nak_count = 0

    for k in range(n):
        f_true = veh.specific_force(x, ControlDeflections(*veh.actuators.aero_deflections()))
        w_true = x[10:13]
        f_meas, w_meas = imu.sample(x, f_true, rng)

        update_flags = 0
        pos_meas = vel_meas = np.zeros(3)
        alt_meas = 0.0
        if k % gnss_period == 0:
            update_flags |= PIL_UPDATE_GNSS
            pos_meas, vel_meas = gnss.sample(x, rng)
        if k % baro_period == 0:
            update_flags |= PIL_UPDATE_BARO
            alt_meas = baro.sample(x, rng)

        pkt = encode_input(k, f_meas, w_meas, dt, update_flags,
                           pos_meas, vel_meas, alt_meas, theta_cmd)

        t0 = time.perf_counter()
        resp = backend.step(pkt)
        t1 = time.perf_counter()
        round_trip_us[k] = (t1 - t0) * 1e6

        out = decode_output(resp)
        if out["cycle_count"] == 0xFFFFFFFF:
            nak_count += 1
            de_cmd = 0.0  # hold neutral on a NAK'd step
        else:
            de_cmd = out["de_cmd"]

        aero_cmd = np.array([0.0, de_cmd, 0.0, 0.0, 0.0, 0.0])
        surf_cmd = allocate(aero_cmd)
        veh.actuators.step(surf_cmd, dt)
        d6 = veh.actuators.aero_deflections()
        defl = ControlDeflections(*d6)

        t_hist[k] = k * dt
        theta_true[k] = np.rad2deg(quat_to_euler(x[6:10])[1])
        theta_est_hist[k] = np.rad2deg(out["theta_est"])
        de_hist[k] = d6[1]
        pos_err[k] = out["p"] - x[0:3]

        x = rk4_step(lambda tt, xx: veh.derivatives(tt, xx, defl), 0.0, x, dt)

    return dict(t=t_hist, theta_true=theta_true, theta_est=theta_est_hist,
               de=de_hist, pos_err=pos_err, round_trip_us=round_trip_us,
               theta_cmd_deg=np.rad2deg(theta_cmd), nak_count=nak_count)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["hostsim", "serial"], default="hostsim")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    c_dir = pathlib.Path(__file__).resolve().parents[2] / "c"

    if args.backend == "hostsim":
        exe = c_dir / "build" / "host_sim"
        if not exe.exists():
            print(f"error: {exe} not found -- run `make all` in {c_dir} first",
                 file=sys.stderr)
            sys.exit(1)
        backend = HostSimBackend(exe)
    else:
        backend = SerialBackend(args.port, args.baud)

    try:
        result = run(backend)
    finally:
        backend.close()

    print(f"Backend: {args.backend}")
    print(f"NAK'd (checksum-failed) steps: {result['nak_count']}")
    print(f"Round-trip time: mean={result['round_trip_us'].mean():.1f} us, "
         f"p99={np.percentile(result['round_trip_us'], 99):.1f} us, "
         f"max={result['round_trip_us'].max():.1f} us")
    print(f"Final theta: true={result['theta_true'][-1]:.3f} deg, "
         f"est={result['theta_est'][-1]:.3f} deg, "
         f"target={result['theta_cmd_deg']:.3f} deg")
    print(f"Final nav position error: {result['pos_err'][-1]} m "
         f"(|.|={np.linalg.norm(result['pos_err'][-1]):.2f} m)")

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0, 0].plot(result["t"], result["theta_true"], label="true")
    ax[0, 0].plot(result["t"], result["theta_est"], "--", label="PIL estimate")
    ax[0, 0].axhline(result["theta_cmd_deg"], color="gray", ls=":", label="command")
    ax[0, 0].set_xlabel("t [s]"), ax[0, 0].set_ylabel("theta [deg]")
    ax[0, 0].set_title(f"Pitch attitude ({args.backend} backend)")
    ax[0, 0].legend()

    ax[0, 1].plot(result["t"], result["de"])
    ax[0, 1].set_xlabel("t [s]"), ax[0, 1].set_ylabel("de [deg]")
    ax[0, 1].set_title("Elevator command")

    ax[1, 0].plot(result["t"], np.linalg.norm(result["pos_err"], axis=1))
    ax[1, 0].set_xlabel("t [s]"), ax[1, 0].set_ylabel("|nav error| [m]")
    ax[1, 0].set_title("PIL navigation error vs. plant truth")

    ax[1, 1].plot(result["t"], result["round_trip_us"])
    ax[1, 1].set_xlabel("t [s]"), ax[1, 1].set_ylabel("round-trip [us]")
    ax[1, 1].set_title(f"PC<->{args.backend} round-trip time per step")

    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.suptitle(f"PIL demo: {args.backend} backend, dependency-free C GNC")
    fig.tight_layout()
    out_path = (pathlib.Path(__file__).resolve().parents[2] / "docs" /
               f"demo_pil_{args.backend}.png")
    fig.savefig(out_path, dpi=130)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
