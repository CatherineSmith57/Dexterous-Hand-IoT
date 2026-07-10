#!/usr/bin/env python3
"""Slight grip-and-return test — raw motor level, no calibration needed.

Moves each finger's mcp/pip motors by a small offset toward grip (flexion),
holds briefly, then returns to the starting position. Repeats N times.

Excluded motors: wrist(1), all abd(4,5,6,13,14), cmc(17)
Active motors:   mcp/pip for every finger (2,3,7,8,9,10,11,12,15,16)

Usage:
    .venv/bin/python scripts/slight_grip.py              # default: 0.15 rad, 3 cycles
    .venv/bin/python scripts/slight_grip.py --delta 0.1  # smaller movement
    .venv/bin/python scripts/slight_grip.py --cycles 5   # repeat 5 times
"""

import argparse
import sys
import time

# Ensure project root is on path so orca_core is importable from .venv
sys.path.insert(0, "/home/becharm/portable/working/orcahand/orca_core")

import numpy as np


# ---------------------------------------------------------------------------
# Motor layout
# ---------------------------------------------------------------------------

# Motors we will actually move (finger mcp + pip only)
ACTIVE_MOTORS = [2, 3, 7, 8, 9, 10, 11, 12, 15, 16]

# All motors (for connect / torque enable / read)
ALL_MOTORS = list(range(1, 18))


def main() -> int:
    parser = argparse.ArgumentParser(description="Slight grip-and-return (raw motor)")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Baud rate")
    parser.add_argument(
        "--delta", type=float, default=0.15,
        help="Radial displacement toward grip per cycle (default: 0.15 rad ≈ 8.6°)",
    )
    parser.add_argument(
        "--cycles", type=int, default=3,
        help="Number of grip+return cycles (default: 3)",
    )
    parser.add_argument(
        "--hold", type=float, default=0.8,
        help="Seconds to hold at gripped position (default: 0.8)",
    )
    parser.add_argument(
        "--speed", type=int, default=80,
        help="Moving speed for WritePosEx (default: 80)",
    )
    parser.add_argument(
        "--acc", type=int, default=60,
        help="Acceleration for WritePosEx (default: 60)",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Import here so --help works without feetech SDK
    # ------------------------------------------------------------------
    from orca_core.hardware.waveshare_client import WaveShareClient

    client = WaveShareClient(
        motor_ids=ALL_MOTORS,
        port=args.port,
        baudrate=args.baudrate,
    )

    print(f"Connecting to {args.port} …")
    client.connect()
    print("Connected. Reading initial positions …")

    # Read current positions of active motors only
    pos0_arr, vel0_arr, cur0_arr = client.read_pos_vel_cur()

    # Build ordered list matching ACTIVE_MOTORS
    pos0 = {}
    for mid in ALL_MOTORS:
        idx = client.motor_ids.index(mid) if mid in client.motor_ids else None
        if idx is not None:
            pos0[mid] = float(pos0_arr[idx])

    start_pos = np.array([pos0[mid] for mid in ACTIVE_MOTORS], dtype=np.float64)

    print(f"\nActive motors: {ACTIVE_MOTORS}")
    print(f"Starting positions (rad): {np.round(start_pos, 4)}")
    print(f"Delta: {args.delta:.3f} rad ({np.degrees(args.delta):.1f}°) toward grip")
    print(f"Cycles: {args.cycles}, hold: {args.hold}s\n")

    try:
        for cycle in range(1, args.cycles + 1):
            # --- Grip (subtract delta → flexion) ---
            grip_pos = start_pos - args.delta
            client.write_desired_pos(ACTIVE_MOTORS, grip_pos)
            print(f"  Cycle {cycle}/{args.cycles} — gripping…", end=" ", flush=True)
            time.sleep(args.hold)
            print("held.")

            # --- Return to start ---
            client.write_desired_pos(ACTIVE_MOTORS, start_pos)
            print(f"  Cycle {cycle}/{args.cycles} — returning…", end=" ", flush=True)
            time.sleep(args.hold)
            print("held.")
            print()

        # Final: back to exact starting position
        client.write_desired_pos(ACTIVE_MOTORS, start_pos)
        time.sleep(0.5)
        print("Done. All motors returned to starting position.")

    except KeyboardInterrupt:
        print("\nInterrupted — returning to start position…")
        client.write_desired_pos(ACTIVE_MOTORS, start_pos)
        time.sleep(0.5)
        return 130
    finally:
        client.set_torque_enabled(ALL_MOTORS, False)
        client.disconnect()
        print("Torque disabled, disconnected.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
