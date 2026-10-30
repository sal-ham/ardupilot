# RPLIDAR S2 backport for ArduCopter 4.6.3 / Cube Orange+

This custom firmware adds the UART RPLIDAR S2 to the existing RPLIDAR proximity
backend. Its version banner is `ArduCopter V4.6.3-S2`; the numeric vehicle version
remains 4.6.3. The baseline firmware and its workflow are kept separately.

## Source and scope

- Release base: [`Copter-4.6.3`, `92b0cd7`](https://github.com/ArduPilot/ardupilot/releases/tag/Copter-4.6.3).
- Model identification: [upstream commit `1e4efba`](https://github.com/ArduPilot/ardupilot/commit/1e4efbac530d38db87e2975725b0f961e0cc250d), already applied by the owner of this fork.
- Dense Express support: [upstream commit `b0070b3`](https://github.com/ArduPilot/ardupilot/commit/b0070b303eaafd8dfa6b2022171a87c1ed62bf1c), from [PR #31663](https://github.com/ArduPilot/ardupilot/pull/31663).
- Packet-angle reference: [SLAMTEC's Dense capsule decoder](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/dataunpacker/unpacker/handler_capsules.cpp).

The 4.6.3 interfaces `distance_min()` / `distance_max()` and its
`BOARD_FLASH_SIZE` build definition are retained. Newer-source names such as
`distance_min_m()` and `HAL_PROGRAM_SIZE_LIMIT_KB` are not introduced.
Existing A1/A2/C1/S1 scan selection stays on the standard scan path.

This is a sensor-driver change. It does not change BendyRuler, avoidance flight
modes, the bootloader, or saved vehicle parameters. The S2 shares `PRX1_TYPE=5`
with the existing A2-named backend.

## Changed files

| File | Change |
| --- | --- |
| `libraries/AP_Proximity/AP_Proximity_RPLidarA2.cpp` | S2 Express command, descriptor handling, buffered packet reception, checksums/resynchronisation, sample decoding and proximity/database updates. |
| `libraries/AP_Proximity/AP_Proximity_RPLidarA2.h` | Express state, declarations, 672-byte receive buffer and an 84-byte previous-capsule cache. |
| `libraries/AP_Proximity/AP_Proximity_config.h` | Express feature guard using the 4.6.3 flash-size definition. |
| `ArduCopter/version.h` | Custom S2 firmware banner. |
| `.github/workflows/build-cubeorangeplus-s2.yml` | Test and build the actual triggering branch commit, then upload firmware and logs. |
| `Tools/scripts/test_rplidar_s2.py` | Compile the real driver with a deterministic UART/proximity fixture, testing Express enabled and disabled. |
| `docs/rplidar-s2-4.6.3.md` | This implementation and bench-test guide. |

## How the S2 path works

1. Query device information and recognise model ID `0x71`.
2. Send `EXPRESS_SCAN` (`0x82`) with the five-byte payload and XOR checksum.
3. Recognise the Dense response descriptor `A5 5A 54 00 00 40 85`.
4. Accumulate 84-byte capsules; validate the sync nibbles and XOR checksum.
   Reject bad packets and resynchronise, with a bounded buffer and 1 ms update
   budget inherited from upstream.
5. Decode the previous capsule when the next capsule supplies its end angle.
   Remove the encoder-reset flag from the angle and handle angle wraparound.
6. Apply mounting direction/yaw correction, distance limits and ignore zones,
   then update the nearest distance in each proximity face and the obstacle
   database.

### Additional corrections for the requested narrow front view

The imported upstream implementation assumes a fixed 0.1125-degree spacing
between samples. This version instead measures the spacing between adjacent
capsules, following SLAMTEC's decoder, so a different scan speed does not move
samples into or out of the ignore zones. Discontinuities and bad checksums
invalidate the previous-capsule cache.

The upstream implementation also advances proximity faces only after accepting
a measurement. With only the front face accepted, this can prevent publishing
or refreshing that face. This version advances faces before the ignore filter,
publishes the completed face, and clears an empty face. Front readings can
therefore increase when an obstacle retreats and become invalid when it is
removed. An ignored direction remains unobserved; it is not declared clear.

## Building and downloading

Push to `copter-4.6.3-rplidar-s2` and follow **Build CubeOrangePlus - 4.6.3 S2**
in GitHub Actions. This workflow checks out the triggering commit. The older
baseline workflow intentionally checks out the fixed official release instead.

The original upstream workflows are preserved in `.github/upstream-workflows/`
on this branch so a sensor-driver push does not start the entire upstream CI
suite. Only the S2 workflow and the saved baseline workflow remain active.
If GitHub says workflows are paused on this fork because of Actions usage,
the repository owner must re-enable them on the Actions page before pushing
the next commit.

The S2 workflow runs the packet tests, uses the 4.6.3 dependency installer and
ARM GCC toolchain, and builds `CubeOrangePlus`. It explicitly enables both
RPLIDAR build flags. Before upload, it checks board ID **1063**, those flags,
and the custom version banner.

The `CubeOrangePlus-Copter-4.6.3-S2` artifact contains:

- `arducopter.apj`: the firmware package for Mission Planner.
- `arducopter.elf`: the debugging binary, copied from Waf's extensionless output.
- `build-manifest.json`: source commit, release base, board and build flags.
- `s2-extra-hwdef.dat`, `sha256.txt`, and this guide.

The `CubeOrangePlus-S2-logs` artifact contains validation and build logs. Success
confirms the tests and compilation for that commit; hardware checks below are
still required.

## Sensor configuration and bench checks

Use the UART S2 with a suitable regulated supply and common ground. Follow the
sensor's own electrical specifications; power capability is separate from the
UART signal connection. For the chosen flight-controller serial port `x`:

```text
SERIALx_PROTOCOL = 11
SERIALx_BAUD     = 1000
PRX1_TYPE       = 5
```

`1000` represents 1,000,000 baud in ArduPilot. Set the sensor's mounting
orientation and yaw correction before checking bearings. The 0.05 m / 50 m
driver bounds follow the upstream identification change; actual usable range
depends on the particular S2 and target. Set proximity range limits to match
the unit and operating conditions.

For the desired front view of approximately -15 to +15 degrees, after alignment:

```text
PRX1_IGN_ANG1 = 70
PRX1_IGN_WID1 = 110
PRX1_IGN_ANG2 = 180
PRX1_IGN_WID2 = 110
PRX1_IGN_ANG3 = 290
PRX1_IGN_WID3 = 110
PRX1_IGN_WID4 = 0
```

These parameters discard the other directions in software. The sensor keeps
scanning 360 degrees, and the mask does not restrict the directions in which an
avoidance planner can move the vehicle.

Bench-test with propellers removed. Confirm the custom version and S2 identity,
correct obstacle bearings, increasing/decreasing distances and removal of a
target. Repeat at the front-mask edges and verify side/rear targets are ignored.
Check that loss of sensor data reports `NoData`, and check scheduler load and
serial reliability at the intended scan speed. Hardware timing, optical
performance, weather resistance and flight behaviour are not established by
the packet fixture or a successful firmware build.

The copied upstream test suite already had simulation failures before this
backport (including PerfInfo and GyroFFT tests). They are separate from the
dedicated S2 build and remain baseline findings to investigate.

## Running the driver fixture locally

With Python 3 and a C++17 compiler:

```sh
python3 Tools/scripts/test_rplidar_s2.py --root . --cxx g++
```

It compiles unmodified copies of the three actual driver files. Its fixtures
replace the UART, clock and proximity/backend environment, covering scan
startup, checksum rejection, fragmented/noisy reception, descriptor handover,
full-scan sector updates, a 30-degree front mask, obstacle retreat/removal,
encoder-reset flags, variable angular spacing, range rejection, mounting
correction, timeout, bounded reception, serial-read error recovery, A2
compatibility and Express disabled.
It is not a complete SITL or flight-controller test.
