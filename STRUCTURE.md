# Folder structure

How this project is laid out and what gets built in each part. See [PLAN.md](PLAN.md) for the
engineering rationale behind each phase.

```
bike_ride_data_collection/
├── PLAN.md                     Engineering plan — the why
├── STRUCTURE.md                This file — the where
│
├── apps/                       Buildable Zephyr applications, one per phase
│   ├── p0_imu_bringup/
│   ├── p1_sd_logger/
│   ├── p2_gps/
│   ├── p3_ble_service/
│   ├── p4_mcuboot_fota/
│   └── p5_tfm/
│
├── common/                     Code shared by every app (and mirrored in tools/)
│   ├── include/bikelog/        Public headers
│   ├── src/                    Implementations
│   └── dts/                    Shared devicetree fragments — partition layout lives here
│
├── boards/                     Out-of-tree board port (needed only by p5)
│   └── seeed/xiao_nrf54l15_tz/
│
├── tools/                      Offline Python analysis
│   ├── bikelog/                Package: parser, calibration, lean estimator, reports
│   └── tests/
│
├── data/                       Captured data — never firmware
│   ├── calibration/            Per-install bias + orientation runs
│   ├── bench/                  Stationary validation logs
│   └── rides/                  Real rides
│
├── keys/                       MCUboot signing keys — NEVER commit the private half
└── docs/                       Wire format spec, pinout, bring-up notes, validation records
```

---

## Phase order and dependencies

Each app **copies forward** from the previous one, so `p5_tfm` is the complete product. Every phase
is independently buildable and flashable so you can always go back to a known-good step.

```
p0_imu_bringup ──► p1_sd_logger ──► p2_gps ──► p3_ble_service ──► p4_mcuboot_fota ──► p5_tfm
   sensor            storage         time &        transport          updates          isolation
                                     speed
```

The ordering is dependency-driven: SMP runs over BLE, so the BLE stack has to work before FOTA can
ride on it. That splits mcumgr across two phases, which is convenient — `fs_mgmt` needs no
bootloader, so log download lands a phase before firmware update does.

| Phase | Adds | Depends on | Done when |
|---|---|---|---|
| **p0** | LSM6DS3TR-C via Zephyr sensor API, INT-triggered, console over USB-C CDC | — | 6-axis streaming at a verified 833 Hz |
| **p1** | IMU FIFO batching, microSD, binary format v1, battery sense, wake-on-motion sessions | p0 | A 10-min bench log parses cleanly in Python |
| **p2** | GPS on `uart21`, UTC into the header, fix records in the stream | p1 | Log has time-aligned speed and position |
| **p3** | BLE stack, ride GATT service, **SMP transport + `fs_mgmt` + `os_mgmt`**, summary-file writer | p2 | Post-ride summary pulled to the phone in under a minute |
| **p4** | MCUboot via sysbuild, signing keys, **`img_mgmt`**, test/confirm/revert | p3 | An image from nRF Connect Device Manager boots and confirms; a broken one auto-reverts |
| **p5** | TF-M / TrustZone. Out-of-tree board port, `/ns` target, BLE keys into ITS | p4 | Same functionality on the `/ns` target, RAM within 128 KB, FOTA still working |

**Not phase folders:** the Python toolkit is continuous work in `tools/` (it must exist by p1 to
validate the format), and ride validation produces data in `data/rides/` plus notes in `docs/`.

---

## What goes in each app folder

Standard Zephyr application layout:

```
apps/pN_name/
├── CMakeLists.txt              Pulls in ../../common/src
├── prj.conf                    Kconfig for this phase
├── boards/
│   └── xiao_nrf54l15_nrf54l15_cpuapp.overlay
├── sysbuild.conf               p4 onward only — MCUboot config
├── src/
│   └── main.c
└── README.md                   What this phase adds, how to verify (write when you start it)
```

Note the name collision: `apps/pN/boards/` holds *devicetree overlays* (a Zephyr convention —
overlays named after the board target are auto-applied), while the top-level `boards/` holds an
actual *board definition*. Different things, unavoidable naming.

### Build commands

```bash
# p0 – p3
west build -b xiao_nrf54l15/nrf54l15/cpuapp apps/p1_sd_logger -p

# p4 (MCUboot enters, so sysbuild)
west build -b xiao_nrf54l15/nrf54l15/cpuapp --sysbuild apps/p4_mcuboot_fota -p

# p5 (TrustZone: non-secure target from the out-of-tree board)
west build -b xiao_nrf54l15_tz/nrf54l15/cpuapp/ns --sysbuild apps/p5_tfm -p \
           -- -DBOARD_ROOT=$(pwd)
```

Flash and view logs over USB-C in every case — the onboard SAMD11 CMSIS-DAP means no external probe.

---

## `common/` — why it exists

Copy-forward duplication is fine for application logic, where seeing each step in isolation is the
point. It is **not** fine for the on-disk record format, because firmware and the Python parser must
agree exactly, forever. Format drift between a log written in p2 and a parser updated in p3 produces
silently wrong data, which is the worst possible failure for a data-collection project.

So the wire format is defined once and shared:

```
common/
├── include/bikelog/
│   ├── format.h        Record types, file header, block layout, magic, version  ◄── source of truth
│   ├── session.h       Wake-on-motion session state machine
│   ├── imu.h           LSM6DS3TR-C FIFO configuration and drain
│   ├── sdlog.h         4 KB block builder, CRC, fsync policy
│   └── gnss.h          GNSS subsystem wrapper
├── src/                Implementations of the above
└── dts/
    └── bikelog_partitions.dtsi     Flash layout — see below
```

`tools/bikelog/format.py` mirrors `format.h`, and `tools/tests/` asserts the two agree on struct
sizes and offsets. Bump `BIKELOG_FORMAT_VERSION` in the header on any change; the parser dispatches
on it so old logs stay readable.

### `common/dts/bikelog_partitions.dtsi`

The flash layout lives here, included by every app's board overlay, so it is written once.

Phases p0–p3 build without a bootloader, so the layout doesn't bite until p4. But per
[PLAN.md §7.6](PLAN.md), when it does land it should already be **TF-M-shaped** — `tfm_ps`,
`tfm_its`, and `tfm_otp` carved out, slots sized at ~644 KB rather than 664, and `slot0_s` /
`slot0_ns` sub-partitions declared even while unused:

```
mcuboot          64 KB
slot0           644 KB   ├─ slot0_s   ~200 KB  (TF-M, p5)
                         └─ slot0_ns  ~444 KB  (app)
slot1           644 KB   (combined S+NS update image)
tfm_ps           16 KB
tfm_its          16 KB
tfm_otp           8 KB
storage          36 KB
```

Then adding TF-M at p5 is a build-target change, not a partition migration under working code.

**Watch the flash budget from p1, not p4.** Phases p0–p3 have the whole 1428 KB available and will
happily grow past what fits later. The real ceiling is **~444 KB** of application flash once TF-M
lands. Check `west build` output against that number each phase rather than discovering it at p5.
Same for RAM: 188 KB now, **128 KB** after TF-M.

---

## `boards/` — the out-of-tree board port

Needed only at p5, but it lives here from the start because it is a prerequisite with a long lead
time.

The XIAO has no `ns` board variant — verified in
`zephyr/boards/seeed/xiao_nrf54l15/board.yml`, which declares only `xip` on `cpuflpr`. The nRF54L15
DK declares `ns` on `cpuapp`; the XIAO doesn't. Same silicon, missing board definition.

```
boards/seeed/xiao_nrf54l15_tz/
├── board.yml                                   Declares the ns variant on cpuapp
├── Kconfig.xiao_nrf54l15_tz
├── xiao_nrf54l15_tz_nrf54l15_cpuapp.dts        Secure / no-TrustZone target
├── xiao_nrf54l15_tz_nrf54l15_cpuapp_ns.dts     Non-secure target
├── xiao_nrf54l15_tz_nrf54l15_cpuapp_ns.yaml
└── xiao_nrf54l15_tz_nrf54l15_cpuapp_ns_defconfig
```

Copy-and-adapt from `zephyr/boards/nordic/nrf54l15dk/`.

Two deliberate choices here:

- **Out-of-tree, not a patch.** Editing `C:\ncs\v3.3.0\zephyr\boards\seeed\...` gets silently wiped
  by the next `west update`. `BOARD_ROOT` pointing at this project survives.
- **A distinct board name** (`xiao_nrf54l15_tz`) rather than shadowing the in-tree `xiao_nrf54l15`.
  Shadowing an in-tree board from `BOARD_ROOT` is fragile; a separate name makes which definition is
  in use unambiguous. Cost is that p5's overlay filenames differ from p0–p4's.

Worth upstreaming to Zephyr once it works.

---

## `tools/` — the Python side

```
tools/
├── requirements.txt
├── bikelog/
│   ├── format.py       Mirrors common/include/bikelog/format.h
│   ├── parse.py        Binary log → DataFrames, CRC-checked, tolerates a truncated tail
│   ├── calibrate.py    Solves bias + mounting quaternion
│   ├── lean.py         Complementary filter, lever-arm correction, g/cos(θ) check
│   ├── events.py       Braking and shift segmentation
│   ├── spectra.py      Spectrogram vs. estimated rpm — confirms the 2× line and no aliasing
│   └── report.py       Per-ride summary and plots
└── tests/              Format round-trip, truncated-file handling, synthetic-corner lean checks
```

`parse.py` must exist by the end of p1 — a log format nothing can read is not validated. `lean.py`
and `spectra.py` are what turn raw captures into the answers the project is actually for.

---

## Deferred: lever sensors

Clutch and front-brake hall sensors are **not in the phase list for now**. Two consequences worth
being explicit about:

- **Clutch data is not captured at all** in p0–p5. It's one of the five signals in
  [PLAN.md §1](PLAN.md), and nothing on the board can infer it.
- **Braking degrades to inference** from IMU deceleration and pitch rate. Usable, but it cannot
  separate braking from engine braking or a downhill gradient — and the CB350's slipper clutch
  deliberately softens engine braking, which muddies that further.

When it comes back, it slots in as `apps/px_levers/` between `p2_gps` and `p3_ble_service` — it only
adds two ADC channels and record types, so it doesn't disturb anything downstream. The two analog
pads stay reserved in the pin budget so nothing has to move.

---

## `keys/` and `data/`

`keys/` holds the MCUboot signing keypair generated at p4. **The private key must never be
committed**, and losing it means no device already in the field can ever be updated again. Back it
up somewhere off this machine before p4 ends.

`data/` is captured output, not source. Raw logs run ~37 MB/hour, so this directory grows fast —
keep it out of version control and archive elsewhere.

Suggested `.gitignore` when you initialise the repo:

```
build/
keys/*.pem
data/**/*.bin
tools/**/__pycache__/
.venv/
```
