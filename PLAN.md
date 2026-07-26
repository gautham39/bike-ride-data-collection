# Bike Ride Data Collection — Honda CB350 H'ness

**Project 1** of the nRF54L15 series. Formerly `e_3_i2c_imu` — promoted to first position because
every later exercise needs real ride data to be worth anything.

- **Board:** Seeed XIAO nRF54L15 **Sense** (LSM6DS3TR-C 6-DOF IMU + MSM261DGT006 PDM mic on board)
- **Bike:** Honda CB350 H'ness — 348.36 cc air-cooled single, SOHC, 21 PS @ 5,500 rpm, 30 Nm @ 3,000 rpm
- **Power:** self-contained LiPo. **Zero electrical connection to the motorcycle.**
- **Scope:** IMU + GPS + microSD logging, BLE, MCUboot + BLE FOTA, and TF-M / PSA security

This project absorbs what the parent plan had as `e_9` (bootloader) and `e_10` (BLE FOTA), and adds
TrustZone. That reordering is deliberate — see §12.

---

## 1. What this is

A ride logger that mounts to the CB350 and records what the bike and rider are doing, at a sample
rate high enough to reconstruct riding behaviour afterwards.

| Signal | Why it matters |
|---|---|
| Speed | Context for everything else; required input to the lean-angle estimate |
| Acceleration | Throttle application, roll-on aggression |
| Braking | Deceleration magnitude, trail-braking, fork dive |
| Clutch | Slip/feather patterns, shift timing, clutch-in coasting |
| Lean angle | Cornering style, max lean, side-to-side asymmetry, transition rate |

**Three principles drive everything below.**

**Capture, don't estimate.** Phase 1 logs raw sensor streams to microSD and derives nothing
on-board. Lean angle, braking events, shift detection — all computed offline in Python. If the
on-board filter is wrong, raw logs can be re-processed; filtered output is lost forever.

**Stay off the bike's harness.** The CB350 has dual-channel ABS and Honda Selectable Torque
Control, both reading the wheel speed sensors. The device is fully standalone: own battery, own
sensors, removable in a minute. Nothing is spliced. See §2.

**Never open the box.** The logger lives in a sealed IP65 enclosure under the seat. Every routine
interaction — firmware updates, log retrieval, status, session control — happens over BLE or
automatically. This is what makes FOTA a real requirement rather than a hobby flourish.

---

## 2. How each signal is actually obtained

Two of the five requested signals cannot come from the IMU. That's the main constraint on the build.

| Signal | Source | Notes |
|---|---|---|
| Acceleration | On-board LSM6DS3TR-C, longitudinal axis | Direct. Free. |
| Braking | IMU decel + pitch rate, plus a hall sensor on the front brake lever | The IMU sees deceleration but can't separate brakes from engine braking or a gradient — and the CB350's **slipper clutch** deliberately softens engine braking on downshifts, muddying the signature further. |
| Lean angle | Gyro roll rate + speed + yaw rate, fused offline | **Not** from the accelerometer — see §3. |
| Clutch | Hall sensor + magnet on the clutch lever | Nothing on the board can see this. Analog position beats a switch tap: it shows feathering and slip. |
| Speed | **GPS module over UART** | **Cannot** come from the IMU. Integrating accelerometer output to get velocity drifts to nonsense within seconds. |

### Why GPS rather than the bike's own speed signal

The CB350 already has wheel speed sensors — ABS is fitted front and rear. Tapping them is still the
wrong move: they feed **ABS and HSTC**, both safety systems, and loading those lines can set fault
codes, light the ABS lamp, or degrade behaviour you depend on in an emergency stop.

GPS (u-blox NEO-M8N, ~$12, 10 Hz) gives speed, position, course, **and absolute UTC time** — the
last makes multi-session alignment trivial and justifies the part on its own. Known weakness:
dropouts in tunnels and dense tree cover. If that matters, add an independent wheel hall sensor with
your own magnet later — still no harness splice.

---

## 3. The lean-angle problem

The one genuinely interesting piece of engineering here, and where most DIY bike loggers go wrong.

**The naive approach fails.** `lean = atan2(a_y, a_z)` from the accelerometer returns approximately
**zero in every steady corner.**

A bike in a stable turn leans precisely so the resultant of gravity and centripetal force points
along the bike's own vertical axis — that is what "balanced" means on two wheels. So a frame-mounted
accelerometer reads roughly `0` laterally and `g/cos(θ)` vertically, regardless of how far over you
are. It is blind to lean in exactly the situation you most want to measure. A generic AHRS filter
(Madgwick, Mahony) inherits the same flaw: it trusts the accelerometer to define "down," so in a
sustained corner it converges to reporting zero lean.

**What works** — fuse two independent estimates:

1. **Gyro roll integration** — accurate short-term, drifts over tens of seconds:
   `θ_gyro(t) = θ(t-1) + ω_roll · dt`
2. **Kinematic lean from speed and yaw rate** — no drift, valid in steady-state turns:
   `θ_kin = atan( v · ψ̇ / g )`
   **Second independent reason speed is mandatory.** Without it there's no drift reference and the
   gyro integration walks away.
3. **Complementary fusion**, `α ≈ 0.98` at 100 Hz: `θ = α · θ_gyro + (1 - α) · θ_kin`
4. **Zero-lean anchoring** — when stopped or tracking straight, pull toward the static accelerometer
   reading to kill accumulated bias.

**Consistency check to log:** in a steady turn `|a|` should read `g/cos(θ)` — 1.41 g at 45°. Log the
magnitude so this can be checked against the fused angle afterwards.

### Lever-arm correction (matters on this mount)

The roll axis runs roughly through the tyre contact patches, near ground level. A sensor under the
seat sits about **0.65 m above it**, so during roll *transients* it picks up `h · α` of lateral
acceleration that has nothing to do with lean.

A fast flick reaching 200 °/s over 0.2 s is `α ≈ 17.5 rad/s²`, giving `0.65 × 17.5 ≈ 11 m/s² ≈ 1.1 g`
of pure artifact. Not a rounding error. Correctable offline by differentiating gyro roll rate — but
only if the mount height `h` is measured and recorded in the file header.

---

## 4. Board specifics (XIAO nRF54L15 Sense)

Verified against the Seeed wiki and the in-tree Zephyr board definition. 128 MHz Cortex-M33 with
FPU/DSP and **TrustZone-M**, 256 KB SRAM on the SoC, 1524 KB RRAM, board rated −40 to 105 °C,
BLE 5.4+ radio on-chip.

1. **No external debug probe needed.** The board carries an onboard **SAMD11 running CMSIS-DAP**, so
   `west flash`, SWD debugging, and log output all work over USB-C. The parent plan listed a J-Link
   as effectively required — for this board it isn't.
2. **Battery sensing is already wired.** `AIN7` on **P1.14**, gated by a TPS22916 load switch enabled
   from **P1.15** (`vbat_pwr`). Enable, sample, disable. This resolves one channel of the parent
   plan's open AIN-mapping question: **AIN7 = P1.14 = VBAT, reserved.**
3. **Charging is on board** — solder a 3.7 V LiPo to the BAT pads, charges from USB-C at ~200 mA.

### Pin budget

No shield — `seeed_xiao_round_display` would disable `xiao_serial` (uart21) to claim D6/D7, and
dropping it frees that UART for GPS. **BLE, FOTA, and TF-M consume no pins.**

| Function | Bus / pins | Notes |
|---|---|---|
| IMU | On-board LSM6DS3TR-C, `i2c30` @0x6a, alias `imu0`, INT on P0.02 | Uses no header pins |
| microSD | `xiao_spi` = `spi00`: SCK **D8**, MISO **D9**, MOSI **D10**, CS **D3** | Fast instance (hfpll) |
| GPS | `xiao_serial` = `uart21`: TX **D6**, RX **D7** | Free because no shield |
| Clutch hall | `xiao_adc`, one analog pad | ⚠ AIN↔pad mapping unverified except AIN7 |
| Brake hall | `xiao_adc`, second analog pad | |
| Battery sense | AIN7 / P1.14, enable P1.15 | Confirmed |
| Session button | External IP67 button | Fallback only — see §5.4 |
| Status LED | On-board `led0` P2.00, **active low** | |
| Console / logs | `uart20` P1.09/P1.08 → USB-C CDC | ⚠ TF-M may want a UART — see §7.5 |
| BLE | On-chip radio | No pins |
| **Spare** | `xiao_i2c` = `i2c22`: **D4**/**D5** | Magnetometer or barometer later |

The Sense variant's PDM mic is available but **not used in Phase 1** — at road speed, wind noise over
an externally-mounted mic will swamp engine sound.

---

## 5. BLE

The radio earns its place by making the sealed enclosure practical. Three jobs:

### 5.1 SMP / mcumgr — the workhorse

`CONFIG_MCUMGR_TRANSPORT_BT=y` with these groups:

| Group | Kconfig | Gives you |
|---|---|---|
| Image | `MCUMGR_GRP_IMG` | **FOTA** — upload, list slots, test, confirm, revert |
| OS | `MCUMGR_GRP_OS` | Reset, echo, task stats |
| **Filesystem** | `MCUMGR_GRP_FS` | **Download logs off the SD card over BLE** |

That third one is the find: `fs_mgmt` means **no custom bulk-transfer protocol is needed**. The nRF
Connect Device Manager phone app already speaks all of it.

### 5.2 Throughput reality — and why sessions write two files

mcumgr is CBOR-framed over SMP, so it isn't fast. Even with 2M PHY, DLE, and a large MTU, expect
roughly **20–60 kB/s**.

| Transfer | Size | Over BLE @ 40 kB/s |
|---|---|---|
| Raw log, 2-hour ride | ~74 MB | **~31 minutes** ✗ |
| Decimated summary, 2-hour ride | ~1.4 MB | **~35 seconds** ✓ |
| Firmware image (S+NS combined) | ~450 KB | ~12 seconds |

So each session writes **two files**: `NNNN_raw.bin` (full 833 Hz archive, retrieved by pulling the
card) and `NNNN_sum.bin` (10 Hz decimated pose, speed, lever positions, all events — ~1/50th the
size). The summary costs ~0.7 MB/hour of card space and turns post-ride download from a chore into a
30-second habit.

### 5.3 Ride service — small custom GATT

Notify-based, deliberately small — status and control, not a data path:

- Live: speed, fused lean angle, max lean per side, battery %, GPS fix quality, free card space
- Control: start/stop session, insert a timestamped marker, trigger calibration runs

### 5.4 Session control without a button

The enclosure is sealed, so session control is **automatic**: the LSM6DS3TR-C has a wake-on-motion /
activity interrupt. Bike moves → logging starts. Still for N minutes → session closes cleanly and the
summary is finalised. BLE is the manual override; one external IP67 button is the last-resort
fallback and the way into service mode.

### 5.5 Power cost

Small, and dominated by GPS regardless. Advertising at 1 s is tens of microamps; connected with
10 Hz notifications is ~2–3 mA against GPS's 25–30 mA. **Don't gate BLE during logging** — it isn't
where the energy goes.

---

## 6. DFU and FOTA

### 6.1 Transport: BLE only

The nRF54L15 SoC **has no USB peripheral** — the USB-C port belongs to the SAMD11 debugger, not to
the target. USB CDC DFU is not an option. Choices are BLE SMP or MCUboot UART serial recovery, and
since the box is sealed under the seat, **BLE SMP is the only one that matters in the field.** Serial
recovery stays configured as a bench-only brick-recovery path.

Reference configs verified present in NCS:
`nrf/samples/dfu/single_slot/sysbuild/ble_mcumgr/boards/nrf54l15dk_nrf54l15_cpuapp.overlay` and the
`fw_loader/ble_mcumgr` `pm_static` YAML. Same SoC as the DK, so they port directly.

### 6.2 Update flow

1. Phone uploads the signed image over BLE SMP into **slot1**.
2. Mark it *test* and reset. MCUboot swaps and boots the new image.
3. App proves itself, then **confirms**. If it never confirms — crash, boot loop, watchdog reset —
   MCUboot **automatically reverts** on the next boot.

That revert path is the whole reason to do this properly. A logger that bricks itself inside a sealed
box under the seat is a seat-off, box-open, cable-in recovery job. **The self-confirm must be real**:
GPS talking, card mounted and writable, IMU responding. Confirming in `main()` defeats the purpose.
Pair with the `wdt31` watchdog so a hung image fails to confirm rather than sitting half-alive.

### 6.3 Security layers

1. **Signed images** — MCUboot verifies ECDSA-P256 or Ed25519. Non-optional. Generate the keypair
   once, back it up, never commit the private half.
2. **Require pairing for SMP** — `CONFIG_MCUMGR_TRANSPORT_BT_AUTHEN=y` so an unbonded phone can't
   touch image or filesystem management.
3. **Service mode** — hold the external button at boot to enable SMP advertising. Normal riding
   sessions don't advertise it at all.

---

## 7. TF-M / TrustZone security

### 7.1 What TF-M actually does

The nRF54L15's Cortex-M33 implements **ARM TrustZone-M**, which splits the chip into two worlds
*in silicon*:

| | Secure world (SPE) | Non-secure world (NSPE) |
|---|---|---|
| Runs | TF-M | Your Zephyr logger app |
| Holds | Keys, crypto, protected storage | Everything else |
| Can reach the other side? | Yes | **No — blocked by hardware** |

**Trusted Firmware-M** is ARM's open-source reference implementation of PSA (Platform Security
Architecture) that lives in the secure world. It boots first, configures the hardware boundary, then
starts your application in the non-secure world.

The enforcement is the point. On the nRF54L15 the **MPC** (memory protection controller) and SAU
partition RAM, RRAM, and peripherals between the two worlds, and the CPU physically refuses a
non-secure access to a secure address — it faults. This is not a software convention that a bug can
sidestep; it is the same class of guarantee as an MMU. The only way across is through **NSC veneers**,
a small fixed table of entry points that TF-M explicitly exports.

TF-M then offers **PSA services** through that narrow door:

| Service | What it gives you |
|---|---|
| **PSA Crypto** | Sign, verify, encrypt, derive. Keys are referenced by *handle* — the app asks the secure world to sign something and never sees the key material. |
| **Internal Trusted Storage (ITS)** | Small confidential storage for keys and secrets |
| **Protected Storage (PS)** | Larger encrypted + authenticated storage |
| **Initial Attestation** | A signed token proving device identity and what firmware is running |
| **Firmware Update** | Standardised PSA FWU API |

### 7.2 What it buys *this* project, honestly

The device has two places untrusted bytes enter: the **NMEA parser** chewing on GPS serial data, and
the **BLE SMP handler** accepting connections and firmware from the air. Those are exactly the code
you'd expect to have a buffer bug.

With TF-M, a memory-corruption bug in either one is contained. The attacker gets the non-secure world
— annoying — but the BLE bonding keys sit in ITS on the other side of a hardware wall, and the crypto
that validates the next firmware image runs where they can't reach it. Without TF-M, one bad parse
owns the whole chip including every key in it.

Concretely worth having here:

- **BLE bonding keys in ITS** rather than in settings storage the app can read.
- **PSA Crypto for the SMP link** — the app never handles key material.
- **Optional log encryption** — your GPS traces are a map of where you live, work, and ride. If the
  bike is stolen with the card in it, that's real. PS can encrypt at rest with a device-bound key.

Be honest about the rest: there's no sophisticated adversary targeting a personal bike logger, and
attestation is pointless for a fleet of one. **The strongest argument is that this is a learning
ladder and PSA/TrustZone is a genuinely valuable skill** — increasingly required by PSA Certified and
the EU Cyber Resilience Act. It's worth doing here for that reason plus the GPS-privacy angle. It is
not worth pretending the threat model demands it.

### 7.3 The cost — verified against your install

Two hard numbers, read from `C:\ncs\v3.3.0`, not estimated.

**RAM.** `zephyr/dts/vendor/nordic/nrf54l15_cpuapp_ns_partition.dtsi` sets the default split:

```
sram0_s   128 KB   secure    (TF-M)
sram0_ns  128 KB   non-secure (your app)
```

**Your app drops from 188 KB to 128 KB.** That's the single biggest consequence. Correcting the
budget in §7.4 below: the earlier estimate of ~79 KB fits in 128 KB with ~49 KB headroom, so this is
workable — but the margin is real and BLE buffers are what will eat it.

Note the TrustZone layout uses all 256 KB split 128/128, with **no FLPR reservation**. The parent
plan's optional FLPR coprocessor idea is incompatible with this layout.

**Flash.** The default TrustZone layout has *no MCUboot and no second slot* — it consumes the entire
1524 KB as one secure + one non-secure image. That conflicts directly with what §6 needs.

The good news, verified in `nrf/modules/trusted-firmware-m/tfm_boards/partition/region_defs.h`:
**NCS supports MCUboot + TF-M + dual-slot together.** `NRF_NS_SECONDARY` is a real, supported
configuration, `slot0_partition` can be a combined MCUboot slot containing `slot0_s_partition` and
`slot0_ns_partition` sub-partitions, and the secondary slot holds the combined S+NS image
(`SECONDARY_PARTITION_SIZE = FLASH_S_PARTITION_SIZE + FLASH_NS_PARTITION_SIZE`).

So a workable layout inside the 1428 KB the board exposes:

```
mcuboot          64 KB
slot0           644 KB   ├─ slot0_s   ~200 KB  (TF-M)
                         └─ slot0_ns  ~444 KB  (app)
slot1           644 KB   (combined S+NS update image)
tfm_ps           16 KB
tfm_its          16 KB
tfm_otp           8 KB
storage          36 KB
```

App flash goes from 664 KB to ~444 KB. Against the ~340 KB estimate in §7.4 that still fits, with
less room than before.

### 7.4 Budget check, corrected for TF-M

| Component | Flash | RAM |
|---|---|---|
| Zephyr kernel + drivers | ~40 KB | ~10 KB |
| BLE controller + host | ~180 KB | ~35 KB |
| mcumgr + SMP + img/os/fs | ~35 KB | ~6 KB |
| FATFS + SD + block buffers | ~30 KB | ~16 KB |
| GNSS, sensors, I2C/ADC | ~20 KB | ~4 KB |
| Application | ~35 KB | ~8 KB |
| **App total** | **~340 KB of ~444 KB** | **~79 KB of 128 KB** |

Both fit, neither is generous. BLE buffer counts and MTU are the first dials if RAM gets tight — at
the cost of §5.2 throughput.

### 7.5 Two porting gaps must be closed first

**This is the part to understand before committing.** TF-M cannot be built for this board today.

**Gap 1 — the XIAO has no `ns` board variant.** Read from
`zephyr/boards/seeed/xiao_nrf54l15/board.yml`:

```yaml
socs:
- name: nrf54l15
  variants:
  - name: xip
    cpucluster: cpuflpr        # ← that's all there is
```

Compare `zephyr/boards/nordic/nrf54l15dk/board.yml`, which has `- name: ns / cpucluster: cpuapp`.
So `nrf54l15dk/nrf54l15/cpuapp/ns` exists and `xiao_nrf54l15/nrf54l15/cpuapp/ns` does not.

**This is a board-definition gap, not a silicon limitation** — the SoC is identical and Nordic's own
DK exposes the target. Closing it means adding, by copy-and-adapt from the DK:

- `board.yml`: an `ns` variant on `cpuapp`
- `xiao_nrf54l15_nrf54l15_cpuapp_ns.dts` — includes `nrf54l15_cpuapp_ns.dtsi`, chooses
  `slot0_ns_partition` for code and `sram0_ns` for RAM, switches entropy to `psa_rng`
- `..._ns.yaml` and `..._ns_defconfig`

**Gap 2 — TF-M platform target.** Upstream TF-M has
`modules/tee/tf-m/.../nordic_nrf/nrf54l15dk_nrf54l15_cpuapp/` but nothing for the XIAO. It's a thin
wrapper over `nordic_nrf/common/nrf54l15/`, so an equivalent is small — but confirm whether NCS's own
`tfm_boards` layer already handles any nRF54L15 board before writing one. Find out by attempting the
build once Gap 1 is closed.

**Do this out-of-tree.** Patching `C:\ncs\v3.3.0\zephyr\boards\seeed\...` gets silently wiped by the
next `west update`. Put a forked board under a project-local `boards/` directory and point
`BOARD_ROOT` at it. Worth upstreaming to Zephyr afterwards.

**One console conflict to watch:** the DK's `_ns.dts` disables `uart30` specifically so TF-M can claim
it for secure logging. The XIAO's console is `uart20`. Decide early whether TF-M gets its own UART or
runs silent, because on this board there's no spare pinned-out UART once GPS has `uart21`.

### 7.6 Sequencing: reserve the space now, enable TF-M later

TF-M changes both the flash layout and the RAM budget. Adding it after FOTA is already working would
mean re-laying-out partitions under working code — exactly the migration you don't want.

Doing TF-M *first* is also wrong: closing two board-porting gaps before you have a single byte of ride
data is a lot of yak-shaving with nothing to show.

**So do neither.** MCUboot arrives at p4, and when it does, define the partition layout **as if TF-M
were already present** — carve `tfm_ps` / `tfm_its` / `tfm_otp`, size the slots at ~644 KB, and
declare the `slot0_s` / `slot0_ns` sub-partitions even though nothing uses them yet. Then the
FOTA-critical addresses (mcuboot, slot0 start, slot1 start, storage) **never move**, and enabling
TF-M at p5 is a build-target change rather than a migration.

The layout lives in one place — `common/dts/bikelog_partitions.dtsi`, included by every app's board
overlay — so there is exactly one definition to get right.

---

## 8. Sensor configuration — and why 833 Hz, not 416 Hz

- **Accel range ±8 g** — Indian road bumps and potholes routinely exceed ±4 g at the frame.
- **Gyro range ±500 dps** — aggressive transitions reach 200–300 °/s roll rate.
- **ODR 833 Hz.**

That last one is specific to this engine. The CB350 is a **single cylinder**, and Honda fits a
**coaxial primary balancer shaft** that cancels the 1× imbalance. A single balancer cancels primary
only — the **secondary, at 2× crank speed, survives** and becomes the dominant residual. Riders report
vibration appearing above ~3,500 rpm, which is consistent.

Assuming a rev limit near 6,500 rpm (confirm against the tachometer):

| Component | At 5,500 rpm (peak power) | At 6,500 rpm |
|---|---|---|
| Firing, 0.5× | 46 Hz | 54 Hz |
| Primary, 1× | 92 Hz *(balancer-cancelled)* | 108 Hz *(cancelled)* |
| **Secondary, 2×** | **183 Hz** | **217 Hz** |

At 416 Hz ODR the Nyquist limit is 208 Hz — **directly on top of the 2× component at high revs.**
Above ~6,250 rpm that folds engine vibration into the sub-10 Hz band where braking and lean live.
833 Hz gives a 416 Hz Nyquist and comfortable margin. Decimate offline.

**Use the IMU's FIFO.** At 833 Hz, per-sample interrupts mean 833 IRQ/s; a watermark of ~170 samples
cuts that to ~5 IRQ/s — which matters more now that the radio and TF-M context switches want cycles.
Caveat: Zephyr's `st,lsm6dsl` driver is the older, simpler one and its FIFO support may be inadequate;
if so, use it for setup and drop to direct I2C register access.

---

## 9. Data format, rates, and storage

Binary, not CSV. 512-byte file header, then type-tagged records buffered into 4 KB blocks, each with a
monotonic tick and a CRC. Header holds: magic + version, **firmware version and image hash**, session
start UTC, ODRs and ranges, IMU bias offsets, mounting quaternion, and mount height `h`.

Recording the firmware version matters once FOTA is live — otherwise you can't tell which build
produced a log, and a behaviour change becomes impossible to attribute.

| Record | Rate | Size | Throughput |
|---|---|---|---|
| IMU (ax,ay,az,gx,gy,gz as int16) | 833 Hz | 12 B | 9.76 KB/s |
| GPS fix (lat, lon, speed, course, hdop) | 10 Hz | 24 B | 0.24 KB/s |
| Clutch + brake analog | 100 Hz | 4 B | 0.40 KB/s |
| Events, battery, markers | on change | 8 B | negligible |
| **Raw file total** | | | **≈ 10.4 KB/s ≈ 37 MB/hour** |
| **Summary file** (§5.2) | 10 Hz | 20 B | **≈ 0.7 MB/hour** |

A 32 GB card holds ~800 hours of both. Space is a non-issue; **integrity and vibration are.**

- **Block CRCs** mean a truncated file still parses to the last good block, losing at most 4 KB.
- **Own battery + auto session control** means killing the ignition never interrupts a write.
- **Secure the microSD card mechanically.** A single above 3,500 rpm can provoke contact bounce in a
  push-push socket. Tape or silicone, plus write-retry handling.

---

## 10. Power budget and the battery

| Consumer | Average draw |
|---|---|
| GPS module (tracking) | 25–30 mA |
| nRF54L15 active, radios off | 5–8 mA |
| microSD (4 KB block writes, averaged) | 3–6 mA |
| LSM6DS3TR-C, accel + gyro high-performance @ 833 Hz | ~1.3 mA |
| BLE advertising / connected with notifications | 0.05–3 mA |
| **Total** | **≈ 42–53 mA** |

**GPS dominates; BLE and TF-M barely register.** TF-M adds context-switch overhead on PSA calls, which
are rare here — not a power factor.

- **1000 mAh → ~20–24 h** · **2000 mAh → ~40–47 h**

A typical ride is 1–3 hours, so 1000 mAh covers many rides — and at ~200 mA it refills in 5–6 h where
2000 mAh takes 10+. **1000 mAh is the better trade.** Log battery voltage from AIN7 and expose it over
BLE.

### Heat is the real constraint

The board is rated to 105 °C. **The LiPo is not** — typically 0–45 °C charging, up to ~60 °C discharge,
and under-seat on an air-cooled single in Indian summer traffic will approach that.

- Mount in the **tail section, away from the engine and exhaust side.**
- Don't leave it charging on the bike in direct sun.
- Treat the LiPo as a consumable.
- LiFePO4 has a wider range but sits at 3.2 V nominal, below the board's 3.7 V minimum — **not a
  drop-in substitute.**

---

## 11. Mounting on the CB350

**Under the seat, on the subframe, near the centreline.** The H'ness has a removable seat with the
battery and a tool tray beneath, and accessible subframe rails. Rigid, weather-protected, laterally
close to the roll axis, far from cylinder heat.

- **Frame, not handlebars.** Bar mounts add steering input and bar vibration on top of chassis motion.
- **Rigid mount for the board.** A compliant mount is an uncontrolled mechanical filter that destroys
  the vibration data. Isolate the *enclosure* if you like, never the PCB inside it.
- **Plastic/ABS enclosure, IP65+.** Metal blocks GPS *and* attenuates BLE. Seat foam and vinyl are
  RF-transparent; a metal side panel is not.
- **Record the mount height** above the contact-patch line for the §3 correction.
- **Lever sensors:** magnets bonded to the clutch and front brake levers, hall sensors on the perch
  clamps, thin cable along the existing harness with zip ties. Nothing spliced.

---

## 12. Implementation phases

Ordering is dependency-driven: SMP runs over BLE, so the stack has to work before FOTA can ride on
it. That conveniently splits mcumgr across two phases — `fs_mgmt` needs no bootloader, so log
download lands a phase before firmware update does. Folder layout in [STRUCTURE.md](STRUCTURE.md).

| Phase | Scope | Done when |
|---|---|---|
| **p0** | IMU bring-up. `imu0` via Zephyr sensor API, INT-triggered, over USB-C CDC. | 6-axis streaming at a verified 833 Hz |
| **p1** | FIFO batch reads + microSD + binary format v1 + battery sense + wake-on-motion sessions. Python parser lands with it. | A 10-minute bench log parses cleanly in Python |
| **p2** | GPS on `uart21`, UTC into the header, fix records in the stream. | Log contains time-aligned speed and position |
| **p3** | BLE stack, ride GATT service, **SMP transport + `fs_mgmt` + `os_mgmt`**, summary-file writer. | Post-ride summary on the phone in under a minute, no cable |
| **p4** | **MCUboot + sysbuild + `img_mgmt` FOTA**, on a **TF-M-shaped partition layout** (§7.6). Signing keys, slot swap, test/confirm/revert. | An image from nRF Connect Device Manager boots and confirms; a deliberately-broken one auto-reverts |
| **p5** | **TF-M.** Out-of-tree board port (§7.5 Gaps 1–2), migrate to the `/ns` target, move BLE keys into ITS, optionally encrypt logs via PS. | Same functionality on the `/ns` target, RAM within 128 KB, FOTA still working |

Running alongside rather than as phases: the **Python toolkit** (§13) must exist by the end of p1 —
a log format nothing can read isn't validated — and **real rides** start as soon as p2 gives usable
speed, feeding validation notes back into every later phase.

p0 is essentially the old `e_3_i2c_imu` exercise. p4 and p5 absorb the parent plan's `e_9` and
`e_10`.

**Deferred: clutch and brake lever sensors.** Not in the phase list for now. Two consequences to be
clear about: **clutch is not captured at all** — it's one of the five signals in §1 and nothing
on-board can infer it — and **braking degrades to inference** from deceleration and pitch rate,
which cannot separate brakes from engine braking or a gradient. When it returns it slots between p2
and p3, adding only two ADC channels and record types. The analog pads stay reserved in §4 so
nothing has to move.

**Watch the flash and RAM ceilings from p1, not p4.** Phases p0–p3 build without a bootloader and
have the full 1428 KB and 188 KB available. The real budget once TF-M lands is **~444 KB flash and
128 KB RAM** (§7.4). Check build output against those numbers every phase rather than discovering
the gap at p5.

---

## 13. Offline toolkit

A `tools/` directory alongside the firmware:

- `parse.py` — binary log → pandas DataFrames, CRC-checked, tolerant of a truncated tail
- `calibrate.py` — solves bias and the mounting quaternion from the two calibration runs
- `lean.py` — the §3 complementary filter, with lever-arm correction and the `g/cos(θ)` check
- `events.py` — braking, shift, and clutch-slip segmentation
- `spectra.py` — spectrogram of raw IMU vs. estimated rpm; confirms the 2× line lands where §8
  predicts and that nothing is aliased
- `report.py` — per-ride summary: max lean each direction, **lean asymmetry**, hardest braking event,
  time-at-lean histogram, speed over the GPS track

The lean-asymmetry number is worth calling out: nearly every rider leans further one way than the
other, and few know which.

---

## 14. Calibration

Results written into the file header. Skipping step 2 is the classic mistake — a board mounted a few
degrees off reads a constant phantom lean.

1. **Bias** — stationary and vertical on the centre stand: accel and gyro offsets over ~10 s.
2. **Mounting orientation** — stationary-vertical gives 2 of 3 rotational DOF from gravity; a
   straight-line acceleration run gives the third. Store as a quaternion; offline tools rotate every
   sample into bike-frame coordinates first.
3. **Lever endpoints** — hall ADC values at fully released and fully pulled, both levers.

All three are triggerable over the BLE ride service, so recalibration after a remount doesn't mean
opening the box.

---

## 15. Firmware architecture (Zephyr)

```
IMU thread      ── FIFO watermark IRQ (P0.02) → drain batch ─┐
GPS thread      ── UART RX → NMEA parse ────────────────────┤
ADC thread      ── 100 Hz timer → clutch + brake ───────────┼──► ring_buf ──► Logger ──► SD
Battery task    ── 0.1 Hz: enable P1.15, read AIN7, disable ┤                    │
Wake-on-motion  ── LSM6DS3TR-C activity IRQ → session ──────┘                    ├──► summary writer
                                                                                  │
BLE thread      ── SMP (img/os/fs) + ride GATT ───────────────────────────────────┘
Watchdog        ── wdt31, fed by the logger; failure to feed blocks FOTA confirm
                                    │
                          ═══════ NSC veneers ═══════   ← hardware boundary (P8)
                                    │
TF-M (SPE)      ── PSA Crypto · ITS (BLE keys) · PS (log encryption)
```

- Zephyr's **GNSS subsystem** (`CONFIG_GNSS=y`, generic NMEA driver) rather than hand-parsing.
- The logger thread is the only filesystem writer; `fs_mgmt` reads must take a lock against it.
- LED (active low): slow blink = idle, solid = logging, double-blink = BLE connected, fast blink =
  SD error / no card / no fix.

Key Kconfig — logger and BLE: `CONFIG_I2C`, `CONFIG_SENSOR`, `CONFIG_LSM6DSL`, `CONFIG_SPI`,
`CONFIG_DISK_ACCESS`, `CONFIG_SDMMC_OVER_SPI`, `CONFIG_FAT_FILESYSTEM_ELM`, `CONFIG_FS_FATFS_LFN`,
`CONFIG_ADC`, `CONFIG_GNSS`, `CONFIG_RING_BUFFER`, `CONFIG_CRC`, `CONFIG_WATCHDOG`, `CONFIG_BT`,
`CONFIG_BT_PERIPHERAL`, `CONFIG_BT_SMP`, `CONFIG_SETTINGS`.

Bootloader and DFU: `CONFIG_BOOTLOADER_MCUBOOT`, `CONFIG_MCUMGR`, `CONFIG_MCUMGR_TRANSPORT_BT`,
`CONFIG_MCUMGR_TRANSPORT_BT_AUTHEN`, `CONFIG_MCUMGR_GRP_IMG`, `CONFIG_MCUMGR_GRP_OS`,
`CONFIG_MCUMGR_GRP_FS`, `CONFIG_IMG_MANAGER`, `CONFIG_STREAM_FLASH`, `CONFIG_FLASH_MAP`,
`CONFIG_ZCBOR`, `CONFIG_NET_BUF`.

TF-M (P8): `CONFIG_BUILD_WITH_TFM`, `CONFIG_TFM_PROFILE_TYPE_*`, `CONFIG_TFM_PARTITION_CRYPTO`,
`CONFIG_TFM_PARTITION_INTERNAL_TRUSTED_STORAGE`, `CONFIG_TFM_PARTITION_PROTECTED_STORAGE`,
`CONFIG_PSA_CRYPTO_*`, plus `-DNRF_NS_SECONDARY=y` for dual-slot.

---

## 16. Open items

1. **CB350 rev limit** — §8 assumes ~6,500 rpm. Confirm from the tachometer; if higher, re-check the
   833 Hz margin.
2. **AIN → pad mapping** for clutch and brake. AIN7/P1.14 is confirmed as VBAT and reserved; the rest
   is fixed in silicon and needs confirming against the datasheet and Seeed's pinout.
3. **Zephyr `st,lsm6dsl` FIFO support** — verify against driver source; fall back to direct register
   access if it can't do watermark batching (§8).
4. **XIAO `ns` board variant** — must be authored out-of-tree before TF-M builds at all (§7.5 Gap 1).
5. **TF-M platform target for the XIAO** — determine whether NCS's `tfm_boards` layer covers it or
   whether an upstream-style platform dir is needed (§7.5 Gap 2).
6. **TF-M secure console** — the DK frees `uart30` for TF-M; decide what the XIAO gives it, or run it
   silent (§7.5).
7. **RAM at 128 KB with TF-M** — §7.4 estimates ~79 KB. Measure at first `/ns` link; BLE buffers and
   MTU are the dials.
8. **`fs_mgmt` vs. active logging** — simplest correct answer: refuse downloads while logging.
9. **Mount height `h`** — measure at install, record in the header (§3).
10. **GPS fix and BLE range under the seat pan** — verify before committing the enclosure position.

Resolved by research:

- ~~SWD probe required~~ — onboard SAMD11 CMSIS-DAP handles flash, debug, and logging over USB-C.
- ~~Which bike, clutch switch~~ — CB350 H'ness; a switch exists, but an analog hall sensor is better
  data and keeps the build off the harness.
- ~~Power source~~ — self-contained LiPo, decided.
- ~~How to get logs off without opening the box~~ — `fs_mgmt` over BLE, plus the summary file.
- ~~Can TF-M and dual-slot FOTA coexist~~ — **yes**, NCS supports `NRF_NS_SECONDARY` with a combined
  S+NS image in the secondary slot (§7.3).

---

## 17. Shopping list

**BLE, FOTA, and TF-M add no parts** — the radio and TrustZone are on-chip.

| Item | Cost | Required? |
|---|---|---|
| GPS module (u-blox NEO-M8N, UART, ceramic patch antenna) | ~$12 | Yes — no speed without it |
| microSD breakout (SPI) + card | ~$5 | Yes |
| 2× linear hall sensor (e.g. A1324) + small magnets | ~$4 | Clutch and front brake levers |
| **1000 mAh** LiPo, 3.7 V, with BAT-pad leads | ~$6 | Yes — see §10 for why not 2000 |
| Plastic IP65 enclosure + subframe strap/bracket | ~$10 | Yes — metal blocks GPS and BLE |
| IP67 panel-mount momentary button | ~$2 | Fallback session control + service mode |
| ~~SWD debug probe~~ | — | **Not needed** — onboard CMSIS-DAP |
