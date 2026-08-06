# r4_mecanum_base — Nesso-driven mecanum rover on an Arduino Uno R4 WiFi

A 4-wheel **mecanum** rover driven from a [Nesso N1](../Nesso_base) handheld over
[NessoLink](https://github.com/ugursayar/NessoLink) `RemoteFrame`s, accepting **both
Wi-Fi (UDP + TCP) and Bluetooth LE** and auto-pairing with whichever remote speaks first.

The drive logic is a port of [`quali_base`](../quali_base) — the same chassis running on
an Arduino Uno Q — onto a bare R4 with no companion Linux MPU.

> **Not yet run on hardware.** Everything below is verified by compilation and by reading
> against quali_base; no motion, pairing or matrix behaviour has been confirmed on the
> bench. The `inv` flags and `VX_SIGN` are a starting guess — see **Bring-up**.

Engineering notes and traps are in [CLAUDE.md](CLAUDE.md).

```
 Nesso N1  ──[ BLE | Wi-Fi UDP:8889 | Wi-Fi TCP:8890 ]──▶  Uno R4 WiFi  ──▶  2x L298N ──▶ 4 mecanum wheels
                                                                          └─▶  12x8 LED matrix + Serial
```

## What came across from quali_base, and what didn't

Everything that is *drive logic* is carried over with its behaviour intact:

- the axis dead-zone / stall-compensation split placed on **opposite sides** of the
  mecanum mix (snapping to `MIN_PWM` before mixing leaks a phantom strafe into a pure
  forward command),
- the per-wheel mecanum mixer and the proportional down-scale on overflow (clamping
  wheels independently distorts the commanded direction),
- the slew limiter, the `REMOTE_DEADBAND = 25` that fixed quali_base's "sometimes it
  won't stop" bug, and the stale-link stop,
- the click / long-press mode gestures, the **throttle lock**, the **secondary stick**
  and `MODE_WHEELTEST`.

What changed, and why:

| | quali_base (Uno Q) | here (Uno R4 WiFi) |
|---|---|---|
| **Transport** | Router Bridge from a Linux MPU that owned the radio | the sketch owns the link: Wi-Fi UDP/TCP + BLE, alternating search, sticky pairing |
| **Vision / `MODE_AUTO`** | camera object-chase on the MPU, gated by a host ARM toggle | **removed** — no camera, no host |
| **Double-click gesture** | selects `MODE_AUTO` | **removed** with it, and with it the 500 ms lag every mode change paid waiting to see if a partner click was coming |
| **Battery (INA219)** | polled and displayed, indicator-only | **not ported** — it takes no action, so it is not drive logic. A4/A5 are left free for it |
| **Matrix** | 13x8 read in portrait, so every draw was rotated | 12x8 landscape, drawn directly |
| **Loop timing** | `delay(60)` — frames arrived on a separate Bridge thread | 60 ms *tick*, no blocking delay: the radio is polled every pass or datagrams drop |

## Full holonomy from a two-stick N1

Mecanum is 3-DOF (forward, strafe, rotate) but the N1's drive stick has two axes, which
it pre-mixes into tank `leftMotor`/`rightMotor` — so a one-stick remote cannot reach all
three at once, which is the only reason modes exist. When the N1 reports a **second
stick** (frame flag `hasAux`) it drives whichever everyday mode is **not** selected, so
the two sticks together span all three DOFs at once:

| selected mode | primary stick (seesaw) | secondary stick (aux 1) |
|---|---|---|
| `DRIVE` (vy, w) | throttle + rotate | throttle + strafe → adds `vx` |
| `STRAFE` (vy, vx) | throttle + strafe | throttle + rotate → adds `w` |
| `WHEELTEST` | ignored | ignored |

The complement is *derived* from the mode rather than being selectable, so there is still
one mode to think about and no new gesture: the mode dots already say what the second
stick does. `NESSO_BTN_STICK2` is deliberately unbound. The two sticks' contributions
**sum then clamp**, so the shared throttle axis adds where they agree and cancels where
they fight. A one-stick remote is unaffected — `hasAux` is honoured rather than inferred
from a non-zero reading, since the encoder zeroes the aux fields and `0/0` cannot
distinguish "no stick" from "stick centred".

**Axis convention is the protocol's, not a local guess.** NessoLink **1.1.2** states it
normatively: every axis is −255..255, `auxX` is screen-horizontal with **+ = right**,
`auxY` screen-vertical with **+ = up**, and a released stick reads exactly 0. The
transmitter has already normalised for its display orientation and module mounting, so
this end reads right/up and does nothing else — no sign flip, no swap, no rescale, and
deliberately **no `AUX_X_SIGN`/`AUX_Y_SIGN` knob**. If an axis is wrong, fix the
transmitter: a receiver cannot know how a handheld's modules are mounted, so compensating
here would leave every other NessoLink receiver still wrong and would invert this one the
moment the transmitter was fixed. (That contract exists because quali_base guessed first
and got both the transposition and the ±515 scale wrong.)

Note the N1 **may already be setting `hasAux`** if a Mini JoyC is fitted — v1 frames have
always had an aux slot — so this can change how an existing handheld drives.

## Throttle lock

A long press sets the **throttle lock**: the forward/back axis is forced to zero, leaving
the mode's other axis live. It is orthogonal to the mode rather than being one — hold in
`DRIVE` and you can only rotate (this *is* quali_base's old `MODE_ROTATE`, bit for bit);
hold in `STRAFE` and you can only slide sideways, which had no way to be expressed before.

The lock applies to the **sum of both sticks**, after they are gathered, so "locked" means
no forward motion can be commanded from anywhere — a second stick that could still drive
forward would make it mean "locked, unless you use the other stick", which is not a lock.
It does not survive a mode change, and a click always clears it in one step.

## Wiring

Two L298N modules, one per axle, so the high-current leads stay short.

| corner | IN1 | IN2 | EN (PWM) | module |
|---|---|---|---|---|
| front-left  | D12 | A0 | **D6** | #2 (front) A |
| rear-left   | D2  | D4 | **D3** | #1 (rear) A |
| front-right | A1  | A2 | **D9** | #2 (front) B |
| rear-right  | D7  | D8 | **D5** | #1 (rear) B |

`analogWrite()` is only real PWM on **D3 D5 D6 D9 D10 D11** on this board — on any other
pin the core degrades it to a 0/1 digital write, so the four EN pins must come from that
set. D0/D1 (Serial1), D13 (built-in LED) and **A4/A5 (I2C)** are deliberately left alone;
A4/A5 in particular stay free so quali_base's INA219 pack monitor can be added later
without moving a motor pin. D10/D11 stay free for SPI.

## Bring-up — in this order

The `inv` flags and `VX_SIGN` in the sketch are a **starting guess, not a measurement**.

1. Set `driveMode = MODE_WHEELTEST`, reflash. Each wheel spins for 2 s in turn while the
   matrix lights the quadrant the code *believes* that wheel sits in. If the lit corner
   and the spinning wheel disagree, **reorder `MOTORS[]`** — never compensate in the mix.
   Driving forward is invariant to any permutation of the four wheels and rotating only
   distinguishes left from right, so a front/rear swap within one side passes both of
   those tests and corrupts nothing but strafe. This is the only test that catches it.
2. Back in `MODE_DRIVE`, push the stick forward. Flip `inv` on every wheel that turns
   backwards. The rover now tracks straight. (Four wheels turning the same *absolute*
   direction is the wrong state — it drives one side backwards.)
3. Only now check strafe. If it slides the wrong way, flip **`VX_SIGN`** — not `inv`,
   which is by then calibrated for forward motion and would break driving straight.

## Controls

| gesture on the stick button | effect |
|---|---|
| single click | clear the throttle lock if set — that is *all* it does; otherwise cycle `DRIVE` ↔ `STRAFE`, or from an off-cycle mode back to `DRIVE` |
| long press (800 ms) | set the **throttle lock** on the current mode (idempotent, not a toggle) |
| reflash only | `MODE_WHEELTEST` |

One click is always exactly one step back toward plain `DRIVE`. `MODE_WHEELTEST` gets no
gesture on purpose: it ignores both sticks and spins wheels on a timer, so landing on it
mid-drive means the rover moves off with the sticks centred.

Matrix cues: sweeping bar = searching (top-left block = Wi-Fi phase, top-right = BLE);
plus-shaped dot = the virtual stick, with `col 0` lit for a Wi-Fi link / `col 1` for BLE
and 1–3 mode dots on the right edge; full **X** = paired but no signal; 3x3 block =
wheel test. **A locked throttle drops the dot's vertical arms and leaves a horizontal
bar** — the indicator describes itself, the arms it drops being the axis it gave up, and
it cannot be confused with "throttle merely centred" (a plus on the centre row).

## Failsafe

No valid frame for `FAILSAFE_MS` (700 ms) → all axes zero and the slew limiter ramps the
rover to a stop. Lost for `UNPAIR_MS` (3 s) → un-pair and search again. Every transport
transition cuts the motors *first*, skipping the ramp: swapping radios blocks for up to
nine seconds and nothing services the drive tick while it does.

## Configure

| constant | default | notes |
|---|---|---|
| `ENABLE_BLE` | `1` | `0` = Wi-Fi only, no ArduinoBLE dependency or modem-firmware update |
| `SECRET_SSID` / `SECRET_PASS` | `arduino_secrets.h` | must be the N1's 2.4 GHz network |
| `UDP_PORT` / `TCP_PORT` | `8889` / `8890` | must match the N1 firmware's `udp_port` / `tcp_port` |
| `SEARCH_PREFER_BLE` | `0` | cold-boot search preference; thereafter sticky |
| `SEARCH_LONG_MS` / `SEARCH_SHORT_MS` | `15000` / `4000` | dwell on the preferred transport / brief probe of the other |
| `MIN_PWM` / `MAX_PWM` | `60` / `255` | stall floor / ceiling |
| `REMOTE_DEADBAND` | `25` | applies to both sticks. Belt-and-braces since NessoLink 1.1.2 made "a released stick reads exactly 0" part of the contract; kept as the only cover against an out-of-date transmitter. A non-zero rest reading now means a stale handheld build, not jitter |
| `SLEW_STEP` | `40` | max PWM change per 60 ms tick |
| `VX_SIGN` / `MOTORS[].inv` | — | calibrate on the bench, see **Bring-up** |

The R4's ESP32-S3 shares one antenna between Wi-Fi and BLE and **cannot run both at
once**, so the search alternates. The preference is sticky — it follows whichever
transport last paired — because the R4's BLE-central connect momentarily disrupts the
N1's own Wi-Fi (the N1's C6 also shares one radio). If you only ever use one transport,
set `SEARCH_PREFER_BLE` accordingly, or `ENABLE_BLE 0` for Wi-Fi-only.

## Build & flash

```powershell
arduino-cli compile --fqbn "arduino:renesas_uno:unor4wifi" `
  --library "D:/packages/arduino/user/libraries/NessoLink" `
  --build-path "D:/r4_mecanum_base/build" .

arduino-cli upload --fqbn "arduino:renesas_uno:unor4wifi" --port COM5 `
  --build-path "D:/r4_mecanum_base/build" .
```

Footprint: **39% flash / 35% RAM** with BLE, **27% / 29%** Wi-Fi-only.

Requires **NessoLink ≥ 1.1.2** (codec only — its Wi-Fi transport classes are ESP32-only,
so this sketch decodes packets off the Renesas MCU's own stack) and, for BLE,
`ArduinoBLE` plus an up-to-date ESP32-S3 modem firmware on the R4. 1.1.2 matters for the
aux axis contract above; the wire format is unchanged from 1.1.1.
