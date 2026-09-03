# Engineering notes — r4_mecanum_base

Engineering notes for r4_mecanum_base that go deeper than README.md.

---

## Code structure traps

### The `.ino` prototype-injection trap — why `enum LinkState` sits at the top

The Arduino preprocessor injects auto-generated prototypes for every function in a `.ino`
**immediately before the first function definition in the file**. Any type used in a
signature must therefore be declared *above that point*.

`enum LinkState` is declared up in the link-config block, far from the state machine that
uses it, purely for this reason: `searchWindowMs()` (with the link-state variables) is
the first function in the file, so prototypes for `gotoState(LinkState)` and
`isWifiDomain(LinkState)` land above it. (It used to be `auxModeFor()`; deleting a
function can MOVE the injection point, which is part of why both build paths get verified
after structural edits.) **Moving the enum back down next to its state
machine breaks the build** with a confusing `'LinkState' was not declared in this scope` at
the *transport* functions, hundreds of lines from the real cause. The same trap fires for
any new type used in a signature — declare it above the first function definition, or move
the function.

### `MOTORS[]` array order is positional

Array order is `{FL, RL, FR, RR}` and must stay that way — `drive4()` indexes it
positionally and the mix assigns a different expression to each corner. Reordering silently
turns a strafe into a spin.

---

## The drive loop owns the radio — there is no Bridge thread

In the Uno Q build the MPU owned the link and pushed frames to the MCU over the Router Bridge,
on its own thread. Here `loop()` does everything, which changes three things:

- **No `delay()` in `loop()`.** The N1 transmits at ~10 Hz; a blocking wait long enough to
  matter drops UDP datagrams and starves the BLE stack. The Uno Q build's `delay(60)` became a
  60 ms **tick** (`DRIVE_TICK_MS`), so `SLEW_STEP` still means "max PWM change per 60 ms"
  while the radio is serviced every pass.
- **No `volatile` on the shared drive state.** `acceptFrame()` runs on the same thread as
  the mix. The Uno Q build needs `volatile` because its Bridge providers do not.
- **Motors must be cut before any transport swap.** `enterWifi()` blocks for up to nine
  seconds and nothing services the drive tick while it does, so the last PWM values would
  stand for the whole window. `gotoState()` calls `stopMotors()` **first**, which also
  zeroes `curPwm[]` so the slew limiter does not ramp down from a stale position. A ramp is
  a comfort feature for a rover under command; a rover whose commands have stopped arriving
  should not be ramping anywhere.

---

## Mix internals

### Deadband, stall snap, and why they sit on opposite sides of the mix

- `axisDead()` runs on the **input axes**: rescales `[DEADBAND..255] → [0..255]`.
- `stallComp()` runs on each **mixed wheel value**: rescales `[1..255] → [MIN_PWM..MAX_PWM]`.

Snapping to `MIN_PWM` *before* mixing injects a 60-unit floor into every axis, so a pure
forward command leaks a phantom strafe/rotate term and the rover crabs instead of tracking
straight. Do not collapse these into one function.

`REMOTE_DEADBAND = 25` applies to **both sticks** (aux is the same ±255 scale). It is now
belt-and-braces — 1.1.2 makes "a released stick reads exactly 0" contractual, and
the Uno Q build's wire log confirms the N1 dead-zones before transmitting. It stays as the only
cover against an out-of-date handheld reintroducing the "sometimes it won't stop" runaway,
where a resting stick at ±11..14 counted as a command and the stall snap lifted it to
~65 PWM. The error is asymmetric: too wide only trims centre travel, too narrow lets the
rover creep away. **A non-zero rest reading now means a stale transmitter, not jitter.**

### Wheel overflow is rescaled together, never clamped per wheel

Clamping each wheel independently distorts the ratio between them, which *is* the commanded
direction — the rover curves away whenever a combined command saturates. `driveTick()`
scales all four down against the max instead. `clampAxis()` on the summed axes is what keeps
two sticks pushing the same axis from handing that ratio to the overflow rescale.

### `clampAxis()` on the raw aux is a malformed-frame guard, not a rescale

The field is int16, so a bad frame could inject ±32767, which the wheel-overflow rescale
would treat as a legitimate full-scale command and divide every other axis down against.

### Why the aux axis convention belongs to the protocol

That contract exists because receivers guessed wrong first. Before NessoLink 1.1.2 the
header specified no aux range and no axis convention, and the N1 was in fact transmitting the
axes **transposed** (stick up → rover strafed right) at **±515** rather than ±255. Its own
on-screen joystick disc rendered from the same transposed pair, so **the handheld looked
correct while the wire was swapped**. Both faults were fixed in the *transmitter*.

This repo's first implementation had the same two bugs (it read `auxY` as the lateral axis
on a ±515 scale) and they were removed rather than re-tuned. A receiver cannot know how a
handheld's modules are mounted: compensating locally leaves every other NessoLink receiver
still wrong, and inverts this one the moment the transmitter is fixed. **If an axis is
wrong, fix the transmitter.** Same rule as `VX_SIGN` — a correction belongs where the
fact it corrects lives.

---

## Matrix draw rules

Every draw routine works in a **logical 8 wide x 12 tall portrait frame** and `px(x, y)`
applies the rotation — logical (0,0) is the viewer's top-left, +x right, +y **down**.
Nothing above `fbRender()` may touch `fb[][]` directly. This matches the Uno Q build (whose
logical frame is 8x13), so an indicator ported from there keeps its geometry and only `H`
changes.

The frame is **banded, and the bands never overlap**:

| band | contents |
|---|---|
| `y = 0` | link lamp (`x=0` Wi-Fi, `x=1` BLE) + mode dots (1..`MODE_COUNT` from `x=W-1` leftwards) |
| `y = 2..H-3` | the stick dot; its plus spans `y = 1..H-2`, so it can never reach either strip |
| `y = H-1` | battery gauge (1..8 px) + the low-battery blink at `x=W-1` |

Ported semantics kept: the throttle lock **drops the dot's vertical arms into a horizontal
bar** (the indicator describing itself — the arms it drops are the axis it gave up, and it
cannot be confused with a merely centred throttle), and `showBattery()` takes the whole
panel on a latched-critical pack.

R4-only: `drawSearch()` and `drawNoSignal()`, which exist because the R4 has link states
the Uno Q build's MCU never had. The `x=1` cell that is the Uno Q build's *arm lamp* says
**which transport** is paired here instead.

**Display ownership is split three ways** and the precedence is deliberate:
`showWheelTest` > `showBattery` (critical) > `showDir` / `drawNoSignal` in `driveTick()`,
with `drawSearch()` drawn by `loop()` — which is why `loop()` must skip it when
`MODE_WHEELTEST` or `battCrit` already claimed the panel. Two routines drawing per pass
defeat `fbRender()`'s identical-frame skip and flicker the display.

`fbRender()` skips identical frames. Anything that writes the panel outside it — the boot
self-test does — must set `haveLast = false`, or the next draw is suppressed.

`driveTick()` draws when paired or wheel-testing; `loop()` draws the search sweep, because
only it knows which transport is being probed. Drawing the X in both places has them
fighting every pass, which defeats the identical-frame skip and flickers the panel.

---

## Gestures resolve on edges, and a link drop abandons a press

The N1 repeats its button state at ~10 Hz, so level-triggering would race through every mode
while the button is held. A click resolves on the **release** (while the button is down
there is no way to know whether it will become a hold); a long press fires **while still
down**, deliberately, so the matrix changes under your thumb and the hold confirms itself.

`updateMode()` returns early on a stale link and clears `lastDown`, so a drop **abandons**
the press in progress rather than letting it fall through as a release — otherwise the rover
would come back from every hiccup in a different mode. A gesture is only a gesture if both
of its edges were observed.

Click priority is `lock → off-cycle mode → cycle`, so one click is always exactly one step
back toward plain `DRIVE`. The lock can only be set in the two cycled modes and any click
clears it first, which is what structurally prevents it surviving a mode change.

---

## Hardware notes (bench, 2026-08-06)

### Sample `millis()` AFTER the transport poll, never before it

`loop()` must not hoist `uint32_t now = millis()` above the poll switch. `acceptFrame()`
runs inside the poll and stamps `lastFrameMs = millis()`, so a `now` taken earlier can be
*less* than `lastFrameMs`; these are `uint32_t`, so `now - lastFrameMs` underflows to
~4.29e9, which reads as "no frame for 49 days" and trips the `UNPAIR_MS` branch **on the
very frame that just paired the link**.

Observed as the R4 pair/unpairing at the N1's own 10 Hz frame rate — **436 cycles in 45 s**,
never staying paired long enough to drive. It presented as a flaky BLE link and was nothing
of the kind. It persisted because `SEARCH_BLE ↔ PAIRED_BLE` does not cross the Wi-Fi/BLE
domain, so `gotoState()` never tore the radio down and the next frame simply re-paired. The
same underflow reaches `stateEnteredMs`, which `acceptFrame()` also stamps — one correctly
placed timestamp covers both. Upstream `Nesso_R4_Receiver` does not have this bug; it calls
`millis()` fresh at the point of use, and the hazard was introduced here by restructuring
`loop()` around the drive tick.

### The ESP32-S3 modem survives an RA4M1 reset — power-cycle to clear it

Reflashing the sketch, or pressing the reset button, restarts **only the main MCU**. The
modem keeps whatever state the *previous* firmware left it in. After the pair/unpair storm
above (which left the modem thrashing a BLE connection at 10 Hz), `BLE.begin()` failed on
every subsequent boot — including boots of corrected firmware. **Unplugging USB and
replugging cleared it immediately**, and BLE has worked since.

**This recurs on every reflash taken while BLE was live**, which during development is
most of them. Budget for it: after flashing, unplug and replug USB if you need BLE. Wi-Fi
is unaffected throughout, so development can continue over UDP/TCP without power cycling.

**`BLE.end()` + `BLE.begin()` does NOT recover it** — tried on hardware 2026-08-06, fails
identically, and was removed rather than left in because it recovered nothing while
doubling the time each dead BLE window costs. Don't re-add that retry without testing it
against a genuinely wedged modem.

So: `BLE.begin() failed` is not automatically an out-of-date modem firmware. Check a full
power cycle *before* reaching for the firmware updater. `enterBle()` records success in
`bleBegun`, and `loop()` bails out of a BLE window whose `begin()` failed rather than
burning a full `SEARCH_SHORT_MS` dwell on a stack that was never started — a receiver that
cannot do BLE should spend its time on Wi-Fi, which still works.

Modem firmware here is **0.6.0**, which is exactly `WIFI_FIRMWARE_LATEST_VERSION` for core
`renesas_uno` 1.6.0. There is nothing to update to; don't flash it speculatively.
`arduino-cli` 1.5.1 has no `firmware` command anyway — it moved to the separate
`arduino-fwuploader`, which is not installed.

### Reading serial from the R4 needs DTR asserted

The R4's Serial is USB CDC. A serial client that opens the port without asserting **DTR**
receives nothing at all and looks exactly like a dead board. From PowerShell:

```powershell
$p = New-Object System.IO.Ports.SerialPort COM5,115200,None,8,one
$p.DtrEnable = $true; $p.RtsEnable = $true; $p.ReadTimeout = 1000
```

Asserting DTR also **resets the board**, so the boot banner is only captured if the client
is already attached. Keep all `Serial.print` strings **pure ASCII** — em-dashes go out as
multi-byte UTF-8 and render as `???` in clients that read the stream as ASCII.

### What the bench has and has not confirmed

Confirmed: BLE scan → connect → subscribe → pair, sustained (20 s, no unpair); frame decode;
the tank→(throttle, turn) unmix; the mecanum mix for forward, reverse and rotate; the slew
limiter stepping in `SLEW_STEP` units; the aux flag arriving set (**a Mini JoyC is fitted**,
exactly the case the docs warn about) and its axes resting at exactly 0, as NessoLink 1.1.2
requires.

Since confirmed on the rover: all four motors; the wheel-test corner mapping; the `inv`
directions; `VX_SIGN`; forward, reverse, rotate and strafe; mode changes; the LED matrix
indicators; and the battery gauge on pack power.

Wi-Fi **UDP** is confirmed too, twice over. First on the bench with the NessoLink
`Nesso_R4_Receiver` example's `tools/nesso_tx.py --pattern idle` — centred sticks, so it
exercises decode and pairing without commanding any motion, which is the right tool once the
motors are wired: it paired as `>>> PAIRED via UDP`, held the link for the whole stream and
dropped cleanly only when the stream stopped. Then with the real N1, including **switching
live between BLE and Wi-Fi UDP in both directions with no power cycle** — the case the three
fixes above exist for.

Not confirmed: TCP framing end-to-end (`ENABLE_TCP` is off by design), and the throttle lock
and aux-stick strafe as distinct gestures.

**`[batt] no sensor` on USB is expected, not a fault** — the UPS is switched off whenever
USB is connected, so the INA219 is unpowered. `pollBattery()` re-probes every
`BATT_RETRY_MS` (5 s) precisely so the module can be absent at boot and join later, which
means a UPS-powered run needs no reset to bring the gauge up. Don't chase it as a wiring
fault on a bench run; check it on pack power, where the matrix gauge row is the readout
since there is no serial.

---

## Building

Verify **both** build paths after touching the transport code: default, and `ENABLE_BLE 0`.
The Wi-Fi-only branch has its own re-connect loop in `loop()` that the BLE build never
executes.
