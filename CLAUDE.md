# CLAUDE.md — r4_mecanum_base

Arduino **Uno R4 WiFi** mecanum rover, driven from a **Nesso N1** handheld over NessoLink
RemoteFrames on **Wi-Fi (UDP + TCP) or BLE**, auto-pairing with whichever remote speaks
first. User-facing docs are in [README.md](README.md).

The drive logic is a port of **[quali_base](../quali_base)** — the same chassis on an
Arduino Uno Q with a Linux MPU. When drive behaviour changes there, it should change here;
`quali_base/CLAUDE.md` carries the original reasoning at more length, and this file records
only what is different or newly true on the R4.

Single sketch, no sub-projects. Build/flash commands are in the README.

---

## Architecture decisions & traps

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

### This sketch owns the radio — there is no Bridge thread any more

In quali_base the MPU owned the link and pushed frames to the MCU over the Router Bridge,
on its own thread. Here `loop()` does everything, which changes three things:

- **No `delay()` in `loop()`.** The N1 transmits at ~10 Hz; a blocking wait long enough to
  matter drops UDP datagrams and starves the BLE stack. quali_base's `delay(60)` became a
  60 ms **tick** (`DRIVE_TICK_MS`), so `SLEW_STEP` still means "max PWM change per 60 ms"
  while the radio is serviced every pass.
- **No `volatile` on the shared drive state.** `acceptFrame()` runs on the same thread as
  the mix. quali_base needs `volatile` because its Bridge providers do not.
- **Motors must be cut before any transport swap.** `enterWifi()` blocks for up to nine
  seconds and nothing services the drive tick while it does, so the last PWM values would
  stand for the whole window. `gotoState()` calls `stopMotors()` **first**, which also
  zeroes `curPwm[]` so the slew limiter does not ramp down from a stale position. A ramp is
  a comfort feature for a rover under command; a rover whose commands have stopped arriving
  should not be ramping anywhere.

### Wi-Fi and BLE cannot run at once — the search alternates, and the preference is sticky

The R4's ESP32-S3 shares one antenna. The link state machine alternates a Wi-Fi search
window and a BLE search window until a valid frame arrives; the first good frame binds the
link (source IP for Wi-Fi, the connected peripheral for BLE).

The preference is **sticky** — `acceptFrame()` sets `preferBle` to whatever just worked —
and the preferred transport gets `SEARCH_LONG_MS` while the other gets only a brief
`SEARCH_SHORT_MS` probe. This is not tidiness: **the R4's BLE-central connect momentarily
disrupts the N1's own Wi-Fi**, because the N1's C6 also shares one radio. A long Wi-Fi
dwell lets a UDP remote pair before any BLE probe, and once paired the R4 stops probing
entirely. Shortening `SEARCH_LONG_MS` or making the probes symmetric reintroduces a
receiver that fights the transmitter it is trying to hear.

`ENABLE_BLE 0` builds Wi-Fi-only with no ArduinoBLE dependency. BLE needs a working
ESP32-S3 modem firmware on the R4 — see the power-cycle note below before suspecting it.

### NEVER call `WiFiServer::available()` from the drive loop — it blocks ~10 s

WiFiS3's `WiFiServer::available()` is a **synchronous modem round-trip** that performs the
`accept` on the ESP32-S3. With no client pending it waits out the modem's timeout —
**measured at ~10.1 s on this board, on every loop pass**. That is 14x `FAILSAFE_MS`.

The symptom is not "TCP is slow", it is "Wi-Fi is unusable": the drive loop stops for 10 s,
the link goes stale, the rover stops, unpairs, re-pairs on the next buffered datagram, and
repeats 10 s later. It was found in one run by the stall detector after being mistakenly
blamed on the INA219 probe — which the same instrumentation cleared at 100 ms.

`ENABLE_TCP` is therefore **0** by default. `TcpParser` is kept and is still correct; it is
the *polling* that is unaffordable. If TCP is ever wanted, poll it on a slow timer or off
the drive loop — never per pass.

### The stall detector is load-bearing, not debug scaffolding

Any loop pass slower than `STALL_WARN_MS` logs `[stall] loop blocked N ms`. Anything over
`FAILSAFE_MS` silently drops the link, and this class of bug is otherwise diagnosed only by
guesswork — the TCP stall above was blamed on the battery probe until this measured both.
`gotoState()` clears `stallRefMs` so transport hand-offs, which legitimately block for
seconds *and stop the motors first*, are not reported; if everything is reported it becomes
noise nobody reads. Keep it.

### Bind sockets idempotently; "associated" != "listening"

`enterWifi()` used to bind the sockets once, inside its 9 s association wait. If the
association had not landed by then the sockets were never opened at all, and a window that
associated a moment later sat fully connected and **deaf** for its entire dwell, with
nothing retrying. Coming off `BLE.end()` the modem can easily need longer than 9 s.
`ensureWifiListening()` is now idempotent and called from `pollWifiFrames()` every pass.

### The sticky search preference must DECAY

`preferBle` biases the search toward whatever paired last — which is right for a brief
dropout on the same transport, and wrong for the most common case, the operator moving the
handheld to the **other** transport. Held indefinitely it gave the departed transport 15 s
windows and the live one 4 s, and that 4 s had to cover a slow reconnect before any
listening began; switching the N1 from BLE to UDP then never reconnected until the R4 was
power-cycled (which is exactly what "fixed" it — reset cleared `preferBle`). After
`SEARCH_FAIR_AFTER` fruitless windows the preference stops applying and both transports get
the full dwell.

### The aux axis convention belongs to the PROTOCOL — never compensate here

NessoLink **1.1.2** specifies it normatively: every axis in the frame, motors and aux
alike, is **−255..255**; `auxX` is screen-horizontal with **+ = right**; `auxY` is
screen-vertical with **+ = up**; a released stick reads **exactly 0**. "Screen-relative"
means the *transmitter* has already normalised for its display orientation and for how each
stick module is mounted. This end reads right/up and does nothing else — no sign flip, no
swap, no rescale, and **deliberately no `AUX_X_SIGN`/`AUX_Y_SIGN` knob**.

That contract exists because receivers guessed wrong first. Before 1.1.2 the header
specified no aux range and no axis convention, and the N1 was in fact transmitting the axes
**transposed** (stick up → rover strafed right) at **±515** rather than ±255. Its own
on-screen joystick disc rendered from the same transposed pair, so **the handheld looked
correct while the wire was swapped**. Both faults were fixed in the *transmitter*.

This repo's first implementation had the same two bugs (it read `auxY` as the lateral axis
on a ±515 scale) and they were removed rather than re-tuned. A receiver cannot know how a
handheld's modules are mounted: compensating locally leaves every other NessoLink receiver
still wrong, and inverts this one the moment the transmitter is fixed. **If an axis is
wrong, fix the transmitter.** Same rule as `VX_SIGN` below — a correction belongs where the
fact it corrects lives.

`clampAxis()` on the raw aux is a **malformed-frame guard, not a rescale**: the field is
int16, so a bad frame could inject ±32767, which the wheel-overflow rescale would treat as
a legitimate full-scale command and divide every other axis down against.

### Deadband, stall snap, and why they sit on opposite sides of the mix

- `axisDead()` runs on the **input axes**: rescales `[DEADBAND..255] → [0..255]`.
- `stallComp()` runs on each **mixed wheel value**: rescales `[1..255] → [MIN_PWM..MAX_PWM]`.

Snapping to `MIN_PWM` *before* mixing injects a 60-unit floor into every axis, so a pure
forward command leaks a phantom strafe/rotate term and the rover crabs instead of tracking
straight. Do not collapse these into one function.

`REMOTE_DEADBAND = 25` applies to **both sticks** (aux is the same ±255 scale). It is now
belt-and-braces — 1.1.2 makes "a released stick reads exactly 0" contractual, and
quali_base's wire log confirms the N1 dead-zones before transmitting. It stays as the only
cover against an out-of-date handheld reintroducing the "sometimes it won't stop" runaway,
where a resting stick at ±11..14 counted as a command and the stall snap lifted it to
~65 PWM. The error is asymmetric: too wide only trims centre travel, too narrow lets the
rover creep away. **A non-zero rest reading now means a stale transmitter, not jitter.**

### Wheel overflow is rescaled together, never clamped per wheel

Clamping each wheel independently distorts the ratio between them, which *is* the commanded
direction — the rover curves away whenever a combined command saturates. `driveTick()`
scales all four down against the max instead. `clampAxis()` on the summed axes is what keeps
two sticks pushing the same axis from handing that ratio to the overflow rescale.

### Pin map is constrained, and is deliberately quali_base's

`analogWrite()` is only real PWM on **D3 D5 D6 D9 D10 D11** — confirmed in the `UNOWIFIR4`
variant's `initVariant()`, not assumed. On any other pin the core **silently degrades it to
a 0/1 digital write**, which would give exactly two speeds, stopped and flat out, so the
four `EN` pins must come from that set.

| wheel | module, channel | IN a | IN b | EN |
|---|---|---|---|---|
| front-left | #2 front, **B** | D12 | D13 | **~D10** |
| rear-left | #1 rear, **A** | D2 | D3 | **~D5** |
| front-right | #2 front, **A** | D8 | D11 | **~D9** |
| rear-right | #1 rear, **B** | D4 | D7 | **~D6** |

Identical to quali_base's table on purpose: the same chassis and motor harness move between
the Uno Q and the R4 with nothing re-terminated, and the two `MOTORS[]` tables read against
each other line for line. The two modules have **opposite channel conventions** (rear
`A = left, B = right`; front `A = right, B = left`), which is why the front pair looks
transposed against the rear — that is the wiring, not a slip.

D0/D1 (Serial1) and **A4/A5** are untouched; A4/A5 carry the INA219. D10–D13 are the SPI
bus, used here as plain GPIO, which costs nothing because nothing on this board needs SPI.

### Full stick IS 100% duty — and the 5 V is not the motor supply

`analogWrite(255)` on the R4 reaches a genuine **100% duty cycle**, verified in the core,
not assumed: it becomes `pulse_perc(100.0)` → `set_duty_cycle(period)`, i.e. compare equal
to period, so there is no low phase. `MAX_PWM = 255` therefore leaves **no headroom** —
there is nothing above it to raise. Default PWM frequency is **490 Hz**
(`STANDARD_PWM_FREQ_HZ`, `FspTimer.h`), fine for an L298N.

Don't confuse the two voltages. EN is a **logic enable** at 5 V (the R4 is a 5 V board);
motor current comes from the L298N's `Vs` rail — the 12.6 V pack — never from the Arduino.
The classic L298N is a BJT H-bridge with two saturated transistors in series per path, so
it drops **~2 V** largely independent of load, worsening with current and leaving as heat:
each motor sees roughly 10.5 V at full throttle off a 12.6 V pack. Firmware cannot recover
that. If more power is ever needed the levers are hardware only — a higher pack voltage, or
a MOSFET driver (TB6612FNG, DRV8871, BTS7960).

`MIN_PWM = 60` was **validated on the rover under real load** — it moves easily on minimal
commands. Too low and a wheel buzzes without turning; too high and control near centre goes
coarse. Don't retune either constant without a loaded bench run.

### `MOTORS[]` order is geometry; `inv` and `VX_SIGN` are bench facts

**Calibrated 2026-08-06 — `MOTORS[]` order and `inv` are now MEASURED.** The wheel test
drove the corner the matrix lit in every slot (order confirmed), and only front-right ran
backwards, giving `inv = {true, true, false, false}`: the whole left side inverted, the
whole right side not. That symmetry is the expected result for a chassis whose sides are
mirrored by mounting with every motor conventionally terminated, so it corroborates itself.
**`VX_SIGN = +1`, re-measured 2026-08-11** after the mecanum wheels were found mounted in
the wrong corners and remounted correctly: strafe then came out inverted, so the sign
flipped from the `-1` measured on 2026-08-06. That earlier value — and its "this chassis's
rollers are mirrored against the standard X layout" story — was an artifact of the
mis-mounted wheels, not a chassis fact. Only strafe flipped when the wheels moved, which is
the signature of a roller-pattern change: forward is invariant to roller orientation and
rotation needed no change either time. **quali_base's `-1` is now suspect for the same
reason** — same chassis, same wheels — and should be re-verified on hardware before being
trusted. Note that remounting wheels between corners changes ONLY `VX_SIGN`: `MOTORS[]`
order and `inv` describe the motors and wiring, which did not move.

The pin map is byte-for-byte quali_base's, deliberately, so the same chassis and harness
move between the Uno Q and the R4 with nothing re-terminated. The one table divergence is
front-right's `inv`: quali_base needs it inverted because *its* front-right leads are
terminated backwards on OUT3/OUT4, and that cable was re-terminated on this build. `inv`
belongs to the MOTOR, not the slot, so it correctly does not travel with the pin map.

**`D13` IS the built-in LED on the R4** (`PIN_LED = 13`, P102) — it is not on the Uno Q, and
quali_base's comment says so. D13 is front-left's second direction pin here, so the onboard
LED mirrors that motor's direction bit. Cosmetic, but it is no longer a status light.


Array order is `{FL, RL, FR, RR}` and must stay that way — `drive4()` indexes it
positionally and the mix assigns a different expression to each corner. Reordering silently
turns a strafe into a spin.

Bring-up order matters and cannot be shortcut (see README):
wheel-test the index→corner map *first*, because driving forward is invariant to any
permutation of the four wheels and rotating only distinguishes left from right, so a
front/rear swap within one side passes both of those tests and corrupts nothing but strafe.
Then `inv` on forward motion, then `VX_SIGN` on strafe — never `inv` for a strafe fault,
since it is by then calibrated for forward motion.

### Ported from quali_base 2026-08-06: throttle lock + secondary stick

- **Throttle lock replaces `MODE_ROTATE`.** ROTATE was `vy = 0, w = turn` — DRIVE with the
  throttle forced to zero, i.e. one case of a lock. As a flag orthogonal to the mode,
  `DRIVE + lock` is the old ROTATE bit for bit and `STRAFE + lock` supplies the half that
  never existed. It zeroes `vy` **after both sticks are summed**, so it means "no forward
  motion can be commanded" rather than "the drive stick's throttle is ignored" — otherwise
  the second stick could still drive forward while "locked".
- **Secondary stick: FIXED mapping, modes disabled (reworked 2026-08-11).** With `hasAux`
  set, primary = throttle + rotate and aux = throttle + strafe, **regardless of mode** —
  all three DOFs live at once, every motion (including the axle pivots) available by
  default. Modes are one-stick machinery: `updateMode()` pins the mode to `DRIVE` while
  aux is present and the click cycle is a no-op (the click still clears the throttle lock
  and still exits `WHEELTEST`, which is not pinned away — a bench diagnostic must survive
  pairing). The original port had the aux stick drive the *complement* mode
  (`auxModeFor()`, now deleted): same reachable motions, but the sticks swapped roles with
  the mode, which bought nothing over a fixed mapping. `NESSO_BTN_STICK2` is deliberately
  unbound, and so is **aux stick 2** (`aux2X`/`aux2Y`/`hasAux2`, NessoLink v2 frames) —
  decoded by the library, dropped here, pending a decision on what a third stick should
  mean when all three DOFs are already covered. Contributions **sum then clamp**. `hasAux`
  is honoured rather than inferred from a non-zero reading — the encoder zeroes the aux
  fields, so `0/0` cannot distinguish "no stick" from "stick centred". The N1 in use
  **does set `hasAux`** (a Mini JoyC is fitted, confirmed on the bench).

### What was deliberately NOT ported from quali_base

- **Vision / `MODE_AUTO` / the host ARM chain** — no camera, no MPU. With nothing left to
  bind to the double click, mode selection also drops the 500 ms commit window; clicks
  commit on the release. Do not re-add a gesture window without a gesture to disambiguate.
(The INA219 battery monitor was initially left out for the same reason — it is
indicator-only, so not drive logic — but was ported in full on 2026-08-06 when the panel
gained a dedicated gauge row. It remains indicator-only: it never cuts the motors and never
shuts anything down, so **the pack's BMS is the only automatic protection**, and a flat pack
under load will be dragged to BMS cutoff if the operator ignores the panel. Note its
failsafe points the *opposite* way to the link's: an absent sensor reports state −1 and
leaves the rover driving, because losing a sensor must not immobilise it, whereas losing the
operator must never leave it driving. Do not "make them consistent".)

### Matrix — logical 8x12 PORTRAIT, rotated 90 deg CW on the way out

The panel is physically 12 cols x 8 rows, but the board is read turned, so every draw
routine works in a **logical 8 wide x 12 tall portrait frame** and `px(x, y)` applies the
rotation — logical (0,0) is the viewer's top-left, +x right, +y **down**. Nothing above
`fbRender()` may touch `fb[][]` directly. This matches quali_base (whose logical frame is
8x13), so an indicator ported from there keeps its geometry and only `H` changes.

`MATRIX_ROTATE_CW` selects the direction and is the **only** place the board's mounting
orientation is encoded — same discipline as `VX_SIGN`. If the whole display comes out
inverted, flip that constant; never "fix" an individual draw routine, since they are all
correct relative to one another in logical coordinates.

The frame is **banded, and the bands never overlap**:

| band | contents |
|---|---|
| `y = 0` | link lamp (`x=0` Wi-Fi, `x=1` BLE) + mode dots (1..`MODE_COUNT` from `x=W-1` leftwards) |
| `y = 2..H-3` | the stick dot; its plus spans `y = 1..H-2`, so it can never reach either strip |
| `y = H-1` | battery gauge (1..8 px) + the low-battery blink at `x=W-1` |

**An empty bottom row means "no INA219", never "flat pack"** — the gauge always lights at
least one pixel while the sensor answers. Those are opposite states (a missing sensor
deliberately disables battery management and leaves the rover driving), so they must not
look alike.

Ported semantics kept: the throttle lock **drops the dot's vertical arms into a horizontal
bar** (the indicator describing itself — the arms it drops are the axis it gave up, and it
cannot be confused with a merely centred throttle), and `showBattery()` takes the whole
panel on a latched-critical pack.

R4-only: `drawSearch()` and `drawNoSignal()`, which exist because the R4 has link states
quali_base's MCU never had. The `x=1` cell that is quali_base's *arm lamp* says **which
transport** is paired here instead.

**Display ownership is split three ways** and the precedence is deliberate:
`showWheelTest` > `showBattery` (critical) > `showDir` / `drawNoSignal` in `driveTick()`,
with `drawSearch()` drawn by `loop()` — which is why `loop()` must skip it when
`MODE_WHEELTEST` or `battCrit` already claimed the panel. Two routines drawing per pass
defeat `fbRender()`'s identical-frame skip and flicker the display.

`fbRender()` skips identical frames. Anything that writes the panel outside it — the boot
self-test does — must set `haveLast = false`, or the next draw is suppressed.

**Display ownership is split.** `driveTick()` draws when paired or wheel-testing; `loop()`
draws the search sweep, because only it knows which transport is being probed. Drawing the
X in both places has them fighting every pass, which defeats the identical-frame skip and
flickers the panel.

### Gestures resolve on edges, and a link drop abandons a press

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

Wi-Fi **UDP** is confirmed too, twice over. First on the bench with
`examples/Nesso_R4_Receiver/tools/nesso_tx.py --pattern idle` — centred sticks, so it
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

## Working on this repo

- `arduino_secrets.h` is **gitignored**; `arduino_secrets.h.example` is the tracked template.
- `build/` is gitignored. Compile with the README's `arduino-cli` invocation — it needs
  `--library "D:/packages/arduino/user/libraries/NessoLink"` (≥ 1.1.2).
- Verify **both** build paths after touching the transport code: default, and
  `ENABLE_BLE 0`. The Wi-Fi-only branch has its own re-connect loop in `loop()` that the
  BLE build never executes.
- Nothing here has run on hardware yet. Claims about motion, pairing behaviour and the
  matrix are from code and compilation only — say so rather than implying otherwise.
