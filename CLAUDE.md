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
uses it, purely for this reason: `auxModeFor()` (defined with the secondary-stick block) is
the first function in the file, so prototypes for `gotoState(LinkState)` and
`isWifiDomain(LinkState)` land above it. **Moving the enum back down next to its state
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

`ENABLE_BLE 0` builds Wi-Fi-only with no ArduinoBLE dependency (27% flash vs 39%). BLE
needs an up-to-date ESP32-S3 modem firmware on the R4.

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

### Pin map is constrained, not arbitrary

`analogWrite()` is only real PWM on **D3 D5 D6 D9 D10 D11**; on any other pin the core
degrades it to a 0/1 digital write, so the four `EN` pins must come from that set. EN takes
D3/D5/D6/D9, leaving D10/D11 for SPI. Avoided on purpose: D0/D1 (Serial1), D13 (built-in
LED), and **A4/A5 (I2C)** — the last so quali_base's INA219 pack monitor can be added
without moving a motor pin. Direction pins take the non-PWM digitals D2/D4/D7/D8/D12 plus
A0/A1/A2 as plain digital outputs.

### `MOTORS[]` order is geometry; `inv` and `VX_SIGN` are bench facts

Array order is `{FL, RL, FR, RR}` and must stay that way — `drive4()` indexes it
positionally and the mix assigns a different expression to each corner. Reordering silently
turns a strafe into a spin.

**The shipped `inv` flags and `VX_SIGN` are a starting guess, not a measurement** — this
repo has never been on hardware. Bring-up order matters and cannot be shortcut (see README):
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
- **Secondary stick drives the mode that is NOT selected** (`auxModeFor()`), so the two
  sticks span all three DOFs at once. Derived from `driveMode` rather than selectable, so
  the mode dots still say what the second stick does. `NESSO_BTN_STICK2` is deliberately
  unbound. Contributions **sum then clamp**. `hasAux` is honoured rather than inferred from
  a non-zero reading — the encoder zeroes the aux fields, so `0/0` cannot distinguish "no
  stick" from "stick centred". Note the N1 **may already set `hasAux`** if a Mini JoyC is
  fitted, so this can change how an existing handheld drives.

### What was deliberately NOT ported from quali_base

- **Vision / `MODE_AUTO` / the host ARM chain** — no camera, no MPU. With nothing left to
  bind to the double click, mode selection also drops the 500 ms commit window; clicks
  commit on the release. Do not re-add a gesture window without a gesture to disambiguate.
- **The INA219 battery monitor and its three matrix screens** (bottom-row gauge, low-battery
  corner blink, full-screen critical battery). It is indicator-only in quali_base — it takes
  no action — so it is not drive logic. A4/A5 are reserved for it.

### Matrix — 12x8 landscape, drawn directly

quali_base's panel is 13x8 read *rotated*, so its `px()` maps a logical 8x13 portrait frame.
The R4's is read landscape, so `fb[row][col]` is direct and no rotation helper exists here.
Indicators that did port keep their semantics: the proportional plus-shaped stick dot
(forward = up), the **throttle lock dropping the dot's vertical arms into a horizontal bar**
(the indicator describing itself — the arms it drops are the axis it gave up, and it cannot
be confused with a merely centred throttle), mode dots along the top-right edge, the
top-left link lamp, the 3x3 wheel-test quadrant, and the boot self-test.

R4-only: `drawSearch()` and `drawNoSignal()`, which exist because the R4 has link states
quali_base's MCU never had. The top-left+1 cell that is quali_base's *arm lamp* is reused
here to say **which transport** is paired (col 0 = Wi-Fi, col 1 = BLE).

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

## Working on this repo

- `arduino_secrets.h` is **gitignored**; `arduino_secrets.h.example` is the tracked template.
- `build/` is gitignored. Compile with the README's `arduino-cli` invocation — it needs
  `--library "D:/packages/arduino/user/libraries/NessoLink"` (≥ 1.1.2).
- Verify **both** build paths after touching the transport code: default, and
  `ENABLE_BLE 0`. The Wi-Fi-only branch has its own re-connect loop in `loop()` that the
  BLE build never executes.
- Nothing here has run on hardware yet. Claims about motion, pairing behaviour and the
  matrix are from code and compilation only — say so rather than implying otherwise.
