/*
 * r4_mecanum_base — Arduino Uno R4 WiFi mecanum rover driven by a Nesso N1 handheld.
 * ===========================================================================
 * The drive logic is a port of quali_base (D:/quali_base/sketch/sketch.ino), which
 * runs the same 4-wheel mecanum chassis on an Arduino Uno Q. What changed and why:
 *
 *   * TRANSPORT. quali_base received NessoLink frames over the Router Bridge from a
 *     Linux MPU that owned the radio; there is no MPU here, so this sketch owns the
 *     link itself — Wi-Fi (UDP + TCP) and Bluetooth LE, auto-pairing with whichever
 *     remote speaks first. The dual-link state machine is taken from
 *     D:/examples/Nesso_R4_Receiver.
 *   * NO VISION. quali_base's MODE_AUTO chased objects detected by a camera on the
 *     MPU, gated by a host-side ARM toggle. Neither exists on a bare R4, so MODE_AUTO,
 *     the `vision`/`arm` streams and the arm-expiry failsafe are all gone. With nothing
 *     left to bind to the double-click gesture, mode selection loses its double-click
 *     window too — and with it the ~500 ms commit lag every mode change used to pay.
 *     The secondary stick and the throttle lock are ported in full (quali_base
 *     2026-08-06): a second stick drives whichever everyday mode is NOT selected, so the
 *     two sticks span all three DOFs at once, and a long press locks the throttle axis
 *     instead of selecting a MODE_ROTATE that no longer exists.
 *   * NO BATTERY MONITOR. quali_base polls an INA219 on the UPS_3S pack. That is
 *     indicator-only there (it takes no action), so it is not drive logic and is not
 *     ported. A4/A5 are left free so it can be added later.
 *   * MATRIX. 12x8 landscape here vs the Uno Q's 13x8 read in portrait, so the
 *     indicator is redrawn for this panel rather than rotated.
 *
 * Everything that IS drive logic is carried over unchanged in behaviour: the axis
 * dead-zone / stall-compensation split around the mecanum mix, the per-wheel mixer,
 * the proportional down-scale on overflow, the slew limiter, the click/hold mode
 * gestures and the stale-link stop.
 *
 * BOARD     : Arduino Uno R4 WiFi
 * LIBRARIES : WiFiS3 + Arduino_LED_Matrix (bundled with the R4 core),
 *             NessoLink (github.com/ugursayar/NessoLink),
 *             ArduinoBLE (only when ENABLE_BLE).
 *
 * BLE PREREQUISITES: an up-to-date ESP32-S3 modem firmware on the R4 plus ArduinoBLE.
 * Set ENABLE_BLE 0 for a Wi-Fi-only build with no ArduinoBLE dependency.
 */

#define ENABLE_BLE 1        // 0 = Wi-Fi only (no ArduinoBLE needed)

#include "WiFiS3.h"
#include "Arduino_LED_Matrix.h"
#include <NessoFrame.h>     // portable codec; the library's WiFi transport classes are
                            // ESP32-only, so the Renesas MCU decodes packets itself
#include "arduino_secrets.h"
#include <string.h>
#include <stdlib.h>

#if ENABLE_BLE
#include <ArduinoBLE.h>
#endif

// ── Link config ───────────────────────────────────────────────────────────────
const char*    WIFI_SSID    = SECRET_SSID;
const char*    WIFI_PASS    = SECRET_PASS;
const uint16_t UDP_PORT     = 8889;    // firmware udp_port
const uint16_t TCP_PORT     = 8890;    // firmware tcp_port

// Nesso BLE Nordic-UART service (from the N1 firmware). We subscribe to TX (notify).
const char*    BLE_NAME     = "NESSO";
const char*    BLE_SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const char*    BLE_TX_UUID  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

// Search dwell + failsafe timing. The PREFERRED transport gets the long window, the
// other a brief probe: the R4's BLE-central connect momentarily disrupts the N1's own
// Wi-Fi (the N1's C6 also shares one radio), so while unpaired we keep BLE probes short
// and infrequent. The preference is *sticky* — it follows whichever transport last
// paired us — so once you have driven over UDP the R4 stays biased to Wi-Fi.
#define SEARCH_PREFER_BLE 0             // cold-boot preference (0 = Wi-Fi first)
const uint32_t SEARCH_LONG_MS  = 15000; // dwell on the preferred transport
const uint32_t SEARCH_SHORT_MS = 4000;  // brief probe of the other transport
// Paired: no valid frame for this long -> treat the link as dead and stop the rover.
// This is quali_base's REMOTE_TIMEOUT (600 ms) rounded to the receiver's own 700 ms
// failsafe so the motors and the "no signal" display agree on when the link is gone.
const uint32_t FAILSAFE_MS     = 700;
const uint32_t UNPAIR_MS       = 3000;  // link lost this long -> re-enter search

// Declared up here, ahead of the first function definition in the file, because the
// Arduino preprocessor injects its auto-generated prototypes immediately before that
// point — a type used in a signature must already exist by then. The state machine that
// uses it lives further down with the transports.
enum LinkState { SEARCH_WIFI, SEARCH_BLE, PAIRED_WIFI, PAIRED_BLE };

ArduinoLEDMatrix matrix;

// ── Motors: 2x L298N, 4 MECANUM wheels, holonomic ─────────────────────────────
// One driver module per axle so the high-current leads stay short; the logic runs are
// the long ones, which is harmless at this PWM rate.
//
// Pin choice is constrained by the R4 WiFi's timers and its reserved pins:
//   * analogWrite() is only PWM on D3 D5 D6 D9 D10 D11 — on every other pin the core
//     falls back to a 0/1 digital write, so the four EN pins MUST come from that set.
//     EN takes D3 D5 D6 D9 and leaves D10/D11 free for SPI.
//   * D0/D1 are Serial1, D13 is the built-in LED, A4/A5 are I2C (SDA/SCL) — all four
//     avoided. A4/A5 in particular are kept free so the INA219 pack monitor from
//     quali_base can be added without moving a motor pin.
//   * Direction pins need no PWM, so they take the non-PWM digitals D2 D4 D7 D8 D12
//     and the analog headers A0 A1 A2 (plain digital outputs here; A0 doubles as the
//     DAC, which we do not use).
//
// Array order is {FL, RL, FR, RR} and must stay that way: drive4() indexes it
// positionally and the mecanum mix assigns a different expression to each corner.
// Reordering these silently turns a strafe into a spin. The order is a geometric fact
// about the chassis, unrelated to which module a motor is wired to.
//
// `inv` flips a motor's sense in software, exactly as swapping its two output leads
// would, so pin order stays natural (IN1 then IN2) on every entry — read the flag, not
// the pin order, to know which way a wheel turns. The values below are a STARTING
// GUESS, not a measurement: they assume the two sides are mirrored by mounting (so one
// whole side must be inverted for the rover to track straight) and that every motor is
// conventionally terminated. Four wheels turning the same absolute direction is the
// wrong state, not the right one — it drives one side backwards.
//
// BRING-UP, in this order — do not skip to the mix:
//   1. Set driveMode = MODE_WHEELTEST below, reflash, and check that the lit matrix
//      quadrant matches the wheel that actually spins. If not, reorder MOTORS[] — never
//      compensate in the mix. Driving forward is invariant to any permutation of the
//      four and rotating only distinguishes left from right, so a front/rear swap within
//      one side passes both those tests and corrupts nothing but strafe. This is the
//      only test that catches it.
//   2. Back in MODE_DRIVE, push the stick forward. Every wheel that turns backwards
//      gets its `inv` flipped. Now the rover tracks straight.
//   3. Only then check strafe, and if it goes the wrong way flip VX_SIGN — not `inv`,
//      which is by now calibrated for forward motion and would break driving straight.
struct Motor { uint8_t in1, in2, en; bool inv; };
Motor MOTORS[4] = {
  {12, A0, 6, true },  // front-left   L298N#2(front)-A: IN1=D12, IN2=A0, ENA=D6 (~)
  { 2,  4, 3, true },  // rear-left    L298N#1(rear) -A: IN1=D2,  IN2=D4, ENA=D3 (~)
  {A1, A2, 9, false},  // front-right  L298N#2(front)-B: IN3=A1,  IN4=A2, ENB=D9 (~)
  { 7,  8, 5, false},  // rear-right   L298N#1(rear) -B: IN3=D7,  IN4=D8, ENB=D5 (~)
};

// Sign of the strafe axis as the CHASSIS sees it. The mecanum mix below assumes the
// standard X roller layout (viewed from above the top rollers form an X: FL/RR at
// +45 deg, FR/RL at -45 deg). A chassis whose rollers are mirrored against that slides
// left when the mix says right — quali_base's is, hence its `-vx`. Flip this ONE
// constant if strafe goes the wrong way; one negation covers the diagonals too, since
// the mirroring is global. Keeping it here and nowhere else is what lets everything
// upstream — the matrix dot, the serial log — read `vx` as "+ = the rover slides right".
const int VX_SIGN = +1;

// ── Drive tuning (PWM units, 0..255) ──────────────────────────────────────────
const int MIN_PWM   = 60;   // below this the motors stall; commands snap up to this
const int MAX_PWM   = 255;  // full speed available
// Belt-and-braces as of NessoLink 1.1.2, which makes "a released stick reads exactly 0"
// part of the contract — the N1 dead-zones its sticks before transmitting, confirmed on
// quali_base's wire log 2026-08-06 (the L/R rest offset that read -12 for months now
// reads 0). Older handheld builds settled at +-11..14, and at a smaller deadband those
// counted as a command: the stall snap lifts them to MIN_PWM, and the shaping emits 0 or
// >=MIN_PWM and never anything between, so the rover kept creeping at ~65 PWM after the
// stick was let go until a release happened to land on 0. That was the "sometimes it
// won't stop" bug. Kept at 25 as the only cover against an out-of-date transmitter
// reintroducing it — the error is asymmetric (too wide only trims centre travel, too
// narrow lets the rover creep away). A non-zero rest reading now means a stale handheld
// build, not normal jitter.
const int REMOTE_DEADBAND = 25;   // applies to BOTH sticks — aux is the same -255..255 scale
const int SLEW_STEP       = 40;   // max PWM change per drive tick (smooth accel/brake ramp)
// quali_base ran its whole loop at this period. Here the same period is a *timer*: the
// transport has to be polled continuously (there is no Bridge thread feeding frames in
// the background any more), and a blocking delay() long enough to matter drops UDP
// datagrams and starves the BLE stack. Driving on a tick keeps SLEW_STEP meaning what
// it meant — max PWM change per 60 ms — while the radio is serviced every pass.
const uint32_t DRIVE_TICK_MS = 60;

// ── Mecanum drive modes ───────────────────────────────────────────────────────
// The wheels are MECANUM, so the rover is holonomic: it can translate sideways without
// rotating. That needs all four wheels driven independently — a skid-steer mixer (both
// left wheels together, both right wheels together) cannot strafe at all, because
// strafing requires the front and rear wheels of one side to turn OPPOSITE ways. All
// four must therefore be working; one dead wheel does not merely weaken a strafe, it
// turns it into a veer.
//
// The N1's drive stick gives two axes and pre-mixes them into tank values, so a
// one-stick remote cannot reach all three DOFs at once — hence modes, cycled by the
// stick click, each re-using the same two axes for a different pair of DOFs. A N1 with a
// second stick fitted lifts exactly that limit (see SECONDARY STICK below), which is why
// the second stick needs no gesture and no mode of its own.
//
// MODE_WHEELTEST spins one wheel at a time in MOTORS[] index order and lights the matrix
// quadrant the code BELIEVES that index sits in — step 1 of bring-up above.
enum DriveMode : uint8_t { MODE_DRIVE = 0, MODE_STRAFE, MODE_WHEELTEST, MODE_COUNT };
// HOW THE MODES ARE SELECTED — one button, two gestures
//   single click  — clear the throttle lock if it is set (that is all it does); otherwise
//                   cycle the first MODE_CYCLE_COUNT modes: DRIVE -> STRAFE -> DRIVE, and
//                   from ANY off-cycle mode, back to DRIVE. So a click is always the way
//                   out of wherever a gesture put you, in exactly one step.
//   long press    — set the THROTTLE LOCK on the current mode (idempotent: holding again
//                   while locked is a no-op, not a toggle. "Hold = lock" is a simpler
//                   thing to remember mid-drive than a toggle whose result depends on
//                   where you already are, and the click out is unambiguous either way.)
//   reflash only  — MODE_WHEELTEST
// Only the two everyday driving modes are on the CYCLE, so clicking round can never land
// on anything that behaves unexpectedly; giving up an axis costs its own deliberate
// gesture. The off-cycle mode sits AFTER the cycled pair in the enum precisely so the
// cycle can be a prefix (`% MODE_CYCLE_COUNT`) — keep that ordering, and keep DRIVE at 0,
// since a click from an off-cycle mode falls back to it.
//
// MODE_ROTATE USED TO BE A MODE HERE and was replaced by the throttle lock, following
// quali_base 2026-08-06. It was `vy = 0, w = turn` — which is DRIVE with the throttle
// forced to zero, i.e. one specific case of the lock. Generalising it costs nothing and
// gains the missing half: STRAFE with the throttle locked (slide sideways without
// creeping forward) had no way to be expressed before. The gesture, the idempotency and
// the click-out are all unchanged; what changed is that "hold" now means "drop the
// throttle axis" rather than "go to a particular mode", so it composes with the mode
// instead of replacing it. Nothing was lost: DRIVE + lock IS the old ROTATE, bit for bit.
//
// MODE_WHEELTEST keeps no gesture at all — it is reached by setting `driveMode`'s initial
// value below and reflashing. It ignores the stick and spins wheels on a timer, so
// landing on it mid-drive means the rover moves off with the stick centred; a bench
// diagnostic you need after rewiring should cost a reflash, not a slip of the thumb.
//
// quali_base has a third gesture, the double click, which selects its vision mode. There
// is no vision here and nothing else worth a gesture, so the double click is gone — and
// with it the reason a single click had to sit in a 500 ms window before committing,
// waiting to find out whether a partner click was coming. Clicks commit on the release.
const uint8_t MODE_CYCLE_COUNT = 2;      // DRIVE, STRAFE — the click cycle, and only it
// How long the button must be held to count as a long press. Clear of a careless click
// below it, and eight link samples long — the N1 samples its buttons into a frame at
// ~10 Hz, so the threshold is never decided by one lucky frame.
const uint32_t LONGPRESS_MS  = 800;
const int      WHEELTEST_PWM = 200;      // pre-stallComp; brisk enough to be unmistakable
const uint32_t WHEELTEST_MS  = 2000;     // dwell per wheel
uint8_t driveMode = MODE_DRIVE;          // set to MODE_WHEELTEST + reflash for bring-up
const NessoButton MODE_BUTTON = NESSO_BTN_STICK;   // click / hold — see above

// ── Throttle lock ─────────────────────────────────────────────────────────────
// Forces the forward/back axis to zero, leaving the mode's OTHER axis live. It is
// orthogonal to the mode rather than being one — hold in DRIVE and you can only rotate
// (the old MODE_ROTATE, exactly); hold in STRAFE and you can only slide sideways.
//
// Why it is worth a gesture at all: both are precision manoeuvres you can already perform
// by holding the throttle centred, and the lock just stops you creeping while you
// concentrate on the other axis. That is why it is a hold rather than a cycle position —
// useful when aiming, in the way when driving.
//
// It applies to the SUM of both sticks and is applied after the mix inputs are gathered.
// "Throttle locked" therefore means no forward motion can be commanded from anywhere,
// which is the only reading that keeps the matrix indicator honest — a second stick that
// could still drive forward would make the lock mean "locked, unless you use the other
// stick", which is not a lock.
bool throttleLock = false;

// ── Secondary stick: the OTHER drive mode, at the same time ───────────────────
// The N1 can carry up to three joysticks (drive seesaw + Mini JoyC + Unit JoyStick2).
// NessoLink carries the extras as aux stick 1 (auxX/auxY — present in v1 frames already,
// flagged by `hasAux`) and aux stick 2 (v2 frames only). This binds AUX STICK 1.
//
// It drives whichever of the two everyday modes is NOT selected, so the two sticks
// together always cover all three degrees of freedom at once. That is the whole point:
// modes exist only because two axes cannot express three DOFs, and a second stick lifts
// exactly that limit without adding anything to the mode machinery.
//
//   selected mode      primary stick (seesaw)     secondary stick (aux 1)
//   DRIVE  (vy, w)     throttle + rotate          throttle + strafe  -> adds vx
//   STRAFE (vy, vx)    throttle + strafe          throttle + rotate  -> adds w
//   WHEELTEST          ignored                    ignored
//
// The THROTTLE LOCK composes with this: it zeroes vy after both sticks are summed, so
// DRIVE + lock gives rotate on the primary and strafe on the secondary — two live axes,
// no forward creep — and STRAFE + lock gives the same pair from the other side.
//
// The complement is DERIVED from driveMode rather than being selectable in its own
// right, so there is still exactly one mode to think about and no new gesture to learn:
// the mode dots already say what the second stick does. NESSO_BTN_STICK2 (the aux
// stick's own click) is deliberately left unbound — a second button that also changed
// mode would make "which stick did I just click?" part of the answer, and the gesture
// set on the drive stick is already at its limit.
//
// The two sticks' contributions SUM and are then clamped, so the shared throttle axis
// adds where they agree and cancels where they fight. Cancelling is the honest result of
// a summed input and the only sane answer to two sticks commanding opposite throttles.
//
// `hasAux` is honoured rather than inferred from a non-zero reading: the encoder zeroes
// the aux fields, so 0/0 cannot distinguish "no stick" from "stick centred". Note the N1
// may ALREADY be setting it if a Mini JoyC is fitted — v1 has always had an aux slot — so
// this can change how an existing handheld drives, not just a future one.
//
// AXIS CONVENTION — specified by the protocol, so this file makes no assumptions.
// NessoLink 1.1.2 states it normatively (see the library's NessoFrame.h): every axis in
// the frame, motors and aux alike, is -255..255; `auxX` is screen-HORIZONTAL with
// positive = RIGHT; `auxY` is screen-VERTICAL with positive = UP. A released stick reads
// exactly 0 — the transmitter applies its own dead zone. "Screen-relative" means the
// TRANSMITTER has already normalised for its display orientation and for how each stick
// module is physically mounted, so this end just reads right/up and does nothing else:
// no sign flip, no swap, no rescale, and deliberately no AUX_X_SIGN / AUX_Y_SIGN knob.
//
// That contract was written down *because* quali_base got it wrong first. Before 1.1.2
// the header specified no aux range and no axis convention, so both had to be guessed —
// and both guesses were wrong: the N1 was transmitting the axes TRANSPOSED (pushing the
// stick up strafed the rover right), because its aux path skipped the orientation flags
// its drive path applied, and its own on-screen joystick disc rendered from the same
// transposed pair — so the handheld looked correct while the wire was swapped. It was
// also sending aux at +-515, the joystick module's raw ADC span, against +-255 for the
// motors. Both were fixed in the TRANSMITTER, not in any receiver: a receiver cannot know
// how a handheld's modules are mounted, so compensating locally would have left every
// other NessoLink receiver still swapped and would have inverted this one the moment the
// transmitter was fixed. Same rule as VX_SIGN in the mix — a correction belongs where the
// fact it corrects lives. If an axis is wrong, fix the transmitter.

// The mode the secondary stick drives — always the other everyday mode. WHEELTEST is off
// the click cycle and has no opposite of its own, so it takes STRAFE; it ignores both
// sticks anyway.
uint8_t auxModeFor(uint8_t m) { return (m == MODE_STRAFE) ? MODE_DRIVE : MODE_STRAFE; }

// ── Link state ────────────────────────────────────────────────────────────────
// (enum LinkState is declared with the link config at the top — see the note there.)
LinkState linkState      = SEARCH_WIFI;
uint32_t  stateEnteredMs = 0;
uint32_t  lastFrameMs    = 0;
bool      preferBle      = SEARCH_PREFER_BLE;  // sticky: follows the last transport that paired

// Length of the current search window — the preferred transport dwells longer.
uint32_t searchWindowMs() {
  bool onWifi = (linkState == SEARCH_WIFI);
  return (onWifi != preferBle) ? SEARCH_LONG_MS : SEARCH_SHORT_MS;   // preferred -> long
}

// ── Shared drive state, written by whichever transport delivered the frame ────
int      remoteL = 0, remoteR = 0;
int      remoteAuxX = 0, remoteAuxY = 0;   // aux stick 1; -255..255, +X = right, +Y = up
bool     remoteHasAux = false;
uint16_t remoteBtns = 0;
bool     haveFrame = false;

// Bound Wi-Fi remote (source-lock): once paired we only trust this origin.
IPAddress boundIp(0, 0, 0, 0);
char      pairLabel[24] = "";

static inline bool isPaired() { return linkState == PAIRED_WIFI || linkState == PAIRED_BLE; }

// ── Motor primitives ──────────────────────────────────────────────────────────
// Shape a raw remote axis onto the usable band. The dead-zone and the MIN_PWM stall snap
// happen on OPPOSITE sides of the mecanum mix, which is why this is two functions and
// not one:
//   * axisDead() runs on the INPUT axes. Rescales [DEADBAND..full] -> [0..255] linearly.
//   * stallComp() runs on each mixed WHEEL value. Rescales [1..255] -> [MIN_PWM..MAX_PWM].
// Snapping to MIN_PWM before mixing would be wrong: it would inject a 60-unit floor into
// every axis, so a pure-forward command would leak a phantom strafe/rotate term and the
// rover would crab instead of tracking straight.
int axisDead(int v) {
  int s = (v < 0) ? -1 : 1, m = abs(v);
  if (m <= REMOTE_DEADBAND) return 0;
  long out = (long)(m - REMOTE_DEADBAND) * 255 / (255 - REMOTE_DEADBAND);
  return s * (int)out;
}
// Clamp an AXIS — not a wheel value — to the -255..255 band that the mecanum mix, the
// matrix dot and dirName() all assume. Needed once two sticks can feed the same axis: a
// summed throttle reaches +-510, which would push showDir()'s dot off the panel and hand
// the ratio between the axes to the wheel-overflow rescale below instead of to the
// operator. Used on the raw aux input too, where it is a MALFORMED-FRAME GUARD and not a
// rescale — the field is int16, so a bad or hostile frame could otherwise inject +-32767
// into the mix, which the wheel-overflow rescale would treat as a legitimate full-scale
// command and divide every other axis down against. It is a no-op for anything in spec.
int clampAxis(int v) { return (v > 255) ? 255 : ((v < -255) ? -255 : v); }
int stallComp(int v) {
  if (v == 0) return 0;
  int s = (v < 0) ? -1 : 1, m = abs(v);
  if (m > 255) m = 255;
  long out = (long)(m - 1) * (MAX_PWM - MIN_PWM) / 254 + MIN_PWM;
  return s * (int)out;
}
int slew(int cur, int target) {             // ramp cur toward target by at most SLEW_STEP
  if (target > cur) return (target - cur > SLEW_STEP) ? cur + SLEW_STEP : target;
  if (target < cur) return (cur - target > SLEW_STEP) ? cur - SLEW_STEP : target;
  return cur;
}
void setMotor(const Motor& m, int v) {
  if (v > 255) v = 255; if (v < -255) v = -255;
  if (m.inv) v = -v;             // clamp first: -(-255) is in range, -(-256) would not be
  if (v >= 0) { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW); }
  else        { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); v = -v; }
  analogWrite(m.en, v);
}
// Per-wheel, not per-side: a drive(left, right) mixer feeds both left wheels the same
// value and both right wheels the same value, which is skid steer and can never strafe.
// Index order matches MOTORS[]: {FL, RL, FR, RR}.
void drive4(const int w[4]) {
  for (int i = 0; i < 4; i++) setMotor(MOTORS[i], w[i]);
}

int curPwm[4] = {0, 0, 0, 0};   // last values written; the slew limiter's state

// Cut the motors NOW, skipping the ramp, and forget where the ramp had got to. Called
// before anything that stops servicing the drive tick — every transport transition takes
// the radio down and enterWifi() blocks for up to nine seconds, during which the last
// PWM values would otherwise stand. A ramp is a comfort feature for a rover under
// command; a rover whose commands have stopped arriving should not be ramping anywhere.
void stopMotors() {
  const int zero[4] = {0, 0, 0, 0};
  drive4(zero);
  for (int i = 0; i < 4; i++) curPwm[i] = 0;
}

// ── 12x8 LED matrix ───────────────────────────────────────────────────────────
// The R4's panel is 12 columns x 8 rows and is read landscape, so unlike quali_base
// (13x8 read rotated into a logical portrait frame) no rotation is needed: fb[row][col]
// with row 0 at the top and forward = up.
uint8_t fb[8][12];
uint8_t lastFB[8][12];
bool    haveLast = false;

static inline void fbClear() { memset(fb, 0, sizeof(fb)); }
static inline void fbSet(int r, int c) {
  if (r >= 0 && r < 8 && c >= 0 && c < 12) fb[r][c] = 1;
}
void fbRender() {
  if (haveLast && memcmp(fb, lastFB, sizeof(fb)) == 0) return;   // skip identical redraws
  matrix.renderBitmap(fb, 8, 12);
  memcpy(lastFB, fb, sizeof(fb));
  haveLast = true;
}

// Proportional "virtual stick" indicator: a plus-shaped dot whose position encodes
// throttle (up = forward) and lateral command (right = right); dead-centre = stopped.
// The horizontal axis shows whichever DOF the current mode steers with, so the dot keeps
// meaning "where the stick is" in every mode. That relies on `vx` and `w` both being in
// the OPERATOR's frame (+ = right) all the way down to here — the chassis's roller
// mirroring is applied in the mix via VX_SIGN, not to the axes.
//
// ROW 0 is the status strip and the dot never reaches it: the centre is clamped to rows
// 2..6 and cols 1..10, so the plus spans rows 1..7 at most.
//   col 0 lit  = paired over Wi-Fi     col 1 lit = paired over BLE
//   right edge = mode, 1..MODE_COUNT dots from col 11 leftwards:
//                1 = DRIVE, 2 = STRAFE, 3 = WHEELTEST
// Three dots stop at col 9, so the two groups can never run together.
//
// The THROTTLE LOCK is drawn by changing the dot's SHAPE rather than by claiming another
// pixel: locked, it loses its vertical arms and becomes a horizontal bar. That is the
// indicator describing itself — the arms it drops are the axis it gave up — and it reads
// at a glance from the shape alone, which a lone extra status pixel would not. It also
// cannot be confused with "throttle merely centred": that is a plus sitting on the centre
// row, this is a bar.
void showDir(int vy, int vx, int w, bool viaBle, uint8_t mode, bool lock) {
  fbClear();
  fbSet(0, viaBle ? 1 : 0);
  for (uint8_t i = 0; i <= mode && i < MODE_COUNT; i++) fbSet(0, 11 - i);

  int lat = (vx != 0) ? vx : w;               // -255..255  right(+) / left(-)
  int c   = 1 + (lat + 255) * 9 / 510;        // 1..10
  int r   = 2 + (255 - vy) * 4 / 510;         // 2..6  (forward = top)
  fbSet(r, c); fbSet(r, c - 1); fbSet(r, c + 1);       // horizontal bar — always
  if (!lock) { fbSet(r - 1, c); fbSet(r + 1, c); }     // vertical arms — only unlocked
  fbRender();
}

// Light the 3x3 quadrant matching MOTORS[idx]'s *assumed* corner, while that wheel — and
// only that wheel — spins. If the lit corner and the spinning wheel disagree, the array
// order is wrong and strafe cannot work; fix the order, never the mecanum signs.
void showWheelTest(int idx) {
  fbClear();
  int c0 = (idx == 0 || idx == 1) ? 0 : 9;    // {FL,RL} left  | {FR,RR} right
  int r0 = (idx == 0 || idx == 2) ? 0 : 5;    // {FL,FR} front | {RL,RR} rear
  for (int dr = 0; dr < 3; dr++)
    for (int dc = 0; dc < 3; dc++) fbSet(r0 + dr, c0 + dc);
  fbRender();
}

// Searching animation. Top-left block lit = scanning Wi-Fi; top-right = scanning BLE.
// A single bar sweeps left<->right below it.
void drawSearch(bool ble) {
  fbClear();
  if (!ble) { for (int c = 0; c < 4;  c++) fbSet(0, c); }
  else      { for (int c = 8; c < 12; c++) fbSet(0, c); }
  int sweep = (int)((millis() / 120) % 12);
  fbSet(3, sweep);
  fbSet(4, sweep);
  fbRender();
}

// Link-loss indicator: a full-matrix X. The rover is stopped whenever this is showing.
void drawNoSignal() {
  fbClear();
  for (int r = 0; r < 8; r++) {
    int c1 = (r * 11 + 3) / 7;
    fbSet(r, c1);
    fbSet(r, 11 - c1);
  }
  fbRender();
}

// ── Pairing: accept a decoded frame from any transport ────────────────────────
// origin is a source IP for UDP (used for source-lock); TCP/BLE pass 0.0.0.0.
void acceptFrame(const NessoFrame& f, IPAddress origin, bool viaBle, const char* kind) {
  bool searching = !isPaired();

  if (!searching && !viaBle && linkState == PAIRED_WIFI &&
      boundIp != IPAddress(0, 0, 0, 0) && origin != IPAddress(0, 0, 0, 0) &&
      origin != boundIp) {
    return;   // source-lock: ignore a *different* Wi-Fi remote while paired
  }

  if (searching) {
    preferBle = viaBle;      // sticky: bias future searches to whatever just worked
    boundIp   = origin;
    if (origin != IPAddress(0, 0, 0, 0))
      snprintf(pairLabel, sizeof(pairLabel), "%s %d.%d.%d.%d", kind,
               origin[0], origin[1], origin[2], origin[3]);
    else
      snprintf(pairLabel, sizeof(pairLabel), "%s", kind);
    linkState      = viaBle ? PAIRED_BLE : PAIRED_WIFI;
    stateEnteredMs = millis();
    Serial.print(">>> PAIRED via "); Serial.println(pairLabel);
  }

  remoteL       = f.leftMotor;
  remoteR       = f.rightMotor;
  remoteAuxX    = f.auxX;
  remoteAuxY    = f.auxY;
  remoteHasAux  = f.hasAux;
  remoteBtns    = f.buttons;
  haveFrame     = true;
  lastFrameMs   = millis();
}

// ── TCP stream reassembly (TCP has no packet boundaries) ──────────────────────
// Resynchronises on the 0xA5 magic and uses the version byte for the length.
struct TcpParser {
  uint8_t buf[96];
  int     n = 0;
  void clear() { n = 0; }
  void feed(const uint8_t* d, int len) {
    for (int i = 0; i < len && n < (int)sizeof(buf); i++) buf[n++] = d[i];
    for (;;) {
      int start = -1;
      for (int i = 0; i < n; i++) if (buf[i] == NESSO_MAGIC) { start = i; break; }
      if (start < 0) { n = 0; break; }
      if (start > 0) { memmove(buf, buf + start, n - start); n -= start; }
      if (n < 2) break;
      uint8_t ver = buf[1];
      int need = (ver == NESSO_PROTO_VER_V1) ? NESSO_FRAME_LEN_V1
               : (ver == NESSO_PROTO_VER_V2) ? NESSO_FRAME_LEN_V2 : -1;
      if (need < 0) { memmove(buf, buf + 1, n - 1); n--; continue; }
      if (n < need) break;
      NessoFrame f;
      if (nessoDecode(buf, need, f)) {
        memmove(buf, buf + need, n - need); n -= need;
        acceptFrame(f, IPAddress(0, 0, 0, 0), false, "TCP");
      } else {
        memmove(buf, buf + 1, n - 1); n--;   // bad CRC — slide past this magic
      }
    }
  }
};

// ── Wi-Fi transport ───────────────────────────────────────────────────────────
WiFiUDP    udp;
WiFiServer tcpServer(TCP_PORT);
TcpParser  tcpParser;

void enterWifi() {
  Serial.print("[wifi] connecting to "); Serial.print(WIFI_SSID); Serial.print(' ');
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (millis() - t0 < 9000) {
    if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) break;
    drawSearch(false);
    delay(200);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[wifi] IP "); Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
    tcpServer.begin();
    tcpParser.clear();
  } else {
    Serial.println("\n[wifi] no connection (will fall back to BLE if enabled)");
  }
}

void exitWifi() {
  udp.stop();
  WiFi.disconnect();
  WiFi.end();
}

void pollWifiFrames() {
  int sz = udp.parsePacket();                              // UDP
  if (sz > 0) {
    IPAddress src = udp.remoteIP();
    uint8_t b[NESSO_FRAME_MAX_LEN];
    int want = sz < (int)NESSO_FRAME_MAX_LEN ? sz : (int)NESSO_FRAME_MAX_LEN;
    int n = udp.read(b, want);
    NessoFrame f;
    if (n >= (int)NESSO_FRAME_LEN_V1 && nessoDecode(b, n, f)) {
      acceptFrame(f, src, false, "UDP");
    } else {
      Serial.print("[udp] rx "); Serial.print(sz); Serial.print("B from ");
      Serial.print(src); Serial.println(" - not a valid frame");
    }
  }
  WiFiClient client = tcpServer.available();               // TCP
  if (client) {
    uint8_t tmp[64];
    int a = client.available(), i = 0;
    while (a-- > 0 && i < (int)sizeof(tmp)) { int c = client.read(); if (c < 0) break; tmp[i++] = (uint8_t)c; }
    if (i > 0) tcpParser.feed(tmp, i);
  }
}

// ── BLE transport (central -> NESSO peripheral) ───────────────────────────────
#if ENABLE_BLE
BLEDevice         blePeripheral;
BLECharacteristic bleTxChar;
bool              bleReady = false;
// Did BLE.begin() actually succeed for the window we are in? A failed begin() leaves the
// stack un-started, so scanning it yields nothing and end()-ing it tears down something
// that was never built. Without this the receiver burns a full SEARCH_SHORT_MS window
// staring at a dead radio on every cycle — time it should be spending listening on Wi-Fi,
// which still works. loop() reads it to bail out of the BLE window immediately.
bool              bleBegun = false;

void enterBle() {
  Serial.println("[ble] starting BLE, scanning for \"NESSO\" ...");
  bleBegun = BLE.begin();
  if (!bleBegun) {
    // Not necessarily fatal or permanent: the ESP32-S3 survives an RA4M1 reset, so a
    // reflash can leave the modem in whatever state the PREVIOUS firmware left it, and a
    // Wi-Fi/BLE hand-off can land on it mid-transition. Retried on the next BLE window,
    // which the bail-out in loop() rate-limits to one attempt per Wi-Fi dwell. A full
    // power cycle (not just the reset button) is what clears a wedged modem.
    Serial.println("[ble] BLE.begin() failed - staying on Wi-Fi, will retry next window");
    return;
  }
  BLE.scanForUuid(BLE_SVC_UUID);
}

void exitBle() {
  if (!bleBegun) return;                  // nothing was ever started; end() would be a lie
  BLE.stopScan();
  if (blePeripheral && blePeripheral.connected()) blePeripheral.disconnect();
  bleReady = false;
  BLE.end();
  bleBegun = false;
}

// While in SEARCH_BLE: find the peripheral, connect, subscribe to TX notifications.
void pollBleSearch() {
  BLEDevice dev = BLE.available();
  if (!dev) return;
  if (dev.localName() != BLE_NAME && !dev.hasLocalName()) return;
  BLE.stopScan();
  Serial.print("[ble] found "); Serial.print(dev.localName()); Serial.print(" (");
  Serial.print(dev.address()); Serial.println("), connecting");
  if (!dev.connect())            { Serial.println("[ble] connect failed"); BLE.scanForUuid(BLE_SVC_UUID); return; }
  if (!dev.discoverAttributes()) { Serial.println("[ble] discover failed"); dev.disconnect(); BLE.scanForUuid(BLE_SVC_UUID); return; }
  BLECharacteristic tx = dev.characteristic(BLE_TX_UUID);
  if (!tx || !tx.canSubscribe() || !tx.subscribe()) {
    Serial.println("[ble] TX characteristic not subscribable");
    dev.disconnect(); BLE.scanForUuid(BLE_SVC_UUID); return;
  }
  blePeripheral = dev;
  bleTxChar     = tx;
  bleReady      = true;
  Serial.println("[ble] subscribed to TX; waiting for frames");
}

// While connected: pull decoded notifications.
void pollBleConnected() {
  if (!blePeripheral.connected()) { bleReady = false; return; }
  if (bleTxChar.valueUpdated()) {
    uint8_t b[NESSO_FRAME_MAX_LEN];
    int len = bleTxChar.valueLength();
    if (len > (int)NESSO_FRAME_MAX_LEN) len = NESSO_FRAME_MAX_LEN;
    bleTxChar.readValue(b, len);
    NessoFrame f;
    if (len >= (int)NESSO_FRAME_LEN_V1 && nessoDecode(b, len, f))
      acceptFrame(f, IPAddress(0, 0, 0, 0), true, "BLE");
  }
}
bool bleReadyDbg() { return bleReady; }
bool bleBegunDbg() { return bleBegun; }
#else
void enterBle() {}
void exitBle()  {}
void pollBleSearch()    {}
void pollBleConnected() {}
bool bleReadyDbg() { return false; }
bool bleBegunDbg() { return false; }
#endif

// ── Link state machine ────────────────────────────────────────────────────────
static inline bool isWifiDomain(LinkState s) { return s == SEARCH_WIFI || s == PAIRED_WIFI; }

void gotoState(LinkState ns) {
  stopMotors();       // FIRST: a transport swap takes the radio down for seconds
  bool wasWifi = isWifiDomain(linkState), willWifi = isWifiDomain(ns);
  if (wasWifi && !willWifi)      { exitWifi(); enterBle(); }
  else if (!wasWifi && willWifi) { exitBle();  enterWifi(); }
  linkState      = ns;
  stateEnteredMs = millis();
  if (ns == SEARCH_WIFI || ns == SEARCH_BLE) {
    boundIp      = IPAddress(0, 0, 0, 0);
    pairLabel[0] = 0;
    remoteBtns   = 0;    // a held button must not survive an un-pair and fire a gesture
  }
  Serial.print("[state] -> ");
  Serial.println(ns == SEARCH_WIFI ? "SEARCH_WIFI" : ns == SEARCH_BLE ? "SEARCH_BLE"
               : ns == PAIRED_WIFI ? "PAIRED_WIFI" : "PAIRED_BLE");
}

// Periodic "still here" tick while searching, so a silent link is visible on Serial.
uint32_t lastBeatMs = 0;
void heartbeat() {
  if (millis() - lastBeatMs < 2000) return;
  lastBeatMs = millis();
  if (linkState == SEARCH_WIFI) {
    uint32_t elapsed = millis() - stateEnteredMs;
    uint32_t win     = searchWindowMs();
    uint32_t leftS   = elapsed < win ? (win - elapsed) / 1000 : 0;
    Serial.print("[search] WiFi - listening UDP:"); Serial.print(UDP_PORT);
    Serial.print(" TCP:"); Serial.print(TCP_PORT);
    Serial.print(" as "); Serial.print(WiFi.localIP());
    Serial.print(". "); Serial.print(leftS); Serial.println("s left");
  } else {
    Serial.print("[search] BLE - ");
    Serial.println(bleReadyDbg() ? "connected to NESSO, waiting for notifications"
                                 : "scanning for NESSO");
  }
}

// ── Mode gestures ─────────────────────────────────────────────────────────────
// Edges only — the N1 repeats its button state at ~10 Hz, so level-triggering would race
// through every mode while the button is held. Gated on a fresh link so a stale held-down
// button cannot cycle after the link drops.
//
// A click resolves on the RELEASE, not the press. It has to: while the button is still
// down there is no way to know whether it will turn out to be a click or a hold, and
// acting on the press meant a long press first cycled DRIVE->STRAFE and only then became
// ROTATE.
void updateMode(bool linkFresh, uint32_t now) {
  static bool     lastDown  = false;
  static uint32_t pressMs   = 0;
  static bool     longFired = false;   // this hold already acted; ignore its release

  // A stale link ABANDONS the press in progress rather than letting it fall through as a
  // release. Treating the drop as a release would change mode every time the link
  // hiccuped with the button down — the rover would come back from a dropout in STRAFE.
  // A gesture is only a gesture if we watched both of its edges.
  if (!linkFresh) { lastDown = false; longFired = false; return; }

  bool down = (remoteBtns & (uint16_t)(1u << MODE_BUTTON)) != 0;

  if (down && !lastDown) {                                        // ── press
    pressMs   = now;
    longFired = false;
  } else if (down && !longFired && (now - pressMs) >= LONGPRESS_MS) {   // ── held
    // Fires while the button is STILL DOWN, deliberately: the matrix changes under your
    // thumb, so the hold confirms itself and you know to let go. A long press that only
    // acted on release would be indistinguishable from a click until it was over.
    //
    // Only the two cycled modes can be locked. WHEELTEST ignores both sticks, so a lock
    // there would be invisible and would then cost an extra click to clear.
    if (driveMode < MODE_CYCLE_COUNT) throttleLock = true;
    longFired = true;
    Serial.println("[mode] THROTTLE LOCK (hold)");
  } else if (!down && lastDown && !longFired) {                   // ── release of a click
    // The click does ONE thing, in priority order, so it is always exactly one step back
    // toward plain DRIVE:
    //   1. locked          -> clear the lock, stay in the mode. The click is spent on the
    //                         escape and does NOT also cycle — the same rule the off-cycle
    //                         mode has always had, and it is what keeps "one click is the
    //                         way out of wherever a gesture put you" true now that a
    //                         gesture can leave you somewhere without changing the mode.
    //   2. off-cycle mode  -> back to DRIVE (WHEELTEST) rather than wrapping.
    //   3. otherwise       -> cycle DRIVE <-> STRAFE, modulo MODE_CYCLE_COUNT.
    // The lock therefore does NOT survive a mode change, which is deliberate: carrying it
    // across a cycle would mean a click could leave you with a dead throttle in a mode you
    // just arrived in, and the whole point of the click is that it always simplifies.
    if (throttleLock)                       throttleLock = false;
    else if (driveMode >= MODE_CYCLE_COUNT) driveMode = MODE_DRIVE;
    else                                    driveMode = (driveMode + 1) % MODE_CYCLE_COUNT;
    Serial.print("[mode] ");
    Serial.print(driveMode == MODE_DRIVE ? "DRIVE" : driveMode == MODE_STRAFE ? "STRAFE"
                                                                              : "WHEELTEST");
    Serial.println(throttleLock ? " +lock" : "");
  }
  lastDown = down;
}

// ── Drive tick ────────────────────────────────────────────────────────────────
// Classify the commanded motion from the axes rather than from the wheel values: with
// mecanum the wheel signs alone no longer identify the motion (a strafe and a rotate can
// drive the same wheel the same way).
const char* dirName(int vy, int vx, int w) {
  const int dz = MIN_PWM - 1;
  if (abs(vy) <= dz && abs(vx) <= dz && abs(w) <= dz) return "STOP";
  if (abs(vx) >= abs(vy) && abs(vx) >= abs(w))        return vx > 0 ? "STRAFE-R" : "STRAFE-L";
  if (abs(w)  >= abs(vy))                             return w  > 0 ? "RIGHT" : "LEFT";
  return vy > 0 ? "FORWARD" : "REVERSE";
}

void driveTick(bool linkFresh, uint32_t now) {
  int vy = 0, vx = 0, w = 0;              // forward(+), strafe-right(+), rotate-right(+)

  if (linkFresh) {
    // The N1 pre-mixes its drive stick into tank values, so undo that before re-mixing
    // for mecanum — otherwise the two mixers fight and a strafe comes out as a turn.
    int thr  = axisDead((remoteL + remoteR) / 2);
    int turn = axisDead((remoteL - remoteR) / 2);
    switch (driveMode) {
      // vx keeps the meaning declared above — +vx = the rover slides RIGHT — in every
      // mode. The chassis's roller orientation is compensated for in the mix via VX_SIGN,
      // NOT here: the matrix dot and dirName() both read vx raw, so a negation at this
      // point would mirror the display against the actual motion.
      case MODE_STRAFE: vy = thr; vx = turn; w = 0;    break;   // drive + slide sideways
      default:          vy = thr; vx = 0;    w = turn; break;   // MODE_DRIVE
    }
    // SECONDARY STICK — drives the mode that is NOT selected, so the two sticks together
    // span all three DOFs (see auxModeFor() and the block above it). The axes are taken
    // exactly as the protocol defines them (+X = right, +Y = up, -255..255): no sign flip,
    // no swap, no rescale. See the axis-convention note above for why none of that belongs
    // on this side. clampAxis() on the raw input is the malformed-frame guard.
    if (remoteHasAux) {
      int ax = axisDead(clampAxis(remoteAuxX));
      int ay = axisDead(clampAxis(remoteAuxY));
      switch (auxModeFor(driveMode)) {
        case MODE_DRIVE: vy += ay; w  += ax; break;   // selected mode is STRAFE
        default:         vy += ay; vx += ax; break;   // selected mode is DRIVE / WHEELTEST
      }
    }
    // THROTTLE LOCK, applied here and nowhere else: after every source of throttle has
    // been summed, so it means "no forward motion can be commanded" rather than "the drive
    // stick's throttle is ignored". DRIVE + lock is the old MODE_ROTATE exactly: vy = 0
    // with w still live.
    if (throttleLock) vy = 0;
    // Both sticks can push the throttle axis, so re-clamp before anything downstream reads
    // these — the mix, the matrix dot and dirName() all assume -255..255.
    vy = clampAxis(vy); vx = clampAxis(vx); w = clampAxis(w);
  }
  // else: no fresh frame — every axis stays 0 and the slew below ramps the rover down.
  // This is the failsafe, and it is the only one: unlike quali_base there is no vision
  // path that a silent remote can hand control to.

  // Mecanum mix, per wheel. Sign errors here are normal on first bring-up and show up as
  // a strafe that rotates or goes the wrong way — fix with VX_SIGN, not the `inv` flags.
  const int sx = VX_SIGN * vx;
  int wheel[4];
  wheel[0] = vy + sx + w;                 // front-left
  wheel[1] = vy - sx + w;                 // rear-left
  wheel[2] = vy - sx - w;                 // front-right
  wheel[3] = vy + sx - w;                 // rear-right

  // Scale all four down together on overflow. Clamping each wheel independently would
  // distort the ratio between them, which IS the commanded direction — the rover would
  // curve away whenever a combined command saturated.
  int mx = 0;
  for (int i = 0; i < 4; i++) if (abs(wheel[i]) > mx) mx = abs(wheel[i]);
  if (mx > 255) for (int i = 0; i < 4; i++) wheel[i] = wheel[i] * 255 / mx;

  // Wheel identification overrides the mix entirely (the stick is ignored). Runs with or
  // without a link: it is a bench diagnostic, not a driving mode.
  int testIdx = -1;
  if (driveMode == MODE_WHEELTEST) {
    testIdx = (int)((now / WHEELTEST_MS) % 4);
    for (int i = 0; i < 4; i++) wheel[i] = (i == testIdx) ? WHEELTEST_PWM : 0;
  }

  // Ramp each wheel toward its target (smooth accel/brake; avoids instant full reversals
  // and the current spikes that come with them). Matrix and motors both come from the
  // same ramped values.
  for (int i = 0; i < 4; i++) curPwm[i] = slew(curPwm[i], stallComp(wheel[i]));
  drive4(curPwm);

  // Report on change, so the serial log shows what was commanded without a running
  // stream. The large-per-wheel-change trigger is there to settle the "is this veer
  // commanded?" question: if the rover wanders while these values hold steady, the cause
  // is mechanical or electrical (traction, a weak motor, a marginal connection); if the
  // values move with it, it is coming down the wire.
  static const char* lastDir = "";
  static bool     lastFresh  = false;
  static uint8_t  lastMode   = 0xff;
  static bool     lastLock   = false;
  static int      lastW[4]   = {0, 0, 0, 0};
  bool wheelJumped = false;
  for (int i = 0; i < 4; i++) if (abs(curPwm[i] - lastW[i]) >= 30) wheelJumped = true;
  const char* dir = (testIdx >= 0) ? "WHEELTEST" : dirName(vy, vx, w);
  // The lock is in the change trigger as well as the payload: it takes an axis away, so it
  // must appear in the log at the moment it happens rather than being inferred later from
  // a throttle that never moves.
  if (dir != lastDir || linkFresh != lastFresh || driveMode != lastMode
      || throttleLock != lastLock || wheelJumped) {
    Serial.print("[drive] "); Serial.print(dir);
    Serial.print(" mode="); Serial.print(driveMode);
    if (throttleLock) Serial.print(" LOCK");
    if (remoteHasAux) Serial.print(" aux");
    Serial.print(" link="); Serial.print(linkFresh ? pairLabel : "none");
    if (testIdx >= 0) { Serial.print(" wheel="); Serial.print(testIdx); }
    Serial.print(" w=");
    for (int i = 0; i < 4; i++) { Serial.print(curPwm[i]); Serial.print(i < 3 ? ',' : ' '); }
    Serial.println();
    lastDir = dir; lastFresh = linkFresh; lastMode = driveMode; lastLock = throttleLock;
    for (int i = 0; i < 4; i++) lastW[i] = curPwm[i];
  }

  // Display ownership is split with loop(): a rover that is SEARCHING shows the search
  // sweep, which loop() draws because only it knows which transport is being probed.
  // Drawing the X here as well would have the two fighting every pass, defeating
  // fbRender()'s identical-frame skip and flickering the panel.
  if (testIdx >= 0)     showWheelTest(testIdx);
  else if (linkFresh)   showDir(vy, vx, w, linkState == PAIRED_BLE, driveMode, throttleLock);
  else if (isPaired())  drawNoSignal();            // paired but silent -> "no signal"
}

// ── Setup / loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(MOTORS[i].in1, OUTPUT);
    pinMode(MOTORS[i].in2, OUTPUT);
    pinMode(MOTORS[i].en,  OUTPUT);
  }
  stopMotors();

  matrix.begin();
  // Boot self-test, as quali_base does: light every pixel briefly. Worth the 400 ms
  // because every other screen here is sparse — a dead row or a stuck pixel would
  // otherwise read as a mode dot, a link lamp or a stick position that isn't there.
  memset(fb, 1, sizeof(fb));
  matrix.renderBitmap(fb, 8, 12);
  delay(400);
  haveLast = false;            // fbRender()'s cache never saw this frame
  drawNoSignal();

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi/BLE module not found!");
    while (true) { drawNoSignal(); delay(1000); }
  }
  Serial.println("=== r4_mecanum_base - Nesso dual-link mecanum rover ===");
  Serial.println("Searching for a remote over Wi-Fi (UDP/TCP) and BLE, alternating");
  Serial.println("(the R4's radio can't do both at once). First valid frame wins.");

#if ENABLE_BLE
  if (preferBle) { linkState = SEARCH_BLE;  enterBle(); }
  else           { linkState = SEARCH_WIFI; enterWifi(); }
#else
  linkState = SEARCH_WIFI; enterWifi();
#endif
  stateEnteredMs = millis();
}

void loop() {
  // 1) Service the radio every pass. No delay() anywhere in here: the N1 transmits at
  //    ~10 Hz and a blocking wait long enough to matter drops datagrams and starves BLE.
  switch (linkState) {
    case SEARCH_WIFI: pollWifiFrames();  break;
    case SEARCH_BLE:  pollBleSearch();
                      if (bleReadyDbg()) pollBleConnected();  break;
    case PAIRED_WIFI: pollWifiFrames();  break;
    case PAIRED_BLE:  pollBleConnected(); break;
  }

  // SAMPLE THE CLOCK *AFTER* THE POLL — never before it. acceptFrame() runs inside the
  // poll above and stamps `lastFrameMs = millis()`, so a `now` taken before it can be
  // EARLIER than lastFrameMs. These are uint32_t, so `now - lastFrameMs` then underflows
  // to ~4.29e9 — which reads as "no frame for 49 days" and trips the UNPAIR_MS branch
  // below on the very frame that just paired the link.
  //
  // Observed on hardware 2026-08-06 before this was moved: the R4 pair/unpaired at the
  // N1's own 10 Hz frame rate (436 cycles in 45 s), never staying paired long enough to
  // drive. It looked like a flaky BLE link and was not one — and it survived because
  // SEARCH_BLE <-> PAIRED_BLE does not cross the Wi-Fi/BLE domain, so gotoState() never
  // tore the radio down and the next frame simply re-paired. The same underflow reaches
  // stateEnteredMs (set by acceptFrame too), which is why one timestamp covers both.
  uint32_t now = millis();

  bool paired    = isPaired();
  bool linkFresh = paired && haveFrame && (now - lastFrameMs) < FAILSAFE_MS;

  // 2) Drive on a fixed tick, so SLEW_STEP stays "max PWM change per 60 ms". Runs while
  //    searching too — with linkFresh false the axes are zero, which both ramps a rover
  //    that just lost its link down to a stop and keeps MODE_WHEELTEST usable on the
  //    bench with no remote present at all.
  static uint32_t lastDriveMs = 0;
  if (now - lastDriveMs >= DRIVE_TICK_MS) {
    lastDriveMs = now;
    updateMode(linkFresh, now);
    driveTick(linkFresh, now);
  }

  // 3) State transitions. The drive tick above has already stopped the rover for any
  //    transition reachable from here, and gotoState() stops it again before taking the
  //    radio down — the transport swap blocks for seconds and nothing services the tick
  //    while it does.
  if (paired) {
    if (!linkFresh && (now - lastFrameMs) > UNPAIR_MS) {
      haveFrame = false;
      Serial.println("[link] lost - re-entering search");
      gotoState(linkState == PAIRED_BLE ? SEARCH_BLE : SEARCH_WIFI);
    }
  } else {
    heartbeat();
    if (driveMode != MODE_WHEELTEST) drawSearch(linkState == SEARCH_BLE);
#if ENABLE_BLE
    // Bail out of a BLE window whose BLE.begin() failed instead of sitting out its full
    // dwell: there is no stack to scan, and Wi-Fi — which still works — is where a remote
    // could actually be heard. Going back through gotoState() re-enters Wi-Fi properly and
    // leaves the next BLE window to retry, so a transient failure costs one hand-off
    // rather than every other search window.
    if (linkState == SEARCH_BLE && !bleBegunDbg()) gotoState(SEARCH_WIFI);
    else if (now - stateEnteredMs > searchWindowMs())
      gotoState(linkState == SEARCH_WIFI ? SEARCH_BLE : SEARCH_WIFI);
#else
    // Wi-Fi-only: if the connect failed (no IP), keep retrying until it lands — there is
    // no BLE phase to fall through to that would otherwise re-enter Wi-Fi.
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
      stopMotors();
      enterWifi();
      stateEnteredMs = now;
    } else if (now - stateEnteredMs > searchWindowMs()) {
      stateEnteredMs = now;
    }
#endif
  }
}
