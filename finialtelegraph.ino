
#include <Servo.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

/*
  TELEGRAPH (LAST-NIGHT FIXED VERSION) — Commented Builder Edition
  ---------------------------------------------------------------
  What it does:
  - Reads RC servo PWM on D2 (1000–2000 µs)
  - Snaps RC to one of 9 discrete "orders" (steps)
  - Requires you to HOLD the snapped step for 1 second to trigger
  - When triggered: plays a bell + runs a sweep show (2000 -> 1000 -> hold)
  - Locks out repeat triggers until you move the stick several steps away
    (prevents "machine-gun bell" and runaway triggering)

  Hardware notes:
  - D5 drives Bridge telegraph (splitter can drive 2 servos)
  - D9 drives Engine telegraph (splitter can drive 2 servos)
  - DFPlayer Mini on SoftwareSerial D11/D12

  Builder adjustments are marked with:  *** TWEAK ME ***
*/

// =====================
// MP3 (DFPlayer Mini)
// =====================
const byte MP3_RX    = 11;  // Nano RX  <- DF TX
const byte MP3_TX    = 12;  // Nano TX  -> DF RX (1k in series recommended)
const byte MP3_VOL   = 30;  // *** TWEAK ME *** 0..30 (30 = loudest)
const int  MP3_TRACK = 1;   // *** TWEAK ME *** track number (0001.mp3 = 1)

SoftwareSerial mp3Serial(MP3_RX, MP3_TX);
DFRobotDFPlayerMini mp3;
bool mp3OK = false;

// --- Bell damper (prevents MP3 "running on") ---
const unsigned long BELL_COOLDOWN_MS = 2000; // *** TWEAK ME *** 1000–3000 typical
unsigned long lastBellMs = 0;

// Plays the bell safely (cooldown + stop-first for picky DFPlayer clones)
void playBell() {
  if (!mp3OK) return;

  unsigned long now = millis();
  if (now - lastBellMs < BELL_COOLDOWN_MS) return; // block repeats
  lastBellMs = now;

  // Many DFPlayer clones behave better if we stop first
  mp3.stop();
  delay(40);
  mp3.play(MP3_TRACK);
}

// =====================
// PINS
// =====================
const uint8_t PIN_SERVO_D5 = 5;   // Bridge master (splitter drives 2)
const uint8_t PIN_SERVO_D9 = 9;   // Engine master (splitter drives 2)
const uint8_t PIN_RC_IN    = 2;   // Receiver PWM in

// =====================
// SERVO RANGE (µs)
// =====================
const int SERVO_MIN = 1000;   // *** TWEAK ME *** your servo safe low end
const int SERVO_MAX = 2000;   // *** TWEAK ME *** your servo safe high end

// 9 selections from 1000 to 2000 inclusive -> step = 125
const int STEPS   = 9;  // *** TWEAK ME *** 9 = classic 9-position feel
const int STEP_US = (SERVO_MAX - SERVO_MIN) / (STEPS - 1); // 125 with 9 steps

// How close RC must be to nearest step to count as "selected"
const int SNAP_TOL_US = 60;   // *** TWEAK ME *** widen if your radio is noisy (60–90)

// Trigger timing
const unsigned long STABLE_SELECT_MS = 1000UL;  // *** TWEAK ME *** hold time to trigger (700–1500)
const unsigned long SAMPLE_MS        = 40UL;    // *** TWEAK ME *** 20–50ms typical

// Show timing
const uint16_t STEP_DELAY_MS  = 800;  // *** TWEAK ME *** time at 2000 and 1000 during sweep
const uint16_t HOLD_SETTLE_MS = 300;  // *** TWEAK ME *** settle after final hold

// =====================
// DAMPER / RE-ARM RULE
// =====================
// After a run, require the stick to move this many *steps away*
// before allowing another run.
// Higher value = harder to re-trigger (more “safety”)
const int REARM_STEPS = 3;    // *** TWEAK ME *** 2 or 3 recommended

// =====================
// OBJECTS
// =====================
Servo s5, s9;

// =====================
// HELPERS
// =====================
int absInt(int x) { return (x < 0) ? -x : x; }

// Read RC pulse width in microseconds.
// IMPORTANT: pulseIn() can be “heavy”, so we only sample every SAMPLE_MS.
int readRcUs() {
  unsigned long pw = pulseIn(PIN_RC_IN, HIGH, 25000UL); // 25ms timeout
  if (pw == 0) return 1500;  // safe fallback if signal missing

  int us = (int)pw;

  // Clamp to normal servo range
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;

  return us;
}

// Snap rc value to nearest step (1000..2000 in 125us increments with 9 steps)
int nearestStepUs(int rc) {
  int idx = (rc - 1000 + (STEP_US / 2)) / STEP_US;  // rounded
  if (idx < 0) idx = 0;
  if (idx > (STEPS - 1)) idx = STEPS - 1;
  return 1000 + idx * STEP_US;
}

bool isSnappedToStep(int rc, int stepVal) {
  return absInt(rc - stepVal) <= SNAP_TOL_US;
}

// Convert a step value (1000..2000) to a step index (0..8)
int stepIndexFromUs(int stepUs) {
  int idx = (stepUs - 1000) / STEP_US;
  if (idx < 0) idx = 0;
  if (idx > (STEPS - 1)) idx = STEPS - 1;
  return idx;
}

// =====================
// SHOW ACTION
// =====================
void sweepThenHold(Servo &s, int holdUs) {
  s.writeMicroseconds(2000);
  delay(STEP_DELAY_MS);

  s.writeMicroseconds(1000);
  delay(STEP_DELAY_MS);

  s.writeMicroseconds(holdUs);
  delay(HOLD_SETTLE_MS);
}

// Run the full telegraph “show” for BOTH bridge and engine
// (bell + synchronized sweep + final hold)
void runShowAndHold(int holdUs) {
  // Bell first so the audio matches the moment the order changes
  playBell();

  // Do the sweep on both telegraphs (same timing)
  sweepThenHold(s5, holdUs);
  sweepThenHold(s9, holdUs);
}

// =====================
// STATE
// =====================
int  currentHoldUs = 1500;
bool hasHold       = false;

// Candidate “hold-to-trigger” tracking
bool candidateActive = false;
int  candidateHoldUs = 1500;
unsigned long candidateStartMs = 0;

unsigned long lastSampleMs = 0;

// Damper state
bool rearmed = true;          // allowed to run?
int  lastTriggerUs = 1500;    // last step that caused a run

// =====================
// SETUP / LOOP
// =====================
void setup() {
  pinMode(PIN_RC_IN, INPUT);

  s5.attach(PIN_SERVO_D5);
  s9.attach(PIN_SERVO_D9);

  // Safe start center (prevents servo slam on boot)
  s5.writeMicroseconds(1500);
  s9.writeMicroseconds(1500);
  delay(500);

  // MP3 init
  mp3Serial.begin(9600);
  delay(800);

  if (mp3.begin(mp3Serial)) {
    mp3OK = true;
    mp3.volume(MP3_VOL);
    delay(200);
  } else {
    mp3OK = false; // telegraph still works even if MP3 fails
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_MS) return;
  lastSampleMs = now;

  int rc = readRcUs();
  int stepVal = nearestStepUs(rc);
  bool snapped = isSnappedToStep(rc, stepVal);

  // Always hold current position once we have one
  // (keeps both telegraphs parked and stable)
  if (hasHold) {
    s5.writeMicroseconds(currentHoldUs);
    s9.writeMicroseconds(currentHoldUs);
  }

  // If not snapped, cancel timing and do nothing
  // This prevents “half positions” from triggering.
  if (!snapped) {
    candidateActive = false;
    return;
  }

  // ---------------------
  // RE-ARM CHECK
  // ---------------------
  // If we're NOT rearmed, we wait until the stick moves far enough away
  // (2–3 steps) from the last triggered step.
  // This prevents repeated triggering when the stick is left at a position.
  if (!rearmed) {
    int curIdx  = stepIndexFromUs(stepVal);
    int lastIdx = stepIndexFromUs(lastTriggerUs);

    if (absInt(curIdx - lastIdx) >= REARM_STEPS) {
      rearmed = true;          // ok, user moved away enough
      candidateActive = false; // force a fresh 1-second hold after re-arm
    } else {
      // Still too close to last trigger -> ignore all triggers
      candidateActive = false;
      return;
    }
  }

  // Start candidate timing
  // (must hold selection steady for STABLE_SELECT_MS)
  if (!candidateActive) {
    if (hasHold && stepVal == currentHoldUs) return; // already holding same
    candidateActive  = true;
    candidateHoldUs  = stepVal;
    candidateStartMs = now;
    return;
  }

  // Candidate active: if selection changes, restart timer
  if (stepVal != candidateHoldUs) {
    // If moved back to current hold, cancel
    if (hasHold && stepVal == currentHoldUs) {
      candidateActive = false;
      return;
    }
    candidateHoldUs  = stepVal;
    candidateStartMs = now;
    return;
  }

  // Held steady long enough -> trigger once
  if (now - candidateStartMs >= STABLE_SELECT_MS) {
    runShowAndHold(candidateHoldUs);

    currentHoldUs = candidateHoldUs;
    hasHold       = true;

    // Damper: lock out until moved far away
    lastTriggerUs = candidateHoldUs;
    rearmed       = false;

    candidateActive = false;
  }
}