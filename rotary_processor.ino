#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- TMC2209 Pins ---
#define DIR_PIN          3
#define STEP_PIN         4
#define SW_TX            5
#define SW_RX            6

// --- Display and Rotary Module Pins ---
#define PUSH_BTN_PIN     7
#define ROT_CLK          2
#define ROT_DT           8

// --- Speaker ---
#define SPEAKER_PIN      9

// --- Buttons ---
#define CONF_BTN_PIN     10
#define BACK_BTN_PIN     12

// --- TMC2209 Config ---
#define DRIVER_ADDRESS   0b00
#define R_SENSE          0.11f
#define RAMP_TICK_MS     20

SoftwareSerial SoftSerial(SW_RX, SW_TX);
TMC2209Stepper driver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);
U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// -----------------------------------------------
// MODIFIERS
// -----------------------------------------------
int   pushPullValue    = 0;     // -3 to +3
int   timeCompValue    = 0;     // 0 to 10 (represents 0% to +20% in 2% steps)

// -----------------------------------------------
// MOTOR STEPS
// -----------------------------------------------
struct MotorStep {
  long          targetSpeed;
  unsigned long pauseTime;
  unsigned long rampUpTime;
  unsigned long dwellTime;
  unsigned long rampDownTime;
};

MotorStep steps60rpmConstant2min[] = {
  { 15000, 0, 1000, 120000, 1000 },
};
MotorStep steps75rpmConstant20sec[] = {
  { 18750, 0, 1000, 20000, 1000 },
};
MotorStep noMovement5s[] = {
  { 0, 0, 0, 5000, 0 },
};
MotorStep steps60rpm5sReversal[] = {
  {  15000, 750, 500, 5000, 500 },
  { -15000, 750, 500, 5000, 500 },
};
MotorStep steps75rpm1sReversal[] = {
  {  18750, 750, 500, 1000, 400 },
  { -18750, 750, 500, 1000, 400 },
};

// -----------------------------------------------
// SUBPROCESS & PROCESS
// -----------------------------------------------
struct SubProcess {
  const char* name;
  MotorStep*  steps;
  int         length;
  int         baseRepeats;      // unmodified repeat count
  bool        isDev;            // affected by push/pull modifier
  bool        timeComp;         // affected by time compensation modifier
};

struct Process {
  const char*  name;
  SubProcess*  subProcesses;
  int          subCount;
  float        pushPullPerStop;  // multiplier per push/pull stop e.g. 0.15 = 15% per stop (multiplicative)
};

SubProcess subProcessesHc110H70F0[] = {
  //  name               steps                     len                                          baseRep  isDev   timeComp
  { "Pre-wash",          steps60rpmConstant2min,   sizeof(steps60rpmConstant2min)/sizeof(steps60rpmConstant2min[0]),    1,  false, false },
  { "Dev HC110 1:63",    steps60rpm5sReversal,     sizeof(steps60rpm5sReversal)/sizeof(steps60rpm5sReversal[0]),       39,  true,  true  },
  { "IlfoStop 1:19",     steps60rpm5sReversal,     sizeof(steps60rpm5sReversal)/sizeof(steps60rpm5sReversal[0]),        1,  false, false },
  { "Ilford Fix 1:4",    steps60rpm5sReversal,     sizeof(steps60rpm5sReversal)/sizeof(steps60rpm5sReversal[0]),       13,  false, false },
};

SubProcess subProcessesKodakC41100F0[] = {
  //  name               steps                     len                                          baseRep  isDev   timeComp
  { "Warm-up",           steps60rpmConstant2min,   sizeof(steps60rpmConstant2min)/sizeof(steps60rpmConstant2min[0]),    2,  false, false },
  { "Pre-soak",          steps75rpmConstant20sec,  sizeof(steps75rpmConstant20sec)/sizeof(steps75rpmConstant20sec[0]),  1,  false, false },
  { "Develop",           steps75rpm1sReversal,     sizeof(steps75rpm1sReversal)/sizeof(steps75rpm1sReversal[0]),       34,  true,  true  },
  { "Bleach",            steps75rpm1sReversal,     sizeof(steps75rpm1sReversal)/sizeof(steps75rpm1sReversal[0]),       69,  false, true },
  { "Wash 1/3",          steps75rpm1sReversal,     sizeof(steps75rpm1sReversal)/sizeof(steps75rpm1sReversal[0]),        5,  false, false },
  { "Wash 2/3",          steps75rpm1sReversal,     sizeof(steps75rpm1sReversal)/sizeof(steps75rpm1sReversal[0]),        5,  false, false },
  { "Wash 3/3",          steps75rpm1sReversal,     sizeof(steps75rpm1sReversal)/sizeof(steps75rpm1sReversal[0]),        5,  false, false },
  { "Fixer",             steps75rpm1sReversal,     sizeof(steps75rpm1sReversal)/sizeof(steps75rpm1sReversal[0]),       69,  false, true },
  { "Wash (M)",          noMovement5s,             sizeof(noMovement5s)/sizeof(noMovement5s[0]),                       38,  false, false },
  { "Final Rinse (M)",   noMovement5s,             sizeof(noMovement5s)/sizeof(noMovement5s[0]),                        2,  false, false },
};

Process processes[] = {
  { "BW HC110 70F",    subProcessesHc110H70F0,    sizeof(subProcessesHc110H70F0)/sizeof(subProcessesHc110H70F0[0]),       0.35f },
  { "KODAK C41 100F",  subProcessesKodakC41100F0, sizeof(subProcessesKodakC41100F0)/sizeof(subProcessesKodakC41100F0[0]), 0.25f },
};
const int NUM_PROCESSES = sizeof(processes) / sizeof(processes[0]);

// -----------------------------------------------
// MODIFIER CALCULATION
// -----------------------------------------------
int calcEffectiveRepeats(SubProcess& sub, Process& proc) {
  float multiplier = 1.0f;

  if (sub.isDev && pushPullValue != 0) {
    multiplier *= pow(1.0f + proc.pushPullPerStop, (float)pushPullValue);
  }
  if (sub.timeComp && timeCompValue != 0) {
    multiplier *= 1.0f + (timeCompValue * 0.02f);  // 2% per step
  }

  int result = (int)round((float)sub.baseRepeats * multiplier);
  return max(result, 1);  // always at least 1 repeat
}

// -----------------------------------------------
// STATE MACHINE
// -----------------------------------------------
enum AppState {
  STATE_MENU,
  STATE_MODIFIER,   // rotary push → edit modifiers
  STATE_RUNNING,
  STATE_AWAIT_NEXT
};
enum Phase {
  PHASE_IDLE,
  PHASE_PAUSE,
  PHASE_RAMP_UP,
  PHASE_DWELL,
  PHASE_RAMP_DOWN
};
enum ModifierField { MOD_PUSHPULL, MOD_TIMECOMP };

AppState      appState         = STATE_MENU;
Phase         currentPhase     = PHASE_IDLE;
ModifierField activeModField   = MOD_PUSHPULL;
int           selectedProcess  = 0;
int           currentSub       = 0;
int           currentStep      = 0;
int           currentRepeat    = 0;
int           activeRepeats    = 0;   // effective repeats for current subprocess
unsigned long phaseStart       = 0;

Process*    activeProcess  = nullptr;
SubProcess* activeSub      = nullptr;

// --- Input state ---
volatile int rotaryDelta   = 0;
bool lastConfState         = false;
bool lastBackState         = false;
bool lastPushState         = false;

// -----------------------------------------------
// DISPLAY HELPERS
// -----------------------------------------------
void drawStrP(int x, int y, const __FlashStringHelper* fstr) {
  char buf[32];
  strncpy_P(buf, (const char*)fstr, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  u8g2.drawStr(x, y, buf);
}
void drawDriverError() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB10_tr);
    drawStrP(4, 20, F("Driver Error!"));
    u8g2.drawHLine(0, 24, 128);
    u8g2.setFont(u8g2_font_helvR08_tr);
    drawStrP(4, 38, F("TMC2209 not found."));
    drawStrP(4, 50, F("Check wiring and"));
    drawStrP(4, 60, F("restart."));
  } while (u8g2.nextPage());
}

void drawMenu() {
  // Build modifier status string for top right e.g. "P+1 T+6%"
  char modBuf[20];
  char ppSign = pushPullValue >= 0 ? '+' : '-';
  snprintf(modBuf, sizeof(modBuf), "P%c%d T+%d%%",
    ppSign, abs(pushPullValue), timeCompValue * 2);

  u8g2.firstPage();
  do {
    // Header row: "Process:" on left, modifiers on right
    u8g2.setFont(u8g2_font_helvB08_tr);
    drawStrP(4, 11, F("Process:"));
    u8g2.drawStr(68, 11, modBuf);
    u8g2.drawHLine(0, 14, 128);

    for (int i = 0; i < NUM_PROCESSES; i++) {
      int y = 29 + (i * 18);
      if (i == selectedProcess) {
        u8g2.drawRBox(2, y - 12, 124, 15, 2);
        u8g2.setDrawColor(0);
        u8g2.drawStr(6, y, processes[i].name);
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(6, y, processes[i].name);
      }
    }
  } while (u8g2.nextPage());
}

void drawModifier() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB10_tr);
    drawStrP(4, 12, F("Modifiers"));
    u8g2.drawHLine(0, 15, 128);
    u8g2.setFont(u8g2_font_helvR08_tr);

    // Push/Pull row
    char ppBuf[16];
    char ppSign = pushPullValue >= 0 ? '+' : '-';
    snprintf(ppBuf, sizeof(ppBuf), "Push/Pull: %c%d", ppSign, abs(pushPullValue));

    if (activeModField == MOD_PUSHPULL) {
      u8g2.drawRBox(2, 20, 124, 15, 2);
      u8g2.setDrawColor(0);
      u8g2.drawStr(6, 31, ppBuf);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(6, 31, ppBuf);
    }

    // Time compensation row
    char tcBuf[20];
    snprintf(tcBuf, sizeof(tcBuf), "Time Comp: +%d%%", timeCompValue * 2);

    if (activeModField == MOD_TIMECOMP) {
      u8g2.drawRBox(2, 38, 124, 15, 2);
      u8g2.setDrawColor(0);
      u8g2.drawStr(6, 49, tcBuf);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(6, 49, tcBuf);
    }

    u8g2.setFont(u8g2_font_helvR08_tr);
    drawStrP(4, 63, F("PUSH=switch  BACK=done"));
  } while (u8g2.nextPage());
}

void drawRunning() {
  char subBuf[10];
  snprintf(subBuf, sizeof(subBuf), "%d/%d", currentSub + 1, activeProcess->subCount);

  char modBuf[14];
  char ppSign = pushPullValue >= 0 ? '+' : '-';
  snprintf(modBuf, sizeof(modBuf), "P%c%d  T+%d%%", ppSign, abs(pushPullValue), timeCompValue * 2);

  int barWidth = (activeRepeats > 0)
    ? (int)((float)(currentRepeat + 1) / (float)(activeRepeats) * 88)
    : 0;

  u8g2.firstPage();
  do {
    // Line 1: process name
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.drawStr(4, 10, activeProcess->name);

    // Line 2: modifiers
    u8g2.drawStr(4, 20, modBuf);
    u8g2.drawHLine(0, 24, 128);

    // Line 3: subprocess name
    u8g2.setFont(u8g2_font_helvB10_tr);
    u8g2.drawStr(4, 38, activeSub->name);

    // Line 4: sub x/y left, progress bar right
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.drawStr(4, 58, subBuf);
    u8g2.drawFrame(28, 50, 96, 8);
    u8g2.drawBox(28, 50, barWidth, 8);
  } while (u8g2.nextPage());
}

void drawAwaitNext() {
  bool hasNext = (currentSub + 1) < activeProcess->subCount;
  bool hasPrev = currentSub > 0;
  SubProcess& thisSub = activeProcess->subProcesses[currentSub];

  char modBuf[14];
  char ppSign = pushPullValue >= 0 ? '+' : '-';
  snprintf(modBuf, sizeof(modBuf), "P%c%d  T+%d%%", ppSign, abs(pushPullValue), timeCompValue * 2);

  char navBuf[12];
  snprintf(navBuf, sizeof(navBuf), "%s%d/%d%s",
    hasPrev ? "< " : "  ",
    currentSub + 1,
    activeProcess->subCount,
    hasNext ? " >" : "  "
  );

  u8g2.firstPage();
  do {
    // Line 1: process name
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.drawStr(4, 10, activeProcess->name);

    // Line 2: modifiers
    u8g2.drawStr(4, 20, modBuf);
    u8g2.drawHLine(0, 24, 128);

    // Line 3: subprocess name
    u8g2.setFont(u8g2_font_helvB10_tr);
    u8g2.drawStr(4, 38, thisSub.name);

    // Line 4: nav
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.drawStr(4, 52, navBuf);

    u8g2.drawHLine(0, 56, 128);
    drawStrP(4, 64, F("CONF to start"));
  } while (u8g2.nextPage());
}

void drawDone() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB10_tr);
    drawStrP(4, 35, F("Done!"));
  } while (u8g2.nextPage());
}

// -----------------------------------------------
// HELPERS
// -----------------------------------------------
void handleRotary() {
  if (digitalRead(ROT_DT) == HIGH) {
    rotaryDelta++;
  } else {
    rotaryDelta--;
  }
}

void setSpeed(long speed) {
  driver.VACTUAL(speed);
  delay(2);
}

long interpolate(long from, long to, float t) {
  return from + (long)((to - from) * t);
}

void playDoneBeep() {
  for (int i = 0; i < 3; i++) {
    tone(SPEAKER_PIN, 1000, 150);
    delay(250);
  }
  noTone(SPEAKER_PIN);
}

void playErrorBeep() {
  for (int i = 0; i < 3; i++) {
    tone(SPEAKER_PIN, 300, 300);
    delay(500);
  }
  noTone(SPEAKER_PIN);
}

void checkTmcDriver() {
  uint8_t version = driver.version();
  Serial.print("Driver version: "); Serial.println(version);
  if (version != 33) {
    drawDriverError();
    playErrorBeep();
    while (true) { delay(1000); }
  }
}

// -----------------------------------------------
// SEQUENCE CONTROL
// -----------------------------------------------
void startSubProcess(unsigned long now) {
  activeSub     = &activeProcess->subProcesses[currentSub];
  activeRepeats = calcEffectiveRepeats(*activeSub, *activeProcess);
  currentStep   = 0;
  currentRepeat = 0;
  currentPhase  = PHASE_PAUSE;
  phaseStart    = now;
  appState      = STATE_RUNNING;
  drawRunning();
}

void startProcess(int index, unsigned long now) {
  activeProcess = &processes[index];
  currentSub    = 0;
  currentPhase  = PHASE_IDLE;
  appState      = STATE_AWAIT_NEXT;
  lastConfState = true;
  drawAwaitNext();
}

void cancelProcess() {
  setSpeed(0);
  currentPhase = PHASE_IDLE;
  appState     = STATE_MENU;
  drawMenu();
}

void advanceStep(unsigned long now) {
  currentStep++;

  if (currentStep >= activeSub->length) {
    currentRepeat++;
    if (currentRepeat >= activeRepeats) {
      setSpeed(0);
      currentPhase = PHASE_IDLE;
      currentSub++;
      playDoneBeep();
      lastConfState = true;

      if (currentSub >= activeProcess->subCount) {
        appState = STATE_MENU;
        drawDone();
        delay(2000);
        drawMenu();
      } else {
        appState = STATE_AWAIT_NEXT;
        drawAwaitNext();
      }
    } else {
      currentStep  = 0;
      currentPhase = PHASE_PAUSE;
      phaseStart   = now;
      drawRunning();
    }
  } else {
    currentPhase = PHASE_PAUSE;
    phaseStart   = now;
  }
}

// -----------------------------------------------
// SETUP
// -----------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN,      OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN,       OUTPUT); digitalWrite(DIR_PIN,  LOW);
  pinMode(PUSH_BTN_PIN,  INPUT_PULLUP);
  pinMode(ROT_CLK,       INPUT_PULLUP);
  pinMode(ROT_DT,        INPUT_PULLUP);
  pinMode(BACK_BTN_PIN,  INPUT_PULLUP);
  pinMode(CONF_BTN_PIN,  INPUT_PULLUP);
  pinMode(SPEAKER_PIN,   OUTPUT);

  u8g2.begin();
  drawMenu();

  SoftSerial.begin(115200);
  driver.beginSerial(115200);
  driver.begin();
  driver.toff(5);
  driver.rms_current(1770);
  driver.microsteps(32);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.VACTUAL(0);

  checkTmcDriver();

  attachInterrupt(digitalPinToInterrupt(ROT_CLK), handleRotary, FALLING);
}

// -----------------------------------------------
// LOOP
// -----------------------------------------------
void loop() {
  unsigned long now      = millis();
  bool confState         = digitalRead(CONF_BTN_PIN) == LOW;
  bool backState         = digitalRead(BACK_BTN_PIN) == LOW;
  bool pushState         = digitalRead(PUSH_BTN_PIN) == LOW;

  // --- MENU ---
  if (appState == STATE_MENU) {
    if (rotaryDelta != 0) {
      selectedProcess = (selectedProcess + (rotaryDelta > 0 ? 1 : -1) + NUM_PROCESSES) % NUM_PROCESSES;
      rotaryDelta = 0;
      drawMenu();
    }
    // Rotary push → enter modifier screen
    if (pushState && !lastPushState) {
      activeModField = MOD_PUSHPULL;
      appState       = STATE_MODIFIER;
      drawModifier();
    }
    if (confState && !lastConfState) {
      startProcess(selectedProcess, now);
    }
  }

  // --- MODIFIER SCREEN ---
  if (appState == STATE_MODIFIER) {
    if (rotaryDelta != 0) {
      int delta = rotaryDelta > 0 ? 1 : -1;
      rotaryDelta = 0;
      if (activeModField == MOD_PUSHPULL) {
        pushPullValue = constrain(pushPullValue + delta, -3, 3);
      } else {
        timeCompValue = constrain(timeCompValue + delta, 0, 10);
      }
      drawModifier();
    }
    // Rotary push → switch between fields
    if (pushState && !lastPushState) {
      activeModField = (activeModField == MOD_PUSHPULL) ? MOD_TIMECOMP : MOD_PUSHPULL;
      drawModifier();
    }
    // Back → return to menu
    if (backState && !lastBackState) {
      appState = STATE_MENU;
      drawMenu();
    }
  }

  // --- AWAIT NEXT SUBPROCESS ---
  if (appState == STATE_AWAIT_NEXT) {
    if (rotaryDelta != 0) {
      if (rotaryDelta > 0 && (currentSub + 1) < activeProcess->subCount) {
        currentSub++;
      } else if (rotaryDelta < 0 && currentSub > 0) {
        currentSub--;
      }
      rotaryDelta = 0;
      drawAwaitNext();
    }
    if (confState && !lastConfState) {
      if (currentSub >= activeProcess->subCount) {
        appState     = STATE_MENU;
        currentPhase = PHASE_IDLE;
        drawMenu();
      } else {
        startSubProcess(now);
      }
    }
    if (backState && !lastBackState) {
      cancelProcess();
    }
  }

  // --- RUNNING ---
  if (appState == STATE_RUNNING) {
    if (backState && !lastBackState) {
      cancelProcess();
    }

    if (currentPhase != PHASE_IDLE) {
      MotorStep&    s       = activeSub->steps[currentStep];
      unsigned long elapsed = now - phaseStart;

      switch (currentPhase) {
        case PHASE_PAUSE:
          if (elapsed == 0) setSpeed(0);
          if (elapsed >= s.pauseTime) {
            currentPhase = PHASE_RAMP_UP;
            phaseStart   = now;
          }
          break;

        case PHASE_RAMP_UP:
          if (s.rampUpTime == 0) {
            setSpeed(s.targetSpeed);
            currentPhase = PHASE_DWELL;
            phaseStart   = now;
          } else {
            float t = (float)elapsed / (float)s.rampUpTime;
            if (t >= 1.0f) {
              setSpeed(s.targetSpeed);
              currentPhase = PHASE_DWELL;
              phaseStart   = now;
            } else {
              setSpeed(interpolate(0, s.targetSpeed, t));
            }
          }
          break;

        case PHASE_DWELL:
          if (elapsed == 0) setSpeed(s.targetSpeed);
          if (elapsed >= s.dwellTime) {
            currentPhase = PHASE_RAMP_DOWN;
            phaseStart   = now;
          }
          break;

        case PHASE_RAMP_DOWN:
          if (s.rampDownTime == 0) {
            setSpeed(0);
            advanceStep(now);
          } else {
            float t = (float)elapsed / (float)s.rampDownTime;
            if (t >= 1.0f) {
              setSpeed(0);
              advanceStep(now);
            } else {
              setSpeed(interpolate(s.targetSpeed, 0, t));
            }
          }
          break;

        default: break;
      }
    }
  }

  lastConfState = confState;
  lastBackState = backState;
  lastPushState = pushState;

  delay(RAMP_TICK_MS);
}