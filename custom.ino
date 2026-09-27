// Spin coater control for RP2040 + REV-41-1291 (HD Hex Motor, brushed DC)
// with REV magnetic quadrature encoder.
//
// Inspired by spincoater_pid.ino's state machine / PID / UI structure,
// but the motor interface is completely different: that file drove a
// brushless motor through a bidirectional-DShot ESC and read RPM back
// from ESC telemetry. This motor is brushed DC, so speed is set with a
// plain PWM duty cycle and RPM is measured directly from the encoder.
//
// Libraries needed:
//   - hd44780          (Bill Perry) -- auto-detects I2C LCD address/expander
//   - PID_v1_bc         (same PID library the original sketch used)
//   - I2CKeyPad         (Rob Tillaart) -- reads the Lonely Binary I2C 4x4 keypad

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <PID_v1_bc.h>
#include <I2CKeyPad.h>

////////////////////////////////////////////
// PINOUT -- CONFIRM AGAINST YOUR WIRING
////////////////////////////////////////////

// Motor driver: generic PWM + direction interface.
// Works directly with TB6612/L298N-style drivers (PWM speed + direction
// bit). If you end up with a single-PWM-input driver (e.g. DRV8871), tie
// its second input to GND in hardware and ignore MOTOR_DIR_PIN.
const int MOTOR_PWM_PIN = 15;  // CHANGE: set to the GPIO wired to your motor driver's PWM/speed input
const int MOTOR_DIR_PIN = 14;  // CHANGE: set to the GPIO wired to your motor driver's direction input (ignore/remove if your driver only takes one PWM line, e.g. DRV8871)
const int MOTOR_DIR_FORWARD = HIGH;  // CHANGE: flip to LOW if the motor spins backwards once wired up

// Quadrature encoder on the REV motor.
// REV specs the bare-shaft encoder as 28 counts/rev counting rises of
// channel A only (1x). This code does full x4 decoding (both channels,
// both edges), so the effective counts/rev is ~112. VERIFY this by hand:
// spin the shaft a known number of full turns and check encoderCount.
const int ENCODER_A_PIN = 6;  // CHANGE: set to the GPIO wired to the encoder's A channel
const int ENCODER_B_PIN = 7;  // CHANGE: set to the GPIO wired to the encoder's B channel
const float ENCODER_COUNTS_PER_REV = 112.0;  // CHANGE: verify by hand-spinning the shaft a known number of turns and comparing to encoderCount; update this value to match

// Lonely Binary I2C 4x4 matrix keypad (soft membrane version). Its adapter
// ships at I2C address 0x20; if you ever wire up a second keypad on the
// same bus, bridge its A0 pad to move it to 0x21.
const uint8_t KEYPAD_I2C_ADDRESS = 0x20;  // CHANGE: if you bridged the A0 pad, or if a scan shows a different address
I2CKeyPad keyPad(KEYPAD_I2C_ADDRESS);

// Physical key -> scan-index layout for Lonely Binary's SOFT keypad, as
// given in their docs. If keys read back scrambled, you have the HEAVY
// DUTY variant instead -- CHANGE to "D#0*C987B654A321NF" for that one.
const char KEYPAD_KEYMAP[] = "DCBA#9630852*741NF";  // CHANGE: swap to the heavy-duty keymap above if this is the wrong keypad variant

// Which physical key does what. 16 keys are available; only 5 are used.
// CHANGE these to whichever keys you'd rather use -- these are just a
// reasonable numpad-style default (2/8 = up/down speed, 4/6 = down/up time).
const char KEY_START_STOP = 'A';
const char KEY_SPEED_UP   = '2';
const char KEY_SPEED_DOWN = '8';
const char KEY_TIME_UP    = '6';
const char KEY_TIME_DOWN  = '4';
const char KEY_NONE = 'N';  // library's "no key pressed" character

// I2C LCD (SunFounder 20x4, digikey CN0296D). hd44780_I2Cexp auto-detects
// the I2C address and backpack expander chip, so no guessing needed here.
hd44780_I2Cexp lcd;
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

////////////////////////////////////////////
// MACHINE PARAMS
////////////////////////////////////////////

enum State { IDLE, RAMPUP, COATING, RAMPDOWN };
State currentState = IDLE;

const int MIN_RPM = 0;
const int MAX_RPM = 6000;  // CHANGE: confirm REV-41-1291 bare-shaft free speed at your supply voltage

// RPM-based ramping
const int RAMP_START_RPM = 300;
const int RAMP_END_RPM   = 300;
const int RAMP_RATE_RPM_PER_SEC = 1500;

////////////////////////////////////////////
// BUTTON DEBOUNCE / REPEAT
////////////////////////////////////////////

const unsigned long BUTTON_DEBOUNCE_MS = 40;
const unsigned long BUTTON_REPEAT_DELAY_MS = 800;
const unsigned long BUTTON_REPEAT_INTERVAL_MS = 200;

struct KeyState {
    char lastRawKey = KEY_NONE;
    bool debounced = false;
    unsigned long lastChangeTime = 0;
    unsigned long lastRepeatTime = 0;
};

KeyState keyState;

////////////////////////////////////////////
// ENCODER (quadrature, x4 decode via interrupt)
////////////////////////////////////////////

volatile long encoderCount = 0;
volatile uint8_t lastEncoderState = 0;

// Standard quadrature state-transition table, indexed by
// (previous 2-bit AB state << 2 | current 2-bit AB state).
// Valid single-step transitions give +1/-1; invalid/skipped transitions
// (missed edges from bounce/noise) give 0 instead of corrupting the count.
static const int8_t QUAD_TABLE[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

void encoderISR() {
    uint8_t a = digitalRead(ENCODER_A_PIN);
    uint8_t b = digitalRead(ENCODER_B_PIN);
    uint8_t state = (a << 1) | b;
    uint8_t idx = (lastEncoderState << 2) | state;
    encoderCount += QUAD_TABLE[idx];
    lastEncoderState = state;
}

const unsigned long RPM_SAMPLE_INTERVAL_MS = 20;
unsigned long lastRpmSampleTime = 0;
long lastEncoderCountSample = 0;

// Returns instantaneous RPM measured over the last sample window, or -1
// if a full window hasn't elapsed yet (caller should skip updating in that case).
double sampleRPM() {
    unsigned long now = millis();
    if (now - lastRpmSampleTime < RPM_SAMPLE_INTERVAL_MS) return -1;

    noInterrupts();
    long count = encoderCount;
    interrupts();

    unsigned long dtMs = now - lastRpmSampleTime;
    long deltaCounts = count - lastEncoderCountSample;

    lastRpmSampleTime = now;
    lastEncoderCountSample = count;

    double revs = deltaCounts / ENCODER_COUNTS_PER_REV;
    double minutes = dtMs / 60000.0;
    return revs / minutes;
}

////////////////////////////////////////////
// GLOBAL VARS
////////////////////////////////////////////

int currentRPM = 0;
unsigned long stateStartTime = 0;

double motor_setRPM = 0;
double motor_actualRPM = 0;
double pid_output = 0;

int rampdownStartRPM = 0;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 250;
bool forceDisplayRefresh = true;

////////////////////////////////////////////
// USER VARS
////////////////////////////////////////////

int coatRpm = 3000;
unsigned long coatTimeMs = 30000;

////////////////////////////////////////////
// SERIAL OVERRIDE
////////////////////////////////////////////

bool serialOverride = false;
int serialTargetRPM = 0;

////////////////////////////////////////////
// PID
////////////////////////////////////////////

// Starting gains only -- these were tuned for a DShot throttle scale
// (48-2000) on a different motor, they will NOT be right for a 0-255 PWM
// duty cycle on this brushed motor. Retune empirically (start with Kp,
// add Ki once steady-state error is visible, leave Kd at 0 unless you see
// oscillation that Ki alone can't damp).
double Kp = 0.5, Ki = 2.0, Kd = 0.0;  // CHANGE: retune on the bench for your actual motor/load
PID motorPID(&motor_actualRPM, &pid_output, &motor_setRPM, Kp, Ki, Kd, DIRECT);

// Minimum PWM duty the motor needs to actually turn (overcomes static
// friction/deadband).
const int MOTOR_MIN_DUTY = 30;  // CHANGE: find empirically -- ramp PWM up from 0 until the shaft starts moving, use that value here
const int MOTOR_MAX_DUTY = 255;

////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////

// Polls the keypad and debounces it, with autorepeat while a key is held.
// Returns KEY_NONE if there's nothing new to act on this call. Sets
// isFirstPress true only on the very first debounced press of a key (not
// on autorepeat firings), so callers can tell a single tap from a hold.
char pollKeypad(bool &isFirstPress) {
    isFirstPress = false;

    char raw = keyPad.isPressed() ? keyPad.getChar() : KEY_NONE;
    unsigned long now = millis();

    if (raw != keyState.lastRawKey) {
        keyState.lastChangeTime = now;
        keyState.lastRawKey = raw;
        keyState.debounced = false;
        return KEY_NONE;
    }

    if (raw == KEY_NONE) {
        keyState.debounced = false;
        return KEY_NONE;
    }

    if (!keyState.debounced && now - keyState.lastChangeTime >= BUTTON_DEBOUNCE_MS) {
        keyState.debounced = true;
        keyState.lastRepeatTime = now;
        isFirstPress = true;
        return raw;
    }

    if (keyState.debounced &&
        now - keyState.lastRepeatTime >= BUTTON_REPEAT_INTERVAL_MS &&
        now - keyState.lastChangeTime >= BUTTON_REPEAT_DELAY_MS) {
        keyState.lastRepeatTime = now;
        return raw;
    }

    return KEY_NONE;
}

void enterState(State newState) {
    currentState = newState;
    stateStartTime = millis();
    forceDisplayRefresh = true;

    if (newState == RAMPDOWN) {
        rampdownStartRPM = (int)motor_setRPM;
        if (rampdownStartRPM < RAMP_END_RPM) {
            rampdownStartRPM = RAMP_END_RPM;
        }
    }
}

bool shouldUpdateDisplay() {
    if (forceDisplayRefresh) {
        forceDisplayRefresh = false;
        lastDisplayUpdate = millis();
        return true;
    }
    if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
        lastDisplayUpdate = millis();
        return true;
    }
    return false;
}

void updateDisplayIdle() {
    lcd.setCursor(0, 0);
    lcd.print("Spin Coater - IDLE  ");
    lcd.setCursor(0, 1);
    lcd.print("TIME:  ");
    lcd.print(coatTimeMs / 1000);
    lcd.print("s          ");
    lcd.setCursor(0, 2);
    lcd.print("SPEED: ");
    lcd.print(coatRpm);
    lcd.print("RPM          ");
}

void updateDisplayRampingUp() {
    lcd.setCursor(0, 0);
    lcd.print("Ramping up...       ");
    lcd.setCursor(0, 1);
    lcd.print("SPEED: ");
    lcd.print((int)motor_actualRPM);
    lcd.print("RPM          ");
}

void updateDisplayCoating(unsigned long remainingMs) {
    lcd.setCursor(0, 0);
    lcd.print("Coating...          ");
    lcd.setCursor(0, 1);
    lcd.print("TIME LEFT: ");
    lcd.print(remainingMs / 1000);
    lcd.print("s     ");
    lcd.setCursor(0, 2);
    lcd.print("SPEED: ");
    lcd.print((int)motor_actualRPM);
    lcd.print("RPM          ");
}

void updateDisplayRampingDown() {
    lcd.setCursor(0, 0);
    lcd.print("Ramping down...     ");
    lcd.setCursor(0, 1);
    lcd.print("SPEED: ");
    lcd.print((int)motor_actualRPM);
    lcd.print("RPM          ");
}

void updateDisplayOverride() {
    lcd.setCursor(0, 0);
    lcd.print("OVERRIDE MODE       ");
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print((int)motor_setRPM);
    lcd.print(" A:");
    lcd.print((int)motor_actualRPM);
    lcd.print("          ");
}

void handleButtons() {
    if (serialOverride) return;

    bool isFirstPress;
    char key = pollKeypad(isFirstPress);
    if (key == KEY_NONE) return;

    if (key == KEY_START_STOP) {
        if (!isFirstPress) return;  // start/stop is one-shot, no autorepeat
        if (currentState == IDLE) {
            enterState(RAMPUP);
        } else {
            enterState(RAMPDOWN);
        }
        return;
    }

    if (currentState != IDLE) return;

    if (key == KEY_SPEED_UP) {
        coatRpm += 100;
        coatRpm = constrain(coatRpm, 500, MAX_RPM);
        forceDisplayRefresh = true;
    } else if (key == KEY_SPEED_DOWN) {
        coatRpm -= 100;
        coatRpm = constrain(coatRpm, 500, MAX_RPM);
        forceDisplayRefresh = true;
    } else if (key == KEY_TIME_UP) {
        coatTimeMs += 5000;
        coatTimeMs = constrain(coatTimeMs, 5000, 180000);
        forceDisplayRefresh = true;
    } else if (key == KEY_TIME_DOWN) {
        coatTimeMs -= 5000;
        coatTimeMs = constrain(coatTimeMs, 5000, 180000);
        forceDisplayRefresh = true;
    }
}

void handleSerial() {
    if (Serial.available()) {
        delay(3);

        String s = "";
        while (Serial.available()) {
            s += (char)Serial.read();
        }
        s.trim();

        int input = s.toInt();

        if (s == "-1") {
            serialOverride = false;
            serialTargetRPM = 0;
            motor_setRPM = 0;
            pid_output = 0;
            enterState(IDLE);
            Serial.println("Serial override OFF. Returned to normal mode.");
            return;
        }

        input = constrain(input, 0, MAX_RPM);

        serialOverride = true;
        serialTargetRPM = input;
        motor_setRPM = serialTargetRPM;
        forceDisplayRefresh = true;

        if (serialTargetRPM == 0) pid_output = 0;

        Serial.print("Serial override ON. Target RPM = ");
        Serial.println(serialTargetRPM);
    }
}

////////////////////////////////////////////
// MAIN SETUP
////////////////////////////////////////////

void setup() {
    Serial.begin(115200);
    delay(3000);

    // Shared I2C bus for both the LCD and the keypad.
    // CHANGE: if your board's SDA/SCL aren't the RP2040 defaults, call
    // Wire.setSDA(pin) / Wire.setSCL(pin) here before Wire.begin().
    Wire.begin();

    if (keyPad.begin() == false) {
        Serial.println("Keypad init failed -- check I2C wiring/address.");
        while (1);
    }
    keyPad.loadKeyMap(KEYPAD_KEYMAP);

    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIR_FORWARD);
    analogWrite(MOTOR_PWM_PIN, 0);

    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    lastEncoderState = (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR, CHANGE);

    int lcdStatus = lcd.begin(LCD_COLS, LCD_ROWS);
    if (lcdStatus) {
        Serial.println("LCD init failed.");
        while (1);
    }
    lcd.backlight();
    lcd.clear();

    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    motorPID.SetSampleTime(RPM_SAMPLE_INTERVAL_MS);

    enterState(IDLE);

    Serial.println("Spin coater ready.");
    Serial.println("Serial commands:");
    Serial.print("  0..");
    Serial.print(MAX_RPM);
    Serial.println("  = direct RPM override");
    Serial.println("  0        = stop motor");
    Serial.println("  -1       = exit override mode");

    forceDisplayRefresh = true;
}

////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////

void loop() {
    handleSerial();
    handleButtons();

    unsigned long elapsed = millis() - stateStartTime;

    // SPEED MEASUREMENT
    double measuredRPM = sampleRPM();
    if (measuredRPM >= 0) {
        motor_actualRPM = 0.8 * motor_actualRPM + 0.2 * measuredRPM;
    }

    // STATE MACHINE
    if (serialOverride) {
        motor_setRPM = serialTargetRPM;
        currentRPM = serialTargetRPM;
        if (shouldUpdateDisplay()) updateDisplayOverride();
    } else {
        switch (currentState) {
        case IDLE:
            currentRPM = 0;
            motor_setRPM = 0;
            if (shouldUpdateDisplay()) updateDisplayIdle();
            break;

        case RAMPUP: {
            float elapsedSec = elapsed / 1000.0;
            int rampTarget = RAMP_START_RPM + (int)(RAMP_RATE_RPM_PER_SEC * elapsedSec);
            rampTarget = min(rampTarget, coatRpm);
            currentRPM = rampTarget;
            motor_setRPM = currentRPM;

            if (shouldUpdateDisplay()) updateDisplayRampingUp();

            if (motor_actualRPM >= coatRpm - 50) {
                enterState(COATING);
            }
            break;
        }

        case COATING:
            currentRPM = coatRpm;
            motor_setRPM = currentRPM;
            if (shouldUpdateDisplay()) updateDisplayCoating(coatTimeMs - elapsed);
            if (elapsed >= coatTimeMs) enterState(RAMPDOWN);
            break;

        case RAMPDOWN: {
            float elapsedSec = elapsed / 1000.0;
            int rampTarget = rampdownStartRPM - (int)(RAMP_RATE_RPM_PER_SEC * elapsedSec);
            currentRPM = max(rampTarget, RAMP_END_RPM);
            motor_setRPM = currentRPM;
            if (shouldUpdateDisplay()) updateDisplayRampingDown();
            if (currentRPM <= RAMP_END_RPM) enterState(IDLE);
            break;
        }
        }
    }

    // PID MOTOR CONTROL
    if (motor_setRPM <= 0) {
        pid_output = 0;
        analogWrite(MOTOR_PWM_PIN, 0);
    } else {
        motorPID.Compute();
        analogWrite(MOTOR_PWM_PIN, (int)pid_output);
    }

    // SERIAL DEBUG OUTPUT
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("Mode: ");
        Serial.print(serialOverride ? "OVERRIDE" :
                     currentState == IDLE ? "IDLE" :
                     currentState == RAMPUP ? "RAMPUP" :
                     currentState == COATING ? "COATING" : "RAMPDOWN");
        Serial.print("\tSetRPM: ");
        Serial.print(motor_setRPM);
        Serial.print("\tActualRPM: ");
        Serial.print(motor_actualRPM);
        Serial.print("\tPWM: ");
        Serial.println(pid_output);
    }
}
