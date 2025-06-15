#include <Arduino.h>

/* ==== PIN MAP – UPDATE IF YOU WIRED DIFFERENTLY =========================== */
const uint8_t PIN_L_PWM  = 9;   // ENA or AIN1 on your H-bridge
const uint8_t PIN_L_DIR  = 8;
const uint8_t PIN_R_PWM  = 10;  // ENB or BIN1
const uint8_t PIN_R_DIR  = 7;

const uint8_t PIN_L_ENC  = 2;   // interrupt-capable
const uint8_t PIN_R_ENC  = 3;   // interrupt-capable

/* ==== SERIAL SETTINGS ===================================================== */
constexpr unsigned BAUD           = 115200;   // must match Pi
constexpr unsigned STATUS_PERIOD  = 50;       // ms between encoder packets

/* ==== RUNTIME GLOBALS ===================================================== */
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

/* ---------- interrupt service routines ----------------------------------- */
void IRAM_ATTR onLeftEncoder()  { leftTicks++;  }
void IRAM_ATTR onRightEncoder() { rightTicks++; }

/* ---------- helper to drive a motor -------------------------------------- */
void setMotor(uint8_t pwmPin, uint8_t dirPin, int speedPct)
{
    bool dir = speedPct >= 0;
    uint8_t duty = constrain(abs(speedPct), 0, 100) * 255 / 100;  // % → 0-255
    digitalWrite(dirPin, dir);
    analogWrite(pwmPin, duty);
}

/* ========================================================================== */
void setup()
{
    pinMode(PIN_L_PWM, OUTPUT);  pinMode(PIN_L_DIR, OUTPUT);
    pinMode(PIN_R_PWM, OUTPUT);  pinMode(PIN_R_DIR, OUTPUT);

    pinMode(PIN_L_ENC, INPUT_PULLUP);
    pinMode(PIN_R_ENC, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_L_ENC), onLeftEncoder,  RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_R_ENC), onRightEncoder, RISING);

    Serial.begin(BAUD);
}

/* ========================================================================== */
void loop()
{
    /* 1 ── check if the Pi sent a command line ----------------------------- */
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();                                     // kill CR/LF
        if (line.startsWith("V,")) {                     // velocity line
            int comma = line.indexOf(',', 2);
            int left  = line.substring(2, comma).toInt();
            int right = line.substring(comma + 1).toInt();
            setMotor(PIN_L_PWM, PIN_L_DIR, left);
            setMotor(PIN_R_PWM, PIN_R_DIR, right);
        } else if (line.startsWith("S")) {               // stop line
            setMotor(PIN_L_PWM, PIN_L_DIR, 0);
            setMotor(PIN_R_PWM, PIN_R_DIR, 0);
        }
    }

    /* 2 ── every 50 ms send encoder deltas back ---------------------------- */
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= STATUS_PERIOD) {
        static long prevL = 0, prevR = 0;
        long curL = leftTicks, curR = rightTicks;
        long dL = curL - prevL;
        long dR = curR - prevR;
        prevL = curL;  prevR = curR;

        Serial.print("E,");
        Serial.print(dL); Serial.print(',');
        Serial.print(dR); Serial.println(",0");  // heading placeholder
        lastSend = millis();
    }
}
