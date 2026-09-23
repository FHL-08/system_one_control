/*
 * motor_firmware.ino: Arduino firmware for the Von fuzzy speed-controller demo.
 *
 * Protocol (115200 baud):
 *   <- host sends "D<0-255>\n"   set PWM duty (motor drive pin)
 *   -> board sends "RPM <float>" every REPORT_MS
 *   <- host sends "S\n"          emergency stop (duty 0)
 *
 * Hardware (defaults, edit below):
 *   PWM_PIN  = 5   -> motor driver PWM/ENA input (e.g. L298N ENA, ESC signal)
 *   ENC_A    = 2   -> encoder channel A (interrupt)
 *   ENC_B    = 3   -> encoder channel B (interrupt; optional but recommended)
 *   ENCODER_PPR    = 3576 -> output-shaft counts/rev for the LGM12-N20
 *                    (encoder CPR x gear ratio x 4 edges)
 *
 * If you have a single-signal tach/hall sensor instead of a quadrature
 * encoder, set SINGLE_CHANNEL_TACH to true and leave ENC_B unconnected.
 */

const uint8_t PWM_PIN = 5;
const uint8_t ENC_A   = 2;
const uint8_t ENC_B   = 3;
const bool SINGLE_CHANNEL_TACH = false;
const long ENCODER_PPR = 3576;          // output-shaft counts/rev (encoder CPR x gear ratio x 4 edges)
const unsigned long REPORT_MS = 100;    // RPM report / control sample rate

volatile long encoder_ticks = 0;
int duty = 0;

void isrA() {
  if (SINGLE_CHANNEL_TACH) {
    encoder_ticks++;
    return;
  }
  encoder_ticks += (digitalRead(ENC_A) == digitalRead(ENC_B)) ? 1 : -1;
}

void isrB() {
  encoder_ticks += (digitalRead(ENC_A) != digitalRead(ENC_B)) ? 1 : -1;
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  if (!SINGLE_CHANNEL_TACH)
    attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);
  analogWrite(PWM_PIN, 0);
  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  // --- command intake ---
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      if (n > 0) {
        if (buf[0] == 'D') {
          duty = constrain(atoi(buf + 1), 0, 255);
          analogWrite(PWM_PIN, duty);
        } else if (buf[0] == 'S') {
          duty = 0;
          analogWrite(PWM_PIN, 0);
        }
      }
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }

  // --- RPM report ---
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= REPORT_MS) {
    noInterrupts();
    long ticks = encoder_ticks;
    encoder_ticks = 0;
    interrupts();
    float rpm = (float)ticks / ENCODER_PPR * (60000.0f / (now - last));
    Serial.print("RPM ");
    Serial.println(rpm, 1);
    last = now;
  }
}
