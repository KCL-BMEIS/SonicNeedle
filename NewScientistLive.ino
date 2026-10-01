// Sonic Needle firmware.
//
// Protocol: the PC sends any byte to request a measurement. The Arduino fires the
// US-100 (trigger/echo mode, jumper removed) and replies with one line:
//
//     <echo_time_us>,<target_hit>\n
//
// echo_time_us is the round-trip echo time in microseconds (0 if no echo arrived
// before ECHO_TIMEOUT_US). target_hit is 1 if the needle touched the target at any
// moment since the previous request, so brief touches between polls aren't missed.
//
// Target switch wiring: needle tip contact -> TARGET_PIN, target plate -> GND.
// The internal pull-up holds the pin HIGH until the circuit closes.
// The Arduino also drives the LED and buzzer directly, so local feedback is instant
// and doesn't depend on the PC. Set FEEDBACK_ENABLED to false if you keep the
// original stand-alone LED/buzzer circuit.

const int TRIG_PIN = 11;
const int ECHO_PIN = 12;
const int TARGET_PIN = 2;
const int LED_PIN = 4;
const int BUZZER_PIN = 5;  // active buzzer (makes its own tone when powered)

const bool FEEDBACK_ENABLED = true;
const unsigned long ECHO_TIMEOUT_US = 30000;  // ~5 m round trip, beyond US-100 range
const unsigned long BAUD = 115200;

bool hitSinceLastReport = false;

void setup() {
  Serial.begin(BAUD);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TARGET_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

bool targetTouched() {
  return digitalRead(TARGET_PIN) == LOW;
}

void updateTarget() {
  bool touched = targetTouched();
  if (touched) {
    hitSinceLastReport = true;
  }
  if (FEEDBACK_ENABLED) {
    digitalWrite(LED_PIN, touched ? HIGH : LOW);
    digitalWrite(BUZZER_PIN, touched ? HIGH : LOW);
  }
}

unsigned long measureEchoUs() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Width of the HIGH pulse on ECHO is the round-trip time; 0 on timeout.
  return pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
}

void loop() {
  updateTarget();

  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    unsigned long echoUs = measureEchoUs();
    updateTarget();
    Serial.print(echoUs);
    Serial.print(',');
    Serial.println(hitSinceLastReport ? 1 : 0);
    hitSinceLastReport = false;
  }
}
