int trigPin = 11;    //Trig
int echoPin = 12;    //Echo
long duration, cm;//, inches;
float pulse_rate_hz = 10;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  while (!Serial) {
    delay(0.1);
  }
}

void loop()
{
  if (Serial.available()) {
    Serial.read();
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    duration = pulseIn(echoPin, HIGH);
    // cm = (duration/2) / 29.1;
    // cm = 343.0 * float(duration) * 1e-6 * 1e2;
    Serial.print(duration);
    Serial.println();
    //delay(1 / pulse_rate_hz - 15e-6);
  }
}
