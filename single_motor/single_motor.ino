int dirpin = 2;
int steppin = 4;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // make the pins outputs:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(dirpin, OUTPUT);
  pinMode(steppin, OUTPUT);
}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    if (Serial.read() == 'c') {
      digitalWrite(LED_BUILTIN, HIGH);
      int numSteps = Serial.parseInt();
      int dir = Serial.parseInt();

      if (dir == 0) {
        digitalWrite(dirpin, LOW);
      } else {
        digitalWrite(dirpin, HIGH);
      }

      for (int i = 0; i < numSteps; ++i) {
        digitalWrite(steppin, LOW);
        delay(100/2);
        digitalWrite(steppin, HIGH);
        delay(100/2);
      }

      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
