// Motor identification
#define NUM_MOTORS 2

// Macros for setting motor direction
#define WIND HIGH
#define UNWIND LOW

typedef struct {
  byte dirpin, steppin;
} Motor;
Motor motors[NUM_MOTORS];

// int numberOfSteps = 100;
byte ledPin = LED_BUILTIN;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 3; // milliseconds

void step(Motor m, int dir, int steps) {
  // Turn on the LED while stepping
  digitalWrite(ledPin, HIGH);

  digitalWrite(m.dirpin, dir);
  
  for(int n = 0; n < steps; n++) {
    digitalWrite(m.steppin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(m.steppin, LOW);
   
    delay(millisbetweenSteps);
  }
 
  digitalWrite(ledPin, LOW);
}


void setup()
{
  motors[0].dirpin = 2;
  motors[0].steppin = 4;
  motors[1].dirpin = 12;
  motors[1].steppin = 13;
  
  Serial.begin(9600);
  Serial.println("Starting StepperTest");

  pinMode(ledPin, OUTPUT);
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pinMode(motors[i].dirpin, OUTPUT);
    pinMode(motors[i].steppin, OUTPUT);
  }
  
  // B/c Anna says so
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}

void loop()
{
  while (Serial.available() > 0) {
    char i = Serial.read();
    int numberOfSteps = Serial.parseInt();
    switch(i) {
      case 'z':
        step(motors[0], WIND, numberOfSteps);
        break;
      case 'x':
        step(motors[0], UNWIND, numberOfSteps);
        break;
      case 'c':
        step(motors[1], WIND, numberOfSteps);
        break;
      case 'v':
        step(motors[1], UNWIND, numberOfSteps);
        break;
    }
  }
}
