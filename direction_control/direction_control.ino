// Motor identification
#define NUM_MOTORS 2

// Macros for setting motor direction
#define WIND HIGH
#define UNWIND LOW

typedef struct {
  byte dirpin, steppin;
  int range;
} Motor;
Motor motors[NUM_MOTORS];

// int numberOfSteps = 100;
byte ledPin = LED_BUILTIN;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 10; // milliseconds


void step(Motor m, int steps) {
  // Turn on the LED while stepping
  digitalWrite(ledPin, HIGH);

  if (steps < 0) {
    steps = -steps;
    digitalWrite(m.dirpin, UNWIND);
  } else {
    digitalWrite(m.dirpin, WIND);
  }
  
  for(int n = 0; n < steps; n++) {
    digitalWrite(m.steppin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(m.steppin, LOW);
   
    delay(millisbetweenSteps);
  }
 
  digitalWrite(ledPin, LOW);
}


void calibrateMotors() {
  Serial.println("Starting motor configuration...");

  for (int i = 0; i < NUM_MOTORS; ++i) {
    Serial.print("Stretch motor ");
    Serial.print(i);
    Serial.println(" as tight as possible");
    Serial.println("Type 0 when done");

    int in;
    int stretched = 0;
    do {
      in = Serial.parseInt();
      stretched += in;
      step(motors[i], in);
    } while (in != 0);

    Serial.print("Loosen motor ");
    Serial.print(i);
    Serial.println(" so it just loses tension");
    Serial.println("Type 0 when done");

    int loosened = 0;
    do {
      in = Serial.parseInt();
      loosened += in;
      step(motors[i], in);
    } while (in != 0);

    motors[i].range = -loosened;
  }
}

void dodgeAngle(int angle) {
  if (angle >= 270 && angle <= 360) {
    step(motors[0], motors[0].range);
    delay(2000);
    step(motors[0], -motors[0].range);
  }
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

  Serial.println("Type -1 to calibrate or 0-365 to avoid");
}

void loop()
{
  while (Serial.available() > 0) {
    int angle = Serial.parseInt();

    if (angle == -1) {
      calibrateMotors();
    } else {
      dodgeAngle(angle);
    }
  }
}
