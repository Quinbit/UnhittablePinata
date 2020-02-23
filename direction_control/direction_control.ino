// Motor identification
#define NUM_MOTORS 2

// Macros for setting motor direction
#define WIND HIGH
#define UNWIND LOW

typedef struct {
  byte dirpin, steppin;
  int steprange, stepsep;
  int wind_dir, unwind_dir;
} Motor;
Motor motors[NUM_MOTORS];

// int numberOfSteps = 100;
byte ledPin = LED_BUILTIN;
int pulseWidthMicros = 20;  // microseconds

//void step(Motor m, int steps) {
//  // Turn on the LED while stepping
//  digitalWrite(ledPin, HIGH);
//
//  if (steps < 0) {
//    steps = -steps;
//    digitalWrite(m.dirpin, m.unwind_dir);
//  } else {
//    digitalWrite(m.dirpin, m.wind_dir);
//  }
//  
//  for(int n = 0; n < steps; n++) {
//    digitalWrite(m.steppin, HIGH);
//    delayMicroseconds(pulseWidthMicros);
//    digitalWrite(m.steppin, LOW);
//   
//    delayMicroseconds(m.stepsep);
//  }
// 
//  digitalWrite(ledPin, LOW);
//}

void stepBoth(int leftSteps, int rightSteps, bool slow) {
  if (leftSteps < 0) {
    leftSteps = -leftSteps;
    digitalWrite(motors[0].dirpin, motors[0].unwind_dir);
  } else {
    digitalWrite(motors[0].dirpin, motors[0].wind_dir);
  }

  if (rightSteps < 0) {
    rightSteps = -rightSteps;
    digitalWrite(motors[1].dirpin, motors[1].unwind_dir);
  } else {
    digitalWrite(motors[1].dirpin, motors[1].wind_dir);
  }

  int leftStepsep = !slow ? motors[0].stepsep / 100 : 200;
  int rightStepsep = !  slow ? motors[1].stepsep / 100 : 200;
  int leftTotal = leftSteps*leftStepsep;
  int rightTotal = rightSteps*rightStepsep;
  
  for(int n = 0; n < max(leftTotal, rightTotal); n++) {
    if (n % leftStepsep == 0 && n < leftTotal) {
      digitalWrite(motors[0].steppin, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(motors[0].steppin, LOW);
    }
    if (n % rightStepsep == 0 && n < rightTotal) {
      digitalWrite(motors[1].steppin, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(motors[1].steppin, LOW);
    }
   
    delayMicroseconds(100);
  } 
}

void dodgeAngle(int angle) {
  digitalWrite(ledPin, HIGH);
  
  // rotate angle frame by 45 degrees
  angle += 45;
  if (angle >= 360) {
    angle -= 360;
  }

  double sintheta = -sin(angle * (PI / 180.0));
  double costheta = -cos(angle * (PI / 180.0));
  
  // calculate motor offsets
  double left = costheta * motors[0].steprange / 2;
  double right = sintheta * motors[1].steprange / 2;

  stepBoth(left, right, false);
//  step(motors[0], left);
//  step(motors[1], right);

  delay(2000);

  stepBoth(-left, -right, true);

  digitalWrite(ledPin, LOW);
//
//  step(motors[0], -left);
//  step(motors[1], -right);
}

void setup()
{
  motors[0].dirpin = 2;
  motors[0].steppin = 4;
  motors[0].wind_dir = WIND;
  motors[0].unwind_dir = UNWIND;
  motors[1].dirpin = 12;
  motors[1].steppin = 13;
  motors[1].wind_dir = UNWIND;
  motors[1].unwind_dir = WIND;

  // motor config
  motors[0].steprange = 120;
  motors[0].stepsep = 2000;
  motors[1].steprange = 260;
  motors[1].stepsep = 1500;
  
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

  // Centre pinata
  stepBoth(motors[0].steprange / 2, motors[1].steprange / 2, true);
//  step(motors[0], motors[0].steprange / 2);
//  step(motors[1], motors[1].steprange / 2);
}

void loop()
{
  while (Serial.available() > 0) {
    int angle = Serial.parseInt();
    Serial.println(angle);

    if (angle > 0 && angle < 360) {
      dodgeAngle(angle);
    }
  }
}
