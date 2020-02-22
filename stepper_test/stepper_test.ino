// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// as posted on Arduino Forum at http://forum.arduino.cc/index.php?topic=208905.0

byte dir1pin = 2;
byte step1pin = 4;
byte dir2pin = 12; 
byte step2pin = 13;

int numberOfSteps = 100;
byte ledPin = LED_BUILTIN;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 10; // milliseconds


void step(int directionPin, int stepPin, int dir, int steps) {
  digitalWrite(ledPin, HIGH);
  
  if (dir == 0) {
    digitalWrite(directionPin, HIGH);
  } else {
    digitalWrite(directionPin, LOW);
  }
  
  for(int n = 0; n < steps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepPin, LOW);
   
    delay(millisbetweenSteps);
  }
 
  digitalWrite(ledPin, LOW);
}


void setup()
{
  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);
 
  delay(2000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // B/c Anna says so
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}

void loop()
{
  while (Serial.available() > 0) {
    char i = Serial.read();
    switch(i) {
      case 'z':
        step(dir1pin, step1pin, 0, numberOfSteps);
        break;
      case 'x':
        step(dir1pin, step1pin, 1, numberOfSteps);
        break;
      case 'c':
        step(dir2pin, step2pin, 0, numberOfSteps);
        break;
      case 'v':
        step(dir2pin, step2pin, 1, numberOfSteps);
        break;
    }
  }
}
