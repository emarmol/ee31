int redLED = 2;
int blueLED = 3;
int greenLED = 4;
int frontSensor = 5;
int leftSensor = 6;
int rightSensor = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode (redLED, OUTPUT);
  pinMode (blueLED, OUTPUT);
  pinMode (greenLED, OUTPUT);
  pinMode (frontSensor, INPUT);
  pinMode (leftSensor, INPUT);
  pinMode (rightSensor, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  bumper_hit();
}

void bumper_hit () {
  if (frontSensor) {
    digitalWrite(redLED, HIGH);
    digitalWrite(blueLED, LOW);
    digitalWrite(greenLED, LOW);
  }
  else if (leftSensor) {
    digitalWrite(redLED, LOW);
    digitalWrite(blueLED, HIGH);
    digitalWrite(greenLED, LOW);
  }  
  else if (rightSensor) {
    digitalWrite(redLED, LOW);
    digitalWrite(blueLED, LOW);
    digitalWrite(greenLED, HIGH);
  }
}
