//#include "TimerOne.h"

//motorA = right motor
//motorB = left motor

// -------------------pin declarations --------------------
int pin1MotorA = 4, pin2MotorA = 8, pin1MotorB = 6, pin2MotorB = 7;
int blueRightPin = 23, blueLeftPin = 25, redRightPin = 22, redLeftPin = 24;
int photoDetectorPIN = A0, hallSensor = A1;
int pinSpeakerModulator = 45, pinRXDigi = 44;
int pinMicRX = 2, pinSpeakerCarrier = 5;
int errorLEDPin = 47, success200LEDPin = 53, success300LEDPin = 51, success400LEDPin = 49;

//---------------------------------------------------------
//---------------boolean variable declaration--------------
bool motorAFWD = 0;
bool motorBFWD = 0;
bool motorAStopped = 0, motorBStopped = 0;
bool stopMotors = 0, goFWD = 0, goBCK = 0, turnL = 0, turnR = 0, turn180 = 0, blueFinishLine = 0;
bool detectingB = 0;
bool followBlue = 0, followRed = 0;
bool tryLeft = 0;
bool tx200 = 0, tx300 = 0, tx400 = 0, tx500 = 0;
bool receivedMsg = 0, signal_200 = 0, signal_300 = 0, signal_400 = 0, signal_compBot = 0, receiveMsgError = 0;
bool transmitting = 0;
//---------------------------------------------------------
//------------generic variable declaration ----------------
int MotorASpeed = 100, MotorBSpeed = 100;
int serialPacket = 0;
float calibrateA = 1.3,  calibrateB = 1;
int blueTHRESH = 500, redTHRESH = 200, magneticTHRESH = 600;
int leftLightMeasurement = 600, rightLightMeasurement = 600;
int magneticMeasurement = 0;
int RXCount = 0, messageTolerance = 30;
float startTime = 0, lastISRTime_slow = 0, lastISRTime_fast = 0;
//---------------------------------------------------------


void setup() {
  //init pins
  Serial.begin(9600);
  while(!Serial);
  MotorASpeed = 100*calibrateA;
  MotorBSpeed = 100*calibrateB;
  pinMode(pin2MotorA, OUTPUT);
  pinMode(pin2MotorB, OUTPUT);
  pinMode(pin1MotorA, OUTPUT);
  pinMode(pin1MotorB, OUTPUT);
  pinMode(blueRightPin, OUTPUT);
  pinMode(blueLeftPin, OUTPUT);
  pinMode(redRightPin, OUTPUT);
  pinMode(redLeftPin, OUTPUT);
  pinMode(photoDetectorPIN, INPUT);
  pinMode(pinSpeakerModulator, OUTPUT);
  pinMode(pinMicRX, INPUT);
  pinMode(pinSpeakerCarrier, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pinMicRX), receivedSignal, RISING);

  digitalWrite(pinSpeakerModulator, LOW);//init speaker as off
  digitalWrite(blueRightPin, HIGH);
  digitalWrite(blueLeftPin, LOW);
  digitalWrite(redRightPin, HIGH);
  digitalWrite(redLeftPin, LOW);
  cli();
  //set timer1 interrupt at 10Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1562;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  //TIMER 2 INITIALIZATION!!
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 100;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


  //TIMER 3 INITIALIZATION!!
  TCCR3A = _BV(COM3A0) | _BV(COM3B0) | _BV(WGM30) | _BV(WGM31);
  // sets COM Output Mode to FastPWM with toggle of OC3A on compare match with OCR3A
  // also sets WGM to mode 15: FastPWM with top set by OCR3A
  TCCR3B = _BV(WGM32) | _BV(WGM33) |  _BV(CS31);
  // sets WGM as stated above; sets clock scaling to "divide by 8"
  OCR3A = 53;
  sei();
}

ISR(TIMER2_COMPA_vect){  
  if(Serial.available())
  {
    serialPacket = (char)Serial.read();
    
    //packet is received in ascii format
    switch(serialPacket){
      case 48://forward
        goFWD = 1;
        break;
      case 49://backwards
        goBCK = 1;
        break;
      case 50://left
        turnL = 1;
        break;
      case 51://right
        turnR = 1;
        break;
      case 52://180 degree turn
        turn180 = 1;
        break;
      case 53://stop motors
        stopMotors = 1;
        break;    
      case 54://6
        blueFinishLine = 1;
        break;
      case 55://7
        followBlue = 1;
        break;
      case 56://8
        followRed = 1;
        break;
      case 97://a
        tx200 = 1;
        break;
      case 98://b
        tx300 = 1;
        break;
      case 99://c
        tx400 = 1;
        break;
    }
  }
}

ISR(TIMER1_COMPA_vect){
 if(followBlue)
   {
        magneticMeasurement = analogRead(hallSensor);
        Serial.println(magneticMeasurement);
        
        //make sure to turn off red leds first
        digitalWrite(redRightPin, LOW);
        digitalWrite(redLeftPin, LOW);
        
        if(digitalRead(blueLeftPin))
          leftLightMeasurement = analogRead(photoDetectorPIN);
        else
          rightLightMeasurement = analogRead(photoDetectorPIN);

        digitalWrite(blueRightPin, !digitalRead(blueRightPin));//toggle blue LEDs for next ISR to read
        digitalWrite(blueLeftPin, !digitalRead(blueLeftPin));
   }
   else if(followRed)
   {
        digitalWrite(blueRightPin, LOW);//ensure both blue leds are OFF!!!
        digitalWrite(blueLeftPin, LOW);
        if(digitalRead(redLeftPin))
          leftLightMeasurement = analogRead(photoDetectorPIN);
        else
          rightLightMeasurement = analogRead(photoDetectorPIN);

        
        digitalWrite(redRightPin, !digitalRead(redRightPin));//toggle bred LEDs for next ISR to read
        digitalWrite(redLeftPin, !digitalRead(redLeftPin));
   }
}
//ISR for RX detected signal
void receivedSignal(){
  if (transmitting || receivedMsg) { // ignore if we're already processing message or if we're transmitting
    lastISRTime_fast = millis();
    return;
  }
  if((millis()-lastISRTime_slow) > 1600){
    if((millis()-lastISRTime_fast) > 80){
      lastISRTime_slow = millis();
      lastISRTime_fast = millis();
      receivedMsg = 1;
      return;
    }
    lastISRTime_fast = millis();
  }
}

void changeDirA(){
    if(motorAFWD){
      digitalWrite(pin2MotorA, LOW);
      delay(10);
      analogWrite(pin1MotorA, MotorASpeed);
      motorAFWD = 0;
    }
    else
    {
      digitalWrite(pin1MotorA, LOW);
      delay(10);
      analogWrite(pin2MotorA, MotorASpeed); 
      motorAFWD = 1;     
    }
}

void changeDirB(){
    if(motorBFWD){
      digitalWrite(pin2MotorB, LOW);
      delay(10);
      analogWrite(pin1MotorB, MotorBSpeed);
      motorBFWD = 0;
    }
    else
    {
      digitalWrite(pin1MotorB, LOW);
      delay(10);
      analogWrite(pin2MotorB, MotorBSpeed); 
      motorBFWD = 1;     
    }
}


void stopMotorA()
{
  digitalWrite(pin1MotorA, LOW);
  digitalWrite(pin2MotorA, LOW);
  motorAStopped = 1;
}


void stopMotorB()
{
  digitalWrite(pin1MotorB, LOW);
  digitalWrite(pin2MotorB, LOW);
  motorBStopped = 1;
}

void movebackwards(){
  if(!motorAFWD)
    changeDirA();
  else
    {
      digitalWrite(pin1MotorA, LOW);
      delay(10);
      analogWrite(pin2MotorA, MotorASpeed);
    }
  if(!motorBFWD)
    changeDirB();
  else
  {
      digitalWrite(pin1MotorB, LOW);
      delay(10);
      analogWrite(pin2MotorB, MotorBSpeed); 
  }
}

void moveforward(){
  if(motorAFWD)
    changeDirA();
  else
    {
      digitalWrite(pin2MotorA, LOW);
      delay(10);
      analogWrite(pin1MotorA, MotorASpeed);
    }
  if(motorBFWD)
    changeDirB();
  else
  {
      digitalWrite(pin2MotorB, LOW);
      delay(10);
      analogWrite(pin1MotorB, MotorBSpeed); 
  }
}

//turn left 90 deg
void turnLeft(){
  moveforward();//move forwards, and turn off motorB
  changeDirB(); //pivot
  delay(150);
  stopMotors = 1;
}

void detectBlueStrip(){
  
  while(analogRead(photoDetectorPIN) < blueTHRESH){
    Serial.println(analogRead(photoDetectorPIN));
  }
  
  stopMotors = 1;//stop motors if we break out of the loop
}

void followBlueLine(){
  Serial.println(analogRead(photoDetectorPIN));
  moveforward();
  Serial.println(analogRead(photoDetectorPIN));
  while((leftLightMeasurement > blueTHRESH) && (rightLightMeasurement > blueTHRESH)){
      delay(1);
      if(magneticMeasurement < magneticTHRESH){
        stopMotors = 1;
        followBlue = 0;
        return;
      }
    }
  if((leftLightMeasurement<blueTHRESH)&&(rightLightMeasurement<blueTHRESH)){
    //we're at end of the line!
    stopMotors = 1;
    followBlue = 0;
    return;
  }
  
  if(leftLightMeasurement < blueTHRESH)
    tryLeft = 0;//if we've gone off the left of the track, we want to try moving right first
  else
    tryLeft = 1;
  
  
  if(!tryLeft)
   turnRight();
  else
    turnLeft();

}


void followRedLine(){
  moveforward();
  while((leftLightMeasurement > redTHRESH) && (rightLightMeasurement > redTHRESH))
  {
    delay(1);
  }
  if((leftLightMeasurement<redTHRESH)&&(rightLightMeasurement<redTHRESH)){
    //at end of red line!
    stopMotors=1;
    followRed=0;
    return;
  }
  if(leftLightMeasurement < redTHRESH)
    tryLeft = 0;//if we've gone off the left of the track, we want to try moving right first
  else
    tryLeft = 1;
  
  
  if(!tryLeft)
   turnRight();
  else
    turnLeft();

}

//turn right 90 deg
void turnRight(){
  moveforward();
  changeDirA();
  delay(150);
  stopMotors = 1;
}

void transmit200(){
 // cli();
  transmitting = 1;
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(200);//might need to be lower than this in practice
  digitalWrite(pinSpeakerModulator, LOW);
  delay(1000);
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(200);//might need to be lower than this in practice
  digitalWrite(pinSpeakerModulator, LOW);
  tx200 = 0;
  transmitting = 0;
 // sei();
}

void transmit300(){
 // cli();
  transmitting = 1;
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(300);
  digitalWrite(pinSpeakerModulator, LOW);
  delay(1000);
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(300);
  digitalWrite(pinSpeakerModulator, LOW);
  tx300 = 0;
  transmitting = 0;
  //sei();
}

void transmit400(){
 // cli();
 transmitting = 1;
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(400);
  digitalWrite(pinSpeakerModulator, LOW);
  delay(1000);
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(400);
  digitalWrite(pinSpeakerModulator, LOW);
  tx400 = 0;
  transmitting = 0;
 // sei();
}
void transmit500(){
 // cli();
 transmitting = 1;
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(500);
  digitalWrite(pinSpeakerModulator, LOW);
  tx500 = 0;
  transmitting = 0;
  //sei();
}

void respondToSignal(){
  int downtimeCounter = 0;
  delay(1200);
  //transmit500();//length shouldn't matter here (we think)
  //then wait until a new signal comes, and measure how long it is
  waitingForPulse:
  downtimeCounter++;
  Serial.println(downtimeCounter);
  startTime = millis();
  while(!digitalRead(pinMicRX)){
    if((millis()-startTime) > 500) {//0.5 second timeout for listening
      receiveMsgError = 1;//timeout occured
      break;
    }
  }//spin until we get the next signal
  
  //count how long this signal is
  if(!receiveMsgError){
    float startMeasuring = millis();
    while(digitalRead(pinMicRX));
    RXCount = (millis() - startMeasuring);
    if(downtimeCounter>5)
    {
      receivedMsg = 0;
      return;
    }
    if(RXCount < 100) goto waitingForPulse;//probably spike before pulse
    Serial.println(RXCount);

    //200ms received
    if((RXCount < (200 + messageTolerance)) && (RXCount > (200 - messageTolerance))) signal_200 = 1;
    //300ms received
    else if((RXCount < (300 + messageTolerance)) && (RXCount > (300 - messageTolerance))) signal_300 = 1;
    //400ms received
    else if((RXCount < (400 + messageTolerance)) && (RXCount > (400 - messageTolerance))) signal_400 = 1;
    //500ms received (companion bot)
    else if((RXCount < (500 + messageTolerance)) && (RXCount > (500 - messageTolerance))) signal_compBot = 1;
    else receiveMsgError = 1;//error if we don't detect signal
  }
  //then accordingly set the proper boolean value to true, and reset the receivedMsg to 0
  receivedMsg = 0;
}

void pulseLED(int pinNumber){
  digitalWrite(pinNumber, HIGH);
  delay(1000);
  digitalWrite(pinNumber, LOW);
}

/*
      case 0://forward
      case 1://backwards
      case 2://left
      case 3://right
      case 4://180 degree turn
      case 5://stop
 */
void loop() {
//  Serial.println(leftLightMeasurement);
 // Serial.println(rightLightMeasurement);
  if(stopMotors){
    stopMotorA();
    stopMotorB();
    stopMotors = 0;
  }
  else if(goFWD)
  {
    moveforward();
    goFWD = 0;
  }
  else if(goBCK)
  {
    movebackwards();
    goBCK = 0;
  }
  else if(turnL)
  {
    turnLeft();
    turnL = 0;
  }
  else if(turnR)
  {
    turnRight();
    turnR = 0;
  }
  else if(turn180)
  {
    turnRight();
    turnRight();
    turn180 = 0;
  }
  else if(blueFinishLine)
  {
    moveforward();
    detectBlueStrip();
    blueFinishLine = 0;
  }
  else if(followBlue)
  {
    //redundancy to ensure we toggle within the ISR
    if(digitalRead(blueRightPin) && digitalRead(blueLeftPin))
      digitalWrite(blueRightPin, LOW);
      else if(!digitalRead(blueRightPin) && !digitalRead(blueLeftPin))
      digitalWrite(blueRightPin, HIGH);
      
    followBlueLine();    
  }
  else if(followRed) {
    //redundancy to ensure we toggle within the ISR
    if(digitalRead(redRightPin) && digitalRead(redLeftPin))
      digitalWrite(redRightPin, LOW);
      else if(!digitalRead(redRightPin) && !digitalRead(redLeftPin))
      digitalWrite(redRightPin, HIGH);
      
    followRedLine();
  }
  else if(tx200) transmit200();
  else if(tx300) transmit300();
  else if(tx400) transmit400();
  else if (receivedMsg){
    respondToSignal();
    Serial.println("ayyoo");
    RXCount = 0;
  }
  else if(signal_200)
  {
    pulseLED(success200LEDPin);
    signal_200 = 0;
  }
  else if(signal_300)
  {
    pulseLED(success300LEDPin);
    signal_300 = 0;
  }
  else if(signal_400)
  {
    pulseLED(success400LEDPin);
    signal_400 = 0; 
  }
  else if(signal_compBot)
  {
    
  }
  else if(receiveMsgError)
  {
    pulseLED(errorLEDPin);
    receiveMsgError = 0;
  }
}
