//---------------------VARIABLES-------------------------------

//-------------------------------------------------------------

//---------------------CONSTANTS-------------------------------
int redLED = 2;
int blueLED = 3;
int greenLED = 4;
//-------------------------------------------------------------

void setup() {
 pinMode(redLED, OUTPUT);
 pinMode(blueLED, OUTPUT);
 pinMode(greenLED, OUTPUT); 

}

void loop() {
  Blink(redLED, 50);
}

void Blink(int LED, int freq){
  analogWrite(LED,255); //255 provides 5V to pin
  delay(freq);
  analogWrite(LED,0); //Turn off LED
  delay(freq);
}

