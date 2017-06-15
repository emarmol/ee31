//////////////////CONSTANTS///////////////////
int fwd = 0;
int rev = 1;
int delay_timer = 1350;
int ams_sensor = A4;
int sensor_val = 0;
int blue_threshold = 15;
int red_threshold = 5;
int color_threshold = 5;
int red_LED = 29;
int blue_LED = 30;
int wheel_rl = 7;
int wheel_rh = 6;
int wheel_ll = 3;
int wheel_lh = 4;
int hall_sensor = 52;
//coms
int pinSpeakerModulator = 45, pinRXDigi = 22;
int pinMicRX = 2; int pinSpeakerCarrier = 5;
int errorLEDPin = 47, success200LEDPin = 53, success300LEDPin = 51, success400LEDPin = 48;
int RXCount = 0, messageTolerance = 30;
//state machine declarations
const int color_select = 1;
const int on = 2;
const int hit_wall = 3;
const int find_line = 4;
const int follow_line = 5;
const int magnet = 6;
const int complete = 7;
const int finish = 8;
const int leds = 9;
const int challenge_2 = 10;
const int receive_led = 37;

/*Vitellary Code*/
int front = 31;
int left = 25;
int right = 49;
int back = 28;
const int sensors[] = {31, 25, 49, 28};
//const int leds[] = {44, 42, 40, 38}; //blue, yellow, red, green
int last_pressed[] = {0, 0, 0}; // front, right, left
long debounceDelay = 10;

/////////////////MUSICAL NOTES///////////////
const int B2 = 123;
const int C3 = 131;
const int Csh3 = 139;
const int D3 = 147;
const int Dsh3 = 156;
const int E3 = 165;
const int F3 = 175;
const int Fsh3 = 185;
const int G3 = 196;
const int Gsh3 = 208;
const int A3Note = 220;
const int Ash3 = 233;
const int B3 = 247;
const int C4 = 261;
const int Csh4 = 277;
const int D4 = 293;
const int Dsh4 = 311;
const int E4 = 329;
const int F4 = 349;
const int Fsh4 = 370;
const int G4 = 392;
const int Gsh4 = 415;
const int A4Note = 440;
const int Ash4 = 466;
const int B4 = 493;
const int C5 = 523;
const int Csh5 = 554;
const int D5 = 587;
const int Dsh5 = 622;
const int E5 = 659;
const int F5 = 698;
const int Fsh5 = 740;
const int G5 = 784;
const int Gsh5 = 831;
const int A5Note = 880;
const int Ash5 = 932;
/////////////////////////////////////////////

//////////////////VARIABLES//////////////////
bool end_state = false;
bool mode_forward = false;
bool mode_reverse = false;
bool mode_right = false;
bool mode_left = false;
bool found = false;
bool first_call = false;
bool msg = false; //resend signal or not
int count = 0;
int state = 1;
bool select_blue = false;
bool select_red = false;
bool tx200 = 0, tx300 = 0, tx400 = 0, tx500 = 0;
bool receivedMsg = 0, signal_200 = 0, signal_300 = 0, signal_400 = 0, signal_compBot = 0, receiveMsgError = 0;
bool transmitting = 0;
bool noise_or_pulse = 0;
bool tone_played = false;
float startTime = 0, lastISRTime_slow = 0, lastISRTime_fast = 0;
float downtimeTimer = 0;
/////////////////////////////////////////////
void setup() {
  pinMode(wheel_rh, OUTPUT);
  pinMode(wheel_rl, OUTPUT);
  pinMode(wheel_ll, OUTPUT);
  pinMode(wheel_lh, OUTPUT);
  pinMode(red_LED, OUTPUT);
  pinMode(blue_LED, OUTPUT);
  pinMode(ams_sensor, INPUT);
  pinMode(50, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(receive_led, OUTPUT);
  digitalWrite (red_LED, HIGH);
  digitalWrite (blue_LED, HIGH);
  Serial.begin(9600);
  /*Vitellary*/
  for (int i = 0; i < 3; i++) {
    //pinMode(leds[i], OUTPUT);
    pinMode(sensors[i], INPUT);
  }
  //Serial.begin(9600);
  //coms initialization
  pinMode(pinSpeakerModulator, OUTPUT);
  pinMode(pinMicRX, INPUT);
  pinMode(pinSpeakerCarrier, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pinMicRX), receivedSignal, RISING);
  digitalWrite(pinSpeakerModulator, LOW);//init speaker as off
  //end coms
  //pinMode(13, OUTPUT); // Set this pin as output
  Serial.begin(9600);
  //Serial1.begin(1200);
  TCCR3A = _BV(COM3A0) | _BV(COM3B0) | _BV(WGM30) | _BV(WGM31);
  // sets COM Output Mode to FastPWM with toggle of OC3A on compare match with OCR3A
  // also sets WGM to mode 15: FastPWM with top set by OCR3A
  TCCR3B = _BV(WGM32) | _BV(WGM33) |  _BV(CS31);
  // sets WGM as stated above; sets clock scaling to "divide by 8"
  OCR3A = 53;
  // above sets the counter value at which register resets to 0x0000;
  // generate 18.523 kHz when OCR3A=53 on Mega pin 5
  // Serial.println(TCCR3A, BIN);Serial.println(TCCR3B, BIN);
  state = 1;
}

void loop() {
  ///////////////////////////////////////////////
  switch (state) {
    case color_select:
      color_state();
      break;
    case on:
      on_state();
      break;
    case hit_wall:
      collide_state();
      break;
    case find_line:
      find_state();
      break;
    case follow_line:
      follow_state();
      break;
    case magnet:
      magnet_state();
      break;
    case complete:
      complete_state();
      break;
    case finish:
      finish_state();
      break;
    case leds:
      leds_blinking();
      break;
    case challenge_2:
      challenge2_state();
      break;
    default:
      state = 1;
      break;
  }
  ///////////////////////////////////////////////
  // put your main code here, to run repeatedly:
  //  if(millis() < 2000) {

  //forward();
  // }
  // if(millis() < 3000) {
  //   R_turn();
  // }
  // if(millis() < 4000) {
  //    L_turn();
  /* }
    if(millis() < 5000) {
     reverse();
    }
    if (millis() < 6000) {
         brake();
    }
    else {
     turn_180();
     delay(5000);
    }*/
  //sensor_val = analogRead(ams_sensor);
  //Serial.println(sensor_val);
  /*follow(color_threshold);
    int hall_sensor = digitalRead(52);*/
  //detect_blue();

  // send_signal(200);
  //transmit200();
  //delay(7000);
  // reverse();
}

void color_state() {
  int sensor_value = debounce(digitalRead(sensors[1]), 1);
  Serial.println("color state");
  if (sensor_value == HIGH) { //left
    Serial.println("blue");
    select_blue = true;
    while (!signal_300) {
      brake();
      if (receivedMsg) {
        //digitalWrite(receive_led, HIGH);
        respondToSignal();
        RXCount = 0;
      }
    }
    state++;
  }
  else if (debounce(digitalRead(sensors[2]), 2) == HIGH) { //right
    Serial.println("red");
    select_red = true;
    state++;
  }
  else if (debounce(digitalRead(sensors[0]), 0) == HIGH) { //front
    Serial.println("red");
    state = challenge_2;
  }
}

void on_state() {
  delay(300);
  signal_200 = false;
  signal_300 = false;
  signal_400 = false;
  tone_go();
  state++;
  Serial.println("on");
}

void collide_state() {
  Serial.println("collide_state");
  for (int i = 0; i < 3; i++) {
    int sensor_value = debounce(digitalRead(sensors[i]), i);
    if (sensor_value == HIGH) {
      if (sensors[i] == front) {
        float start_time = millis();
        while (millis() - start_time < 200) {
          reverse();
        }
      }
      if (select_red) {
        float start_time = millis();
        while (millis() - start_time < 800) {
          L_turn();
          color_threshold = red_threshold;
        }
        state++;
      }
      else if (select_blue) {
        float start_time = millis();
        while (millis() - start_time < 700) {
          R_turn();
          color_threshold = blue_threshold;
        }
        state++;
      }
    }
  }
  if (select_blue) {
    forward_blue();
  }
  else
    forward();
}

void find_state() {
  Serial.println("find");
  forward();
  sensor_val = analogRead(ams_sensor);
  if ( sensor_val >= color_threshold) {
    float start_time = millis();
    while (millis() - start_time < 5) {
      forward();
    }
    if (select_blue) {
      digitalWrite(receive_led, HIGH);
      start_time = millis();
      while (millis() - start_time < 10) {
        L_turn();
      }
      tone_blue();
    }
    else if (select_red) {
      digitalWrite(8, HIGH);
      start_time = millis();
      while (millis() - start_time < 10) {
        R_turn();
      }
      tone_red();
    }
    forward();
    state++;
  }
}

void follow_state() {
  Serial.println("follow");
  //digitalWrite(receive_led, LOW);
  follow(color_threshold);
  if (digitalRead(52)) //reason for not using hall_sensor constant?
    state++;
}

void magnet_state() {
  Serial.println("magnet");
  brake();
  tone_magnet();
  if (select_red) {
    digitalWrite(8, LOW);
    delay(200);
    digitalWrite(8, HIGH);
  }
  else {
    digitalWrite(receive_led, LOW);
    delay(200);
    digitalWrite(receive_led, HIGH);
  }
  transmit200();
  if (receivedMsg) {
    //digitalWrite(receive_led, HIGH);
    respondToSignal();
    RXCount = 0;
  }
  while (!signal_200) {
    brake();
    if (receivedMsg) {
      //digitalWrite(receive_led, HIGH);
      respondToSignal();
      RXCount = 0;
    }
  }
  //digitalWrite(receive_led, LOW);
  state++;
}

void complete_state() {
  Serial.println("complete");
  follow(color_threshold);
  //  if (digitalRead(sensors[0]) == HIGH) {
  //    digitalWrite(8, HIGH);
  //    brake();
  //  }
}


void finish_state() {
  if (!tone_played) {
    tone_end();
    tone_played = true;
  }
  brake();
  if (select_red) {
    transmit300();
    select_red = false;
  }
  else if (select_blue) {
    float start_time = millis();
    while (millis() - start_time < 500) {
      reverse();
    }
    brake();
    delay(100);
    transmit400();
    select_blue = false;
    state++;
  }
  if (receivedMsg) {
    respondToSignal();
    RXCount = 0;
  }
  if (signal_400) {
    state++;
  }
}

void leds_blinking () {
  for (int i = 0; i < 11; i++) {
    digitalWrite(8, HIGH);
    digitalWrite(receive_led, HIGH);
    delay(200);
    digitalWrite(8, LOW);
    digitalWrite(receive_led, LOW);
    delay(200);
  }
  while (1) {
    brake();
  }
}

void challenge2_state() {
  digitalWrite(red_LED, LOW);
  digitalWrite(blue_LED, LOW);
  if (receivedMsg) {
    respondToSignal();
    RXCount = 0;
  }
  delay(500);
  if (signal_300 || debounce(digitalRead(sensors[0]), 0)) {
    float start_time = millis();
    while (millis() - start_time < 2100) { //forward 1 foot
      forward2();
    }
    start_time = millis();
    while (millis() - start_time < 500) { // brake
      brake();
    }
    turn_180();
    start_time = millis();
    while (millis() - start_time < 500) { // brake
      brake();
    }
    start_time = millis();
    while (millis() - start_time < 500) { // backwards 3 inches
      reverse();
    }
    start_time = millis();
    while (millis() - start_time < 500) { // brake
      brake();
    }
    start_time = millis();
    while (millis() - start_time < 750) { // turn left
      L_turn();
    }
    for (int i = 0; i < 3; i++) {
      brake();
      delay(300);
      start_time = millis();
      while (millis() - start_time < 620) { // turn right 3 x
        R_turn();
      }
    }
    start_time = millis();
    while (millis() - start_time < 2300) {
      reverse();
    }
    brake();
    if (!signal_300)
      transmit300();
    while (1) {
      brake();
    }
  }
}

void pause(int time) {
  digitalWrite(pinSpeakerModulator, LOW);
  delay(time);
  digitalWrite(pinSpeakerModulator, HIGH);
}

void note(int freq, int duration) {
  OCR3A = (981719 / freq);
  delay(duration);
  pause(5);
}

void noteLegato(int freq, int duration) {
  OCR3A = (981719 / freq);
  delay(duration);
}

float findTempo(float bpm) {
  return ((1 / (bpm / 60)) * 4000);
}

void tone_go() {
  float tempo = findTempo(81);
  float W = tempo;
  float H = tempo / 2;
  float Q = tempo / 4;
  float E = tempo / 8;
  float Sx = tempo / 16;
  brake();
  digitalWrite(pinSpeakerModulator, HIGH);
  note(G4, Q);
  note(Fsh4, Q);
  note(E4, Q);
  note(D4, Q);
  note(E4, E);
  note(E4, Q);
  digitalWrite(pinSpeakerModulator, LOW);
  OCR3A = 53;
}

void tone_red() {
  float tempo = findTempo(112);
  float W = tempo;
  float H = tempo / 2;
  float Q = tempo / 4;
  float E = tempo / 8;
  float Sx = tempo / 16;
  brake();
  digitalWrite(pinSpeakerModulator, HIGH);
  note(F4, Q);
  note(G4, Q);
  note(A4Note, E);
  note(G4, Q);
  note(A4Note, E);
  Serial.println(Ash4);
  note(Ash4, Q);
  note(A4Note, Q);
  note(G4, E);
  note(F4, Q);
  note(G4, E);
  note(A4Note, Q);
  note(G4, Q);
  note(F4, Q);
  note(A4Note, Q);
  note(G4, W);
  digitalWrite(pinSpeakerModulator, LOW);
  OCR3A = 53;
}

void tone_blue() {
  float tempo = findTempo(124);
  float W = tempo;
  float H = tempo / 2;
  float Q = tempo / 4;
  float E = tempo / 8;
  float Sx = tempo / 16;
  float curTime = millis();
  brake();
  digitalWrite(pinSpeakerModulator, HIGH);
  noteLegato(E3, Q);
  note(E3, E);
  note(E3, E);
  noteLegato(G3, E);
  note(G3, Sx);
  noteLegato(E3, Sx);
  note(E3, E);
  note(D3, E);
  note(C3, H);
  note(B2, H);
  digitalWrite(pinSpeakerModulator, LOW);
  OCR3A = 53;
}

void tone_magnet() {
  float curTime = millis();
  brake();
  OCR3A = 1000;
  for (int i = 0; i < 5; i++) {
    digitalWrite(pinSpeakerModulator, HIGH);
    delay(250);
    digitalWrite(pinSpeakerModulator, LOW);
    delay(250);
  }
  digitalWrite(pinSpeakerModulator, LOW);
  OCR3A = 53;
}

void tone_end() {
  float curTime = millis();
  brake();
  digitalWrite(pinSpeakerModulator, HIGH);
  while (millis() - curTime < 400) {
    OCR3A = 1680;
  }
  while (millis() - curTime < 600) {
    OCR3A = 1540;
  }
  while (millis() - curTime < 700) {
    OCR3A = 1660;
  }
  while (millis() - curTime < 800) {
    OCR3A = 1580;
  }
  while (millis() - curTime < 1000) {
    OCR3A = 1600;
  }
  digitalWrite(pinSpeakerModulator, LOW);
  OCR3A = 53;
}

void forward() {
  mode_right = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_forward) {
    delay(1);
    mode_forward = true;
  }
  analogWrite(wheel_lh, 76);
  analogWrite(wheel_ll, 0);
  analogWrite(wheel_rl, 68);
  analogWrite(wheel_rh, 0);
}


void forward2() {
  mode_right = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_forward) {
    delay(1);
    mode_forward = true;
  }
  analogWrite(wheel_lh, 75);
  analogWrite(wheel_ll, 0);
  analogWrite(wheel_rl, 68);
  analogWrite(wheel_rh, 0);
}

void forward_blue() {
  mode_right = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_forward) {
    delay(1);
    mode_forward = true;
  }
  analogWrite(wheel_lh, 68);
  analogWrite(wheel_ll, 0);
  analogWrite(wheel_rl, 64);
  analogWrite(wheel_rh, 0);
}

void reverse() {
  mode_forward = false;
  mode_right = false;
  mode_left = false;
  if (!mode_reverse) {
    delay(1);
    mode_reverse = true;
  }
  analogWrite(wheel_lh, 0);
  analogWrite(wheel_ll, 34);
  analogWrite(wheel_rl, 0);
  analogWrite(wheel_rh, 73);
}

void L_turn () {
  mode_forward = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_right) {
    delay(1);
    mode_right = true;
  }
  analogWrite(wheel_rl, 50);
  analogWrite(wheel_rh, 0);
  analogWrite(wheel_ll, 32);
  analogWrite(wheel_lh, 0);
}

void R_turn () {
  mode_forward = false;
  mode_right = false;
  mode_reverse = false;
  if (!mode_left) {
    delay(1);
    mode_left = true;
  }
  analogWrite(wheel_lh, 100);
  analogWrite(wheel_ll, 0);
  analogWrite(wheel_rh, 60);
  analogWrite(wheel_rl, 0);

}

void turn_180() {
  //if 180 turn is left, need to change modes
  mode_forward = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_right) {
    delay(1);
    mode_right = true;
  }
  float start_time = millis();
  while (millis() - start_time < delay_timer) {
    analogWrite(wheel_rl, 50);
    analogWrite(wheel_rh, 0);
    analogWrite(wheel_ll, 32);
    analogWrite(wheel_lh, 0);
  }
  brake();
}

void brake() {
  analogWrite(wheel_rl, 0);
  analogWrite(wheel_rh, 0);
  analogWrite(wheel_ll, 0);
  analogWrite(wheel_lh, 0);
}

int ams_read() {
  return analogRead(ams_sensor);
}

void detect_blue() {
  if ( ams_read() > blue_threshold) {
    brake();
  }
  else {
    forward();
  }
}

void follow(int threshold) {
  found = false;
  //collision detection
  for (int i = 0; i < 3; i++) {
    int sensor_value = debounce(digitalRead(sensors[i]), i);
    if (sensor_value == HIGH) {
      if (sensors[i] == front || sensors[i] == left || sensors[i] == right) {
        last_pressed[i] = millis();
        if (sensors[i] == front) {
          if (abs(last_pressed[0] - last_pressed[1]) <= 500) {
            front_right_collide();
          }
          else if (abs(last_pressed[0] - last_pressed[2]) <= 500)
            front_left_collide();
          else state = finish;
        }
        if (sensors[i] == right) {
          if (abs(last_pressed[0] - last_pressed[1]) <= 500) {
            front_right_collide();
          }
          else right_collide();
        }
        if (sensors[i] == left) {
          if (abs(last_pressed[0] - last_pressed[2]) <= 500) {
            front_left_collide();
          }
          else left_collide();
        }
      }
      else {
        back_collide();
      }
    }
  }
  //end collision detection
  if (ams_read() > threshold) {
    forward();
    found = true;
  }
  else {
    float curTime = millis();
    while (millis() < curTime + 250) {
      L_turn();
      if (ams_read() > threshold) {
        count = 0;
        found = true;
        curTime = millis();
        while (millis() - curTime < 50) {
          forward();
        }
        break;
      }
    }
    if (!found) {
      curTime = millis();
      while (millis() < curTime + 600) {
        R_turn();
        if (ams_read() > threshold) {
          count = 0;
          found = true;
          curTime = millis();
          while (millis() - curTime < 50) {
            forward();
          }
          break;
        }
      }
    }
  }
}

void detect_color() {
  bool found_blue = false; //for future use
  if (!found) {
    forward();
  }
  if (ams_read() > color_threshold) {
    if (!found) {
      float curr_time = millis();
      while (millis() - curr_time < 50) {
        forward();
      }
      brake();
    }
    digitalWrite (red_LED, LOW);
    if (ams_read() > blue_threshold) {
      float curr_time = millis();
      found_blue = true;
      while (millis() - curr_time < 500) {
        L_turn();
      }
      while (1) {
        brake();
      }
    }
    else {
      float curr_time = millis();
      digitalWrite(red_LED, HIGH);
      digitalWrite(blue_LED, LOW);
      found_blue = false;
      while (millis() - curr_time < 500) {
        R_turn();
      }
      while (1) {
        brake();
      }
    }
    found = true;
  }
}

void right_collide() {
  float start_time = millis();
  while (millis() - start_time < 500) {
    L_turn();
  }
  forward();
}

void left_collide() {
  float start_time = millis();
  while (millis() - start_time < 500) {
    R_turn();
  }
  forward();
}

void front_right_collide() {
  float start_time = millis();
  while (millis() - start_time < 250) {
    reverse();
  }
  start_time = millis();
  while (millis() - start_time < 250) {
    L_turn();
  }
  forward();
}

void front_left_collide () {
  float start_time = millis();
  while (millis() - start_time < 250) {
    reverse();
  }
  start_time = millis();
  while (millis() - start_time < 250) {
    L_turn();
  }
  forward();
}

void front_collide() {
  float start_time = millis();
  while (1) {
    brake();
  }
  /*while (millis() - start_time < 250) {
    reverse();
    }
    turn_180();*/
}

void back_collide() {
  //forward, call line follow
  forward();
}

int debounce(int sensor_value, int i) {
  // Requires the sensor reading to remain constant for 50ms
  boolean not_same = true;
  int current_sensor_val = sensor_value;
  int new_sensor_val;

  while (not_same) {
    float start_time = millis();
    while (millis() - start_time < debounceDelay) {
      break;
    }
    new_sensor_val = digitalRead(sensors[i]);
    if (new_sensor_val != current_sensor_val) {
      current_sensor_val = new_sensor_val;
    } else {
      not_same = false;
    }
  }
  return current_sensor_val;
}

//ISR for RX detected signal
void receivedSignal() {
  if (transmitting || receivedMsg) { // ignore if we're already processing message or if we're transmitting
    noise_or_pulse = 1;
    lastISRTime_fast = millis();
    return;
  }
  if ((millis() - lastISRTime_slow) > 1600) {
    if ((millis() - lastISRTime_fast) > 80) {
      lastISRTime_slow = millis();
      lastISRTime_fast = millis();
      receivedMsg = 1;
      return;
    }
    lastISRTime_fast = millis();
  }
}

void transmit200() {
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

void transmit300() {
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

void transmit400() {
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

void transmit500() {
  // cli();
  transmitting = 1;
  digitalWrite(pinSpeakerModulator, HIGH);
  delay(500);
  digitalWrite(pinSpeakerModulator, LOW);
  tx500 = 0;
  transmitting = 0;
  //sei();
}

void respondToSignal() {
  int downtimeCounter = 0;
  for (int i = 0; i < 100; i++) {

    //if we receive ISR during this delay loop, check to see how long it was on for
    if (noise_or_pulse) {
      if (digitalRead(pinRXDigi)) {
        downtimeTimer = millis();
        while (digitalRead(pinRXDigi));
        if ((millis() - downtimeTimer) > 150) {
          //digitalWrite(pinYellowLED, HIGH);
          delay(1000);
          //digitalWrite(pinYellowLED, LOW);
          goto waitingForPulse;
        }
      }
      noise_or_pulse = 0;
    }
    delay(10);
  }
  //digitalWrite(pinBlueLED, LOW);


  //then wait until a new signal comes, and measure how long it is
waitingForPulse:
  receiveMsgError = 0;
  downtimeCounter++;
  //Serial.println(downtimeCounter);
  startTime = millis();
  while (!digitalRead(pinRXDigi)) {
    if ((millis() - startTime) > 500) { //0.5 second timeout for listening
      receiveMsgError = 1;//timeout occured
      break;
    }
  }//spin until we get the next signal

  //count how long this signal is
  if (!receiveMsgError) {
    delay(5);
    float startMeasuring = millis();
    while (digitalRead(pinRXDigi));
    RXCount = (millis() - startMeasuring);
    if (downtimeCounter > 5)
    {
      receivedMsg = 0;
      return;
    }
    if (RXCount < 100) goto waitingForPulse; //probably spike before pulse
    Serial.println(RXCount);

    //200ms received
    if ((RXCount < (200 + messageTolerance)) && (RXCount > (200 - messageTolerance))) signal_200 = 1;
    //300ms received
    else if ((RXCount < (300 + messageTolerance)) && (RXCount > (300 - messageTolerance))) signal_300 = 1;
    //400ms received
    else if ((RXCount < (400 + messageTolerance)) && (RXCount > (400 - messageTolerance))) signal_400 = 1;
    //500ms received (companion bot)
    else if ((RXCount < (500 + messageTolerance)) && (RXCount > (500 - messageTolerance))) signal_compBot = 1;
    else receiveMsgError = 1;//error if we don't detect signal
  }
  //then accordingly set the proper boolean value to true, and reset the receivedMsg to 0
  RXCount = 0;
  receivedMsg = 0;
  digitalWrite(8, LOW);
}

/* //Vitellary
  for (int i = 0; i < 4; i++) {
    int sensor_value = debounce(digitalRead(sensors[i]), i);
    if (sensor_value == HIGH) {
      if (sensors[i] == front || sensors[i] == left || sensors[i] == right) {
        last_pressed[i] = millis();
        if (sensors[i] == front) {
          if (abs(last_pressed[0] - last_pressed[1]) <= 500) {
            front_right_collide();
          }
          else if (abs(last_pressed[0] - last_pressed[2]) <= 500)
            front_left_collide();
          else front_collide();
        }
        if (sensors[i] == right) {
          if (abs(last_pressed[0] - last_pressed[1]) <= 500) {
            front_right_collide();
          }
          else right_collide();
        }
        if (sensors[i] == left) {
          if (abs(last_pressed[0] - last_pressed[2]) <= 500) {
            front_left_collide();
          }
          else left_collide();
        }
      }
      else {
        back_collide();
      }
    } else {
      forward();
    }
  }
  //End Vitellary*/
