  //////////////////CONSTANTS///////////////////
int fwd = 0;
int rev = 1;
int delay_timer = 500;
int ams_sensor = A0;
int sensor_val = 0;
int blue_threshold = 30;
int color_threshold = 15;
int red_LED = 29;
int blue_LED = 30;
int wheel_rl = 6;
int wheel_rh = 7;
int wheel_ll = 3;
int wheel_lh = 4;
int hall_sensor = 52;
//coms
int pinMicRX = 2;
int carrier_freq = 5;
int errorLEDPin = 47, success200LEDPin = 53, success300LEDPin = 51, success400LEDPin = 49;
int RXCount = 0, messageTolerance = 30;
//state machine declarations
const int on = 1; 
const int hit_wall = 2; 
const int find_line = 3;
const int follow_state = 4;
const int magnet = 5;
const int complete = 6;

/*Vitellary Code*/
int front = 22;
int left = 24;
int right = 26;
int back = 28;
const int sensors[] = {22, 24, 26, 28};
//const int leds[] = {44, 42, 40, 38}; //blue, yellow, red, green
int last_pressed[] = {0, 0, 0}; // front, right, left
long debounceDelay = 10;
/////////////////////////////////////////////

//////////////////VARIABLES//////////////////
bool mode_forward = false;
bool mode_reverse = false;
bool mode_right = false;
bool mode_left = false;
bool found = false;
bool first_call = false;
bool msg = false; //resend signal or not
int count = 0;
int state = 1;
bool tx200 = 0, tx300 = 0, tx400 = 0, tx500 = 0;
bool receivedMsg = 0, signal_200 = 0, signal_300 = 0, signal_400 = 0, signal_compBot = 0, receiveMsgError = 0;
bool transmitting = 0;
float startTime = 0, lastISRTime_slow = 0, lastISRTime_fast = 0;
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
  digitalWrite (red_LED, HIGH);
  digitalWrite (blue_LED, HIGH);
  Serial.begin(9600);
  /*Vitellary*/
  for (int i = 0; i < 4; i++) {
    //pinMode(leds[i], OUTPUT);
    pinMode(sensors[i], INPUT);
  }
  //Serial.begin(9600);
  // attachInterrupt(digitalPinToInterrupt(com_intr), com_check, RISING);
  //pinMode(13, OUTPUT); // Set this pin as output
  pinMode(carrier_freq, OUTPUT);//pinMode(2, OUTPUT);
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
}

void loop() {
///////////////////////////////////////////////
/*(switch (state) {
    case on:
      //do something when var equals 1
      break;
    case hit_wall:
      //do something when var equals 2
      break;
    case find_line:
      //
      break;
    case follow_state:
      //
      break;
    case magnet:
      //
      break;
    case complete:
      //
      break;
    default: 
      // if nothing else matches, do the default
      // default is optional
    break;
  }*/
///////////////////////////////////////////////
  // put your main code here, to run repeatedly:
  //  if(millis() < 2000) {

  forward();
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
  //follow(color_threshold);
  //int hall_sensor = digitalRead(52);
  //magnet_detected();
  //detect_blue();

  //Vitellary
  /*for (int i = 0; i < 4; i++) {
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
  //  } else {
  //   forward();
   }
   }
  /*End Vitellary*/
 // send_signal(200);
//  delay(2000);
}

void forward() {
  mode_right = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_forward) {
    delay(1);
    mode_forward = true;
  }
  analogWrite(wheel_lh, 78);
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
  analogWrite(wheel_ll, 64);
  analogWrite(wheel_rl, 0);
  analogWrite(wheel_rh, 64);
}

void L_turn () {
  mode_forward = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_right) {
    delay(1);
    mode_right = true;
  }
  analogWrite(wheel_rl, 100);
  analogWrite(wheel_rh, 0);
  analogWrite(wheel_ll, 64);
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
  analogWrite(wheel_rh, 64);
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
  int start_time = millis();
  while (millis() - start_time < delay_timer) {
    analogWrite(wheel_rl, 100);
    analogWrite(wheel_rh, 0);
    analogWrite(wheel_ll, 64);
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
  if (ams_read() > threshold) {
    forward();
    found = true;
  }
  else {
    int curTime = millis();
    while (millis() < curTime + 125) {
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
      while (millis() < curTime + 500) {
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
  if (!found) {
    count++;
  }
  if (count > 5) {
    while (1)
    {
      brake();
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
      int curr_time = millis();
      while (millis() - curr_time < 50) {
        forward();
      }
      brake();
    }
    digitalWrite (red_LED, LOW);
    if (ams_read() > blue_threshold) {
      int curr_time = millis();
      found_blue = true;
      while (millis() - curr_time < 500) {
        L_turn();
      }
      while (1) {
        brake();
      }
    }
    else {
      int curr_time = millis();
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

void magnet_detected () {
  if (digitalRead(52)) {
      turn_180();
      turn_180();
    //digitalWrite (50, HIGH);
  }
  else {
    digitalWrite (50, LOW);
  }
}

void right_collide() {
  int start_time = millis();
  while (millis() - start_time < 500) {
    L_turn();
  }
  forward();
}

void left_collide() {
  int start_time = millis();
  while (millis() - start_time < 500) {
    R_turn();
  }
  forward();
}

void front_right_collide() {
  int start_time = millis();
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
  int start_time = millis();
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
  int start_time = millis();
  while(1){
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
    int start_time = millis();
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

/*void com_check() {
  //first communication 50ms - ask to resend
  int start_time = millis();
  int end_time; //define outside isr
  while (millis() - start_time < 50) {
    if (/*decide pin number == HIGH) {
      //make sure pin is high
      msg = true;
    }
    else {
      msg = false;
    }
  }
  if (msg) {
    //resend signal
    first_call = true;
  }
  if (first_call == true) {
    start_time = millis();
    while (millis() - start_time < 1000) {
      while (digitalRead(/*decide pin) == HIGH) {    }
      end_time = millis();
    }
      if (end_time == /*decide range for 200ms) {
          //starting
      }

      if (end_time == /*decide range for 300ms) {
          //found mine
      }

      if (end_time == /*decide range for 400ms) {
          //stop
      }
  }
}
      /*if(/*decide pin number == LOW) {
         //make sure pin is high
         msg = true;
         //starting
        }
        else {
         msg = false;
         break;
        }
        }
        //300ms
        start_time = millis();
        while (millis() - start_time < 300) {
        if(/*decide pin number == LOW) {
         //make sure pin is high
         msg = true;
         //found mine
        }
        else {
         msg = false;
         break;
        }
        }
        //400ms
        start_time = millis();
        while (millis() - start_time < 400) {
        if(/*decide pin number == LOW) {
         //make sure pin is high
         msg = true;
         //end/stop
        }
        else {2aq
         msg = false;
         break;
        }*/
 //   }
//  }
//}*/

/*Companion bot code:*/
/*void respondToSignal(){
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
/*End Companion bot code
void send_signal(int duration) {
  //change to use as delay
  digitalWrite(send_msg, HIGH);
  delay(duration);
  digitalWrite(send_msg, LOW);
}*/

