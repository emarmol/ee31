//////////////////CONSTANTS///////////////////
int fwd = 0;
int rev = 1;
int delay_timer = 2000;
int ams_sensor = A0;
int sensor_val = 0;
int blue_threshold = 90;
int color_threshold = 40;
int red_LED = 22;
int blue_LED = 23;
//int hall_sensor = 6;
/////////////////////////////////////////////

//////////////////VARIABLES//////////////////
bool mode_forward = false;
bool mode_reverse = false;
bool mode_right = false;
bool mode_left = false;
bool found = false;
int count = 0;
/////////////////////////////////////////////
void setup() {
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(red_LED, OUTPUT);
  pinMode(blue_LED, OUTPUT);
  pinMode(ams_sensor, INPUT);
  pinMode(7, OUTPUT);
  digitalWrite (red_LED, HIGH);
  digitalWrite (blue_LED, HIGH);
  Serial.begin(9600);
}

void loop() {
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

  //detect_color();
  follow();
  //int hall_sensor = digitalRead(6);
  //Serial.println(hall_sensor);
  //magnet_detected();
  //detect_blue();
}

void forward() {
  mode_right = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_forward) {
    delay(1);
    mode_forward = true;
  }
  analogWrite(3, 64);
  analogWrite(2, 0);
  analogWrite(4, 82);
  analogWrite(5, 0);
}

void reverse() {
  mode_forward = false;
  mode_right = false;
  mode_left = false;
  if (!mode_reverse) {
    delay(1);
    mode_reverse = true;
  }
  analogWrite(3, 0);
  analogWrite(2, 64);
  analogWrite(4, 0);
  analogWrite(5, 64);
}

void L_turn () {
  mode_forward = false;
  mode_left = false;
  mode_reverse = false;
  if (!mode_right) {
    delay(1);
    mode_right = true;
  }
  analogWrite(4, 100);
  analogWrite(5, 0);
  analogWrite(2, 64);
  analogWrite(3, 0);
}

void R_turn () {
  mode_forward = false;
  mode_right = false;
  mode_reverse = false;
  if (!mode_left) {
    delay(1);
    mode_left = true;
  }
  analogWrite(3, 100);
  analogWrite(2, 0);
  analogWrite(5, 64);
  analogWrite(4, 0);

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
    analogWrite(4, 100);
    analogWrite(5, 0);
    analogWrite(2, 64);
    analogWrite(3, 0);
  }
  brake();
}

void brake() {
  analogWrite(4, 0);
  analogWrite(5, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
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

void follow() {
  found = false;
  if (ams_read() > blue_threshold) {
    forward();
    found = true;
  }
  else {
    int curTime = millis();
    while (millis() < curTime + 125) {
      L_turn();
      if (ams_read() > blue_threshold) {
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
      while (millis() < curTime + 300) {
        R_turn();
        if (ams_read() > blue_threshold) {
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
  if (digitalRead(6)) {
    digitalWrite (7, HIGH);
  }
  else {
    digitalWrite (7, LOW);
  }
}

