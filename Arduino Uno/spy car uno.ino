#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>
#include <NewPing.h>

//#define stepper_test
#ifndef stepper_test

const int stepper_en = 2;
const int sonarpin2 = 10;
const int sonarpin1 = 9;

NewPing hcsr04_1(sonarpin2, A1, 200);
NewPing hcsr04_2(sonarpin1, A0, 200);

int command = 0;
int num_byte = 0;
int speed = 250;
uint32_t current_time = 0;

int distance_back = 0, distance_front = 0;
int last_state = LOW;
String distance = "00000000s";

bool CW = false, stepper_running = false;

AF_DCMotor motor1(1); 
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

void activateMotor(int motor1_t, int motor2_t, int motor3_t, int motor4_t) {
  motor1.run(motor1_t);
  motor2.run(motor2_t);
  motor3.run(motor3_t);
  motor4.run(motor4_t);
}

void setMotorSpeed(int scale) {
  motor1.setSpeed(speed/scale);
  motor2.setSpeed(speed/scale);
  motor3.setSpeed(speed/scale);
  motor4.setSpeed(speed/scale);
}

void rest() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void receive_event_handler(int num_byte0) {
  if (Wire.available() == 1) {
    int temp = Wire.read();
    if (temp != 13 && temp < 100)
    command = temp;

    else if (temp >= 100) {
      speed = temp;
    }
  }
}

void request_event_handler() {
  Wire.write(distance_back);
  Wire.write(distance_front);
}

void setup() {
  // put your setup code here, to run once:

  Wire.begin(0x16);
  Wire.onRequest(request_event_handler);
  Wire.onReceive(receive_event_handler);
  Serial.begin(9600);
  setMotorSpeed(1);
  rest();
  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(sonarpin2, OUTPUT);
  pinMode(sonarpin1, OUTPUT);
  pinMode(stepper_en, OUTPUT);
  
  digitalWrite(sonarpin2, LOW);
  digitalWrite(sonarpin1, LOW);
  digitalWrite(stepper_en, LOW);

  //run_stepper(2);

  Serial.println("ready");
}

String distance_length(int dis) {
  if (dis < 10) return "1";
  else if (dis >= 10 && dis < 100) return "2";
  return "3";
}

void reset_distance() {
  for (int i = 0; i < 8; i++) {
    distance[i] = '0';
  }
}

void loop() {

    if (millis() - current_time >= 50)
    {
      distance_front = hcsr04_1.ping_cm(200);
      distance_back = hcsr04_2.ping_cm(200);
      current_time = millis();
      reset_distance();
      // String a = (distance_length(distance_back) + distance_length(distance_front) + String(distance_back) + String(distance_front));
      // for (int i = 0; i < a.length(); i++) {
      //   distance[i] = a[i];
      // }
      //Serial.println(a);
    }

  if (command == 0) {
    // activateMotor(FORWARD, FORWARD, FORWARD, FORWARD);
    rest(); 
    digitalWrite(stepper_en, LOW);
  }

  else {
    setMotorSpeed(1);
    //  if (distance_back < 15 || distance_front < 15)
    //   setMotorSpeed(2);
    switch (command)
    {
    case 1:
    //Serial.println((distance_front));
      if (distance_front >= 10 || distance_front == 0)
      activateMotor(FORWARD, FORWARD, FORWARD, FORWARD);
      else 
      rest();
      break;
    
    case 2:
      if (distance_back >= 10 || distance_back == 0)
      activateMotor(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      else
      rest();
      break;

    case 3:
      activateMotor(FORWARD, FORWARD, BACKWARD, BACKWARD);
      break;
    
    case 4:
      activateMotor(BACKWARD, BACKWARD, FORWARD, FORWARD);
      break;
    
    case 5:
      activateMotor(BACKWARD, BACKWARD, FORWARD, FORWARD);
      break;

    case 6:
      activateMotor(FORWARD, FORWARD, BACKWARD, BACKWARD);
      break;
    
    case 11:
      digitalWrite(stepper_en, HIGH);
      break;
    
    case 12:
      digitalWrite(stepper_en, HIGH);
      break;
    }
  }
}
#else
#include <Stepper.h>

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void loop() {
 
}
#endif
