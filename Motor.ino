#include <HMC5883L.h>
#include <NewPing.h>
#include <Wire.h>
#include <MaxMatrix.h>

byte frame[] = {8, 8,
                B11111111,
                B00000001,
                B10000001,
                B10000001,
                B10000001,
                B10000001,
                B10000001,
                B11111101
               };

byte turnleft[] = {8, 8,
                   B00010000,
                   B00110000,
                   B01111111,
                   B11111111,
                   B11111111,
                   B01111111,
                   B00110000,
                   B00010000
                  };

byte turnright[] = {8, 8,
                    B00001000,
                    B00001100,
                    B11111110,
                    B11111111,
                    B11111111,
                    B11111110,
                    B00001100,
                    B00001000
                   };

byte reset[] = {8, 8,
                B00111100,
                B01000010,
                B10000001,
                B10000001,
                B10000101,
                B10000110,
                B01000111,
                B00110000
               };

enum {L, R};
enum {CLOCKWISE = 0, COUNTER_CLOCKWISE};

//sonic sensors
NewPing sonarL(24, 22, 50);
NewPing sonarC(32, 30, 50);
NewPing sonarR(28, 26, 50);

//L298N PWM pins
const int RM1 = 4;
const int RM2 = 5;
const int LM1 = 6;
const int LM2 = 7;
const int LB = 30;
const int RB = 45;
const int Lstep = 23;
const int Rstep = 25;
int cnt;

//Eletronic compass
HMC5883L compass;
float initDigree, currDegree;

//MAX7219
int data = 49;    // DIN pin of MAX7219 module
int load = 51;    // CS pin of MAX7219 module
int clock = 53;  // CLK pin of MAX7219 module
int maxInUse = 1;    //change this variable to set how many MAX7219's you'll use

MaxMatrix LEDMatrix(data, load, clock, maxInUse); // define module

int dis_left, dis_center, dis_right, start = 0;
int LS, RS;

void Drive(int Wheel, int Speed) {
  if ((Speed >> 15) & 1) {
    if (Wheel == L) {
      analogWrite(LM1, 0);
      analogWrite(LM2, -Speed + LB);
    }
    if (Wheel == R) {
      analogWrite(RM1, 0);
      analogWrite(RM2, -Speed + RB);
    }
  }
  else {
    if (Wheel == L) {
      analogWrite(LM1, Speed + LB);
      analogWrite(LM2, 0);
    }
    if (Wheel == R) {
      analogWrite(RM1, Speed + RB);
      analogWrite(RM2, 0);
    }
  }
}

void Stop(int delay_time) {
  analogWrite(LM1, 0);
  analogWrite(LM2, 0);

  analogWrite(RM1, 0);
  analogWrite(RM2, 0);
  delay(delay_time);
}

float getCurrDegree() {
  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  return heading * 180 / M_PI;
}

int getWarp() {
  int warp = abs(getCurrDegree() - currDegree);
  if (warp > 180) warp = abs(warp - 360);

  return warp;
}

void turn(int rotate_type) {
  float temp, target = currDegree + (rotate_type ? -90 : 90);
  int speed = rotate_type ? -60 : 60;
  byte* arrow = rotate_type ? turnleft : turnright;

  //LED Indicator
  LEDMatrix.writeSprite(0, 0, arrow);

  temp = getCurrDegree();
  if (target < 0) target += 360;
  if (target >= 360) target -= 360;
  
  Drive(L, speed);
  Drive(R, -speed);
  while (!(abs(temp - target) <= 20 || abs(temp - target) >= 340)) {
    temp = getCurrDegree();
    //Serial.println(temp);
    //Drive(L, speed);
    //Drive(R, -speed);
    //delay(50);
    //Stop(70);
  }
  Stop(100);
  currDegree += (rotate_type ? -90 : 90);
  if (currDegree < 0) currDegree += 360;
  if (currDegree >= 360) currDegree -= 360;
  delay(100);
  setHarmonics();

  LEDMatrix.clear();
}

void setHarmonics() { //S2CA triburst! (S2CA = Spark Song Combination Art)
  LEDMatrix.writeSprite(0, 0, reset);
  int dir;
  int temp = getCurrDegree() - currDegree;
  int Speed;
  if (temp < -180) dir = L;
  else if (temp > 180) dir = R;
  else if (temp > 0) dir = L;
  else dir = R;

  if (dir == L) Speed = -70;
  else Speed = 70;

  temp = getCurrDegree();
  while (1) {
    temp = getCurrDegree();
    Drive(L, Speed);
    Drive(R, -Speed);
    delay(50);
    Stop(70);
    if (abs(temp - currDegree) <= 5 || abs(temp - currDegree) >= 355) break;
  }
  LEDMatrix.clear();
}

void step() {
  int dis = 50;
  int Lstate, Rstate, LP, RP; //LPrevious, RPrevious
  int Lstuck = 0, Rstuck = 0;
  Lstate = LP = digitalRead(Lstep);
  Rstate = RP = digitalRead(Rstep);
  int Lcnt = 0, Rcnt = 0;
  Drive(L, 50);
  Drive(R, 50);
  while (1) {
    Lstate = digitalRead(Lstep);
    Rstate = digitalRead(Rstep);
    if (Lstate != LP) {
      Lcnt++;
      LP = Lstate;
      Lstuck = 0;
    }
    else {
      Lstuck++;
    }

    if (Rstate != RP) {
      Rcnt++;
      RP = Rstate;
      Rstuck = 0;
    }
    else {
      Rstuck++;
    }

    if (Lcnt > dis) Drive(L, 0);
    if (Rcnt > dis) Drive(R, 0);
    if (Lcnt > dis && Rcnt > dis && sonarL.ping_cm()) break;

    Serial.print(Lstuck);
    Serial.print("  ");
    Serial.println(Rstuck);
    if (Lstuck > 40 || Rstuck > 40) {
      Drive(L, -70);
      Drive(R, -70);
      delay(25);
      Stop(0);
      setHarmonics();
      Lstuck = 0;
      Rstuck = 0;
      Lcnt = dis / 2;
      Rcnt = dis / 2;
      Drive(L, 50);
      Drive(R, 50);
    }
  }
  Stop(1);
}
/**************************************************************************************************/

void setup() {
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(Lstep, INPUT);
  pinMode(Rstep, INPUT);

  Serial.begin(9600);
  Wire.begin();
  LEDMatrix.init();
  LEDMatrix.setIntensity(15);

  while (!compass.begin()) delay(500);
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  //compass.setOffset(24, 20);
  compass.setOffset(135, 10);
  delay(1000);
  currDegree = initDigree = getCurrDegree();
  Serial.println(initDigree);

  //get initial position
  dis_left = sonarL.ping_cm();
  dis_center = sonarC.ping_cm();
  dis_right = sonarR.ping_cm();
  LS = 70;
  RS = 70;
  cnt = 0;
}

void loop() {
  int warp = getWarp();
  if (warp > 15) setHarmonics();
  Serial.println(getCurrDegree());
  //get measurement data from sensors
  dis_left = sonarL.ping_cm();
  dis_center = sonarC.ping_cm();
  dis_right = sonarR.ping_cm();

  if (dis_left == 0) {
    cnt++;
    if (cnt == 3 || (cnt == 9 || cnt == 14)) {
      step();
    }
    else {
      Drive(L, 100);
      Drive(R, 100);
      delay(70);
      Stop(100);
      turn(COUNTER_CLOCKWISE);
      step();
      Stop(100);
    }
  }
  else if (dis_center < 12 && dis_center) {
    Stop(20);
    if (dis_right == 0 || dis_right > 20) {
      turn(CLOCKWISE);
      step();
      Stop(100);
    }
    else {
      if (dis_left < dis_right) {
        turn(CLOCKWISE);
        delay(100);
        turn(CLOCKWISE);
      }
      else {
        turn(COUNTER_CLOCKWISE);
        delay(100);
        turn(COUNTER_CLOCKWISE);
      }
    }
  }

  else {
    if (dis_left < 5 && dis_left) {
      Drive(L, -100);
      Drive(R, -100);
      delay(100);
      Stop(1);
      Drive(L, 100);
      delay(100);
      Stop(1);
    }
    if (dis_right < 5 && dis_right) {
      Drive(L, -100);
      Drive(R, -100);
      delay(100);
      Stop(1);
      Drive(R, 100);
      delay(100);
      Stop(1);
    }
    LS = 70;
    RS = 70;
    if (dis_left < dis_right) LS += 10;
    else if (dis_left > dis_right) RS += 10;
    Drive(L, LS);
    Drive(R, RS);
    delay(100);
  }

}
