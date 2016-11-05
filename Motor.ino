#include <HMC5883L.h>
#include <NewPing.h>
#include <Wire.h>

enum {L, R};
enum {CLOCKWISE = 0, COUNTER_CLOCKWISE};

//sonic sensors
NewPing sonarL(18, 19, 50);
NewPing sonarC(16, 17, 50);
NewPing sonarR(14, 15, 50);

//L298N PWM pins
const int RM1 = 2;
const int RM2 = 4;
const int LM1 = 3;
const int LM2 = 5;

//Eletronic compass
HMC5883L compass;
float initDigree, currDegree;

int dis_left, dis_center, dis_right, start = 0;
int offX = 0, offY = 0, minX = 0, minY = 0, maxX = 0, maxY = 0;
int LS, RS;

void setup() {
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(6, INPUT_PULLUP);

  Serial.begin(9600);
  Wire.begin();

  while (!compass.begin()) delay(500);
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(24, 20);
  delay(1000);
  currDegree = initDigree = getCurrDegree();
  Serial.println(initDigree);

  //get initial position
  dis_left = sonarL.ping_cm();
  dis_center = sonarC.ping_cm();
  dis_right = sonarR.ping_cm();
  LS = 70;
  RS = 70;
}

void Drive(int Wheel, int Speed) {
  if ((Speed >> 15) & 1) {
    if (Wheel == L) {
      analogWrite(LM1, 0);
      analogWrite(LM2, -Speed);
    }
    if (Wheel == R) {
      analogWrite(RM1, 0);
      analogWrite(RM2, -Speed);
    }
  }
  else {
    if (Wheel == L) {
      analogWrite(LM1, Speed);
      analogWrite(LM2, 0);
    }
    if (Wheel == R) {
      analogWrite(RM1, Speed);
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

void turn(int rotate_type) {
  //Serial.println("Rotating...");
  float temp, target = currDegree + (rotate_type ? -80 : 90);
  int speed = rotate_type ? -100 : 100;

  temp = getCurrDegree();

  if (target < 0) target += 360;
  if (target >= 360) target -= 360;

  while (abs(temp - target) >= 4) {
    temp = getCurrDegree();
    Serial.println(temp);
    Drive(L, speed);
    Drive(R, -speed);
    delay(50);
    Stop(70);
  }
  Stop(100);
  currDegree += (rotate_type ? -90 : 90);
  if (currDegree < 0) currDegree += 360;
  if (currDegree >= 360) currDegree -= 360;
  //Serial.println("Finish");
}

void loop() {

  /*
    while (!start) {
    Vector mag = compass.readRaw();

    // Determine Min / Max values
    if (mag.XAxis < minX) minX = mag.XAxis;
    if (mag.XAxis > maxX) maxX = mag.XAxis;
    if (mag.YAxis < minY) minY = mag.YAxis;
    if (mag.YAxis > maxY) maxY = mag.YAxis;

    // Calculate offsets
    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
    if (!digitalRead(6)) start = 1;
    }
  */
  //compass.setOffset(offX, offY);
  Serial.println(getCurrDegree());
  //get measurement data from sensors
  dis_left = sonarL.ping_cm();
  dis_center = sonarC.ping_cm();
  dis_right = sonarR.ping_cm();

  if (dis_left == 0) {
    Drive(L, 100);
    Drive(R, 100);
    delay(300);
    Stop(100);
    turn(COUNTER_CLOCKWISE);
    Drive(L, 100);
    Drive(R, 100);
    delay(700);
    Stop(100);
  }
  else if (dis_center < 15 && dis_center) {
    Stop(20);
    if (dis_right == 0 || dis_right > 20) {
      turn(CLOCKWISE);
      Drive(L, 90);
      Drive(R, 100);
      delay(500);
      Stop(100);
    }
    else {
      turn(CLOCKWISE);
      delay(500);
      turn(CLOCKWISE);
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
