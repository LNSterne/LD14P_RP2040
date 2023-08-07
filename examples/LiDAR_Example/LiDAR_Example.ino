#include "ld14p.h"

Points2D sdata;
byte buf1[2048];
int counter = 0;
double t1, t2;

void setup() {
  // Set the baud rate for the SoftwareSerial object
  Serial.begin(115200);
}

void loop() {
  t1 = millis();
  if (t1 > t2 + 100) {
    sdata = GetLaserScanData();
    for (double ii = 0; ii < sdata.size(); ii++) {
      if (abs(sdata[ii].angle - 180) < 1) {
        Serial.println(sdata[ii].angle);
        Serial.println(sdata[ii].distance);
      }
    }
    t2 = t1;
  }
}
void setup1() {
  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.setFIFOSize(128);
  Serial2.begin(230400);
}

void loop1() {
  if (Serial2.available() > 0) {
    buf1[counter] = Serial2.read();
    counter++;
  }
  if (counter > 1024) {
    Parse(buf1,counter);
    AssemblePacket();
    counter = 0;
  }
}