#include <SoftwareSerial.h>
 
//Parameter Initalization 
SoftwareSerial XBee(2,3); //RX, TX
int LVert = A0;
int LHor = A1;
int RVert = A2;
int RHor = A3;
int LV = 0;
int LH = 0;
int RV = 0;
int RH = 0;
 
 
void setup() {
Serial.begin(115200);
XBee.begin(9600);
}
 
void loop() {
  String Stringval ="";
  char Charval[23];
  LV = analogRead(LVert);
  LH = analogRead(LHor);
  RV = analogRead(RVert);
  RH = analogRead(RHor);
// Serial.println(LV);
// Serial.println(LH);
// Serial.println(RV);
// Serial.println(RH);
  Stringval=("#S#" + String(LV) + ":" + String(LH) + ":" + String(RV) + ":" + String(RH) + "#E#");
  Stringval.toCharArray(Charval,Stringval.length()+1);
  Serial.println(Charval);
  XBee.write(Charval);
  delay(200);
}
