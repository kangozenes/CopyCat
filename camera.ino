#include<Servo.h>
Servo x;
int genislik = 640, yukseklik = 480, xp = 90;  
const int aci = 2;  
void setup() {
  Serial.begin(9600);
  x.attach(6);
}
void loop() {
  if (Serial.available() > 0)
  {
    int x_merkez;
    if (Serial.read() == 'X')
    {
      x_merkez = Serial.parseInt(); 
      
    }
    if (x_merkez > genislik / 2 + 60)
      xp += aci;
      x.write(xp);
    if (x_merkez < genislik / 2 - 60)
      xp -= aci;
      x.write(xp);
   
  }
}
