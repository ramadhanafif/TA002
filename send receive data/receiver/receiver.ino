#include <SoftwareSerial.h>

SoftwareSerial espSerial(2, 3); // RX, TX

void setup()
{
  // set the data rate for the SoftwareSerial port
  espSerial.begin(115200);
  Serial.begin(115200);
}

union Data {
  int i;
  float f;
};



union Data ri () {
  union Data temp;
  temp.i = 0;
  for (int i = 0; i < 4; i++) {
    temp.i = temp.i << 8;
    temp.i = temp.i & espSerial.read();
  }
  return temp;
}

//char buffer;
void loop() // run over and over
{
  if (espSerial.available()) {
    Serial.println("\n.");
    if (espSerial.read() == 0x2) {
      Serial.print(ri().i); Serial.print(' ');
      Serial.print(ri().i); Serial.print(' ');
      Serial.print(ri().f); Serial.print(' ');
      Serial.print(ri().i); Serial.print(' ');
      Serial.print(ri().f); Serial.print(' ');
      Serial.print(ri().i); Serial.print(' ');
      Serial.print(ri().i); Serial.print(' ');
    }
    delay(2500);
  }
  delay(50);
}
