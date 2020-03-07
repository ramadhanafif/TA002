#include <SoftwareSerial.h>

SoftwareSerial espSerial(2, 3); // RX, TX

void setup()
{
  // set the data rate for the SoftwareSerial port
  espSerial.begin(9600);
  Serial.begin(115200);
}
String data;
//char buffer;
void loop() // run over and over
{
  if (espSerial.available()) {
    data = espSerial.readString();
    Serial.println(data);
  }
  delay(50);
}
