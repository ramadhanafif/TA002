#include <MapleFreeRTOS900.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(PA5, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PA5, HIGH);
  digitalWrite(PB1, LOW);

  delay(100);
  digitalWrite(PB1, HIGH);
  digitalWrite(PA5, LOW);
  delay(100);
}
