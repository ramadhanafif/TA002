/* 
* Atur temperatur dengan relay
* Perioda = 1 sec
* Board: ESP32
*/

void setup() {
  // put your setup code here, to run once:
  pinMode(16, OUTPUT);

}
uint32_t vv = 1; //persenan

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(16, HIGH);
  delay(10 * vv);
  digitalWrite(16, LOW);
  delay(10 * (100 - vv));

}
