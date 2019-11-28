//Arduino LM35
//Baca temperatur, output celcius

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0, INPUT);

}
uint32_t in;

void loop() {
  // put your main code here, to run repeatedly:

  in = analogRead(A0);
  Serial.println((float)in * 500 / 1023);
  delay(1000);
}
