void setup() {
  // put your setup code here, to run once:
  pinMode(A0,INPUT);
  Serial.begin(9600);
}

int reading;

void loop() {
  // put your main code here, to run repeatedly:
  reading = analogRead(A0);
  Serial.println((float)reading*500/1023);  
  delay(1000);
}
