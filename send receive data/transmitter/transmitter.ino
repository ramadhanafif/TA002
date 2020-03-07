

//FORMAT DATA: STATE;SP TEMP;TEMP;SP RPM;RPM;SP SEKON;SEKON
uint32_t state = 7;
uint32_t sptemp = 70;
double temp = 70.2;
uint32_t sprpm = 50;
double rpm = 50.4;
uint32_t spsec = 3600;
uint32_t sec = 1200;

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(9600);
  Serial.begin(115200);
}
char data[99];

void loop() {
  // put your main code here, to run repeatedly:
  sprintf(data, "%d,%d,%.2f,%d,%.2f,%d,%d",
          state,
          sptemp, temp,
          sprpm, rpm,
          spsec, sec);

  Serial2.println(data);
  delay(3000);

}
