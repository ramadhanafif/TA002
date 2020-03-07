union Data {
  int i;
  float f;
};
union Data msg[9];
void setup() {
  // put your setup code here, to run once:


  //FORMAT DATA: STATE;SP TEMP;TEMP;SP RPM;RPM;SP SEKON;SEKON

  msg[0].i = 0x2;
  msg[1].i = 7;
  msg[2].f = 70;
  msg[3].i = 70.2;
  msg[4].f = 50;
  msg[5].i = 50.4;
  msg[6].i = 3600;
  msg[7].i = 1200;
  msg[8].i = 0x6;
  Serial2.begin(115200);
  //  for (int x = 0; x < 9; x++)
  //  {
  //    Serial2.write(msg[x].i);
  //  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int x = 0; x < 9; x++)
  {
    if (x == 4 || x == 2)
      Serial2.print(msg[x].f);
    else
      Serial2.print(msg[x].i);
  }
  delay(3000);
}
