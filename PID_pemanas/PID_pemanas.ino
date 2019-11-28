//#include "Wire.h"
//#include <max6675.h>

/*int thermoDO = 11; //bisa juga S0
  int thermoCS = 10;
  int thermoCLK = 9; //bisa juga SCK
*/
//int led = 8;
//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//pin
#define sensor A0
#define relay 3

//koefisien PID
#define kp 2.08
#define ki 1.67
//static float kd = 2.15;

//variable PID
float p, i, d, pid;
float error, errorx, sumerr;

//set point = 50
#define p 50.0

//variable suhu
float vout, suhu;

//pwm relay
//uint32_t vv = 10; //persenan
float persen;

void setup() {
  pinMode(relay, OUTPUT);//pin relay
  //pinMode(led,OUTPUT);
  Serial.begin(57600);
}


void loop() {

  vout = analogRead(sensor); //pembacaan sensor
  vout = (vout * 500) / 1023; //suhu dalam celcius

  //relay
  digitalWrite(relay, HIGH);
  delay(20 * persen);
  digitalWrite(relay, LOW);
  delay(20 * (100 - persen));

  //PID
  error = sp - suhu;
  p = error * kp;
  sumerr = error + errorx;
  i = ki * sumerr;
//  d = error - errorx;
  pid = p + i + d;
  pid = 255.0 - pid;

  if (pid < 1) {
    pid = 0;
  }

  persen = pid / 2.55;

  if (persen > 100) {
    persen = 100;
  }

  persen = 100 - persen;

  //display
//  Serial.print("suhu= ");
  Serial.println(suhu);
  //Serial.print("pid= ");
  //Serial.println(persen);

  delay(1000);

  errorx = error;

}
