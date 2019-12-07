//pin
#define sensor A0
#define relay 3

//koefisien PID
#define kp 1.5
#define ki 0.003125
#define kd 18.0

//variable PID
float p, i, d, pid;
float error = 0,
      errorx = 0,
      sumerr = 0;

//set point = 50
#define sp 50.0

//variable suhu
float vout, suhu;

float persen;

void setup() {
  pinMode(relay, OUTPUT);//pin relay
  Serial.begin(115200);
}

void loop() {
  vout = analogRead(sensor); //pembacaan sensor
  suhu = vout / 2.046; //suhu dalam celcius

  //relay
  digitalWrite(relay, HIGH);
  delay(20 * persen);
  digitalWrite(relay, LOW);
  delay(20 * (100 - persen));

  //PID
  error = sp - suhu;
  p = error * kp;
  sumerr += errorx;
  i = ki * sumerr;
  d = error - errorx;
  pid = p + i + d;

  //Normalisasi PID, konversi ke duty cycle
  persen = pid / 10;

  if (persen > 100) {
    persen = 100;
  }

  //Data
  //  Serial.print("suhu= ");
  Serial.println(suhu);
  //Serial.print("pid= ");
  //Serial.println(persen);

  errorx = error;

}
