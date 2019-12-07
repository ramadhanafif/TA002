
//pin
int sensor = A0;
int relay = 7;

//koefisien PID
float kp = 50;
float ki = 0.05;
float kd = 20;

//variable PID
float p, i, d, pid;
float error, errorx, sumerr;

//set point = 50
float sp = 50.0;

//variable suhu
float vout, suhu;

//pwm relay
//uint32_t vv = 10; //persenan
int persen;

void setup() {
  pinMode(relay, OUTPUT);//pin relay
  //pinMode(led,OUTPUT);
  Serial.begin(9600);

  // wait for MAX chip to stabilize
  //delay(500);
}


void loop() {

  //float suhu = thermocouple.readCelsius();
  //analogWrite(led,pid);

  //baca suhu
  vout = analogRead(sensor); //pembacaan sensor
  suhu = (vout * 500) / 1023; //suhu dalam celcius

  //relay
  digitalWrite(relay, HIGH);
  delay(1 * persen);
  digitalWrite(relay, LOW);
  delay(1 * (100 - persen));

  //PID
  error = sp - suhu;
  p = error * kp;
  sumerr += error;
  i = ki * sumerr;
  d = error - errorx;
  pid = p + i + d;

  //normalisasi
  pid = pid / 1000;

  if (pid > 1)
    pid = 1;
  
  persen = (pid * 100);

  //display

  Serial.print(suhu);
  Serial.print(";");
  Serial.print(persen);
  Serial.print(";");
  Serial.println(pid);

  delay(1000);

  errorx = error;

}
