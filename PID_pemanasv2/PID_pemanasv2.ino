/* pid pemanas yg bener
    board wemos lolin
*/

#define relay 4
//PID constants
double kp = 1;
double ki = 0.05;
double kd = 1;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
double persen = 0;

void setup() {
  setPoint = 50;                          //set point at zero degrees
  Serial.begin(115200);
  pinMode(relay,OUTPUT);
  Serial.println("tes");
  delay(10000);
}

void loop() {
  input = analogRead(34);                //read from rotary encoder connected to A0
  input = input * 500 / 4095;

  output = computePID(input);
  delay(100);

  pesen = output / 5000 * 100;
  if (persen > 100) 
    persen = 100;
  if (persen < 0)
    persen = 0;

  cetak(persen);

  Serial.print(input); Serial.print(' '); Serial.println(persen);
}

void cetak(double pwm) {
  digitalWrite(relay, HIGH);
  delay(20 * pwm);
  digitalWrite(relay, LOW);
  delay(20 * (100 - pwm));
}

double computePID(double inp) {
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                                // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;          //PID output

  lastError = error;                                //remember current error
  previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
}
