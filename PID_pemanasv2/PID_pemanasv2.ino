/* pid pemanas yg bener
    board wemos lolin
*/

#define relay 3

//PID constants
#define kp 10
#define ki 0.03
#define kd 140

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
double persen = 0;

void setup() {
  setPoint = 100;                          //set point at zero degrees
  Serial.begin(115200);
  pinMode(relay, OUTPUT);
}

void loop() {
  input = analogRead(A0);                //read from rotary encoder connected to A0
  input = input / 2.046;

  output = computePID(input);

  persen = output / 500 * 100;
  if (persen > 100)
    persen = 100;
  if (persen < 0)
    persen = 0;

  cetak(persen);

  Serial.print(input); Serial.print(';');
  Serial.print(output); Serial.print(';');
  Serial.println(persen);
}

void cetak(double pwm) {
  digitalWrite(relay, HIGH);
  delay(20 * pwm);
  digitalWrite(relay, LOW);
  delay(20 * (100 - pwm));
}

double computePID(double inp) {
  currentTime = millis() / 1000;              //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                                // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;          //PID output

  lastError = error;                                 //remember current error
  previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
}
