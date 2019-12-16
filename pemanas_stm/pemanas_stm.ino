/* KODE PENGENDALI PEMANAS
   BOARD: BLACKPILL
*/

//Pin Definition
#define SW PA0
#define TEMP_SENSOR PA1

//PID constants
#define KP 10
#define KI 0.011

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double input, output, setPoint;
double cumError;
double dutyCycle;

void setup() {
  setPoint = 100;                          //set point at zero degrees
  Serial.begin(115200);
  pinMode(SW, PWM);
}

void loop() {
  input = analogRead(TEMP_SENSOR);                //read from rotary encoder connected to A0
  input = input / (330 / 4095);

  output = computePID(input);

  dutyCycle = output / 500;
  if (dutyCycle > 1)
    dutyCycle = 1;
  else if (dutyCycle < 1)
    dutyCycle = 0;

  pwmWrite(SW, (uint16_t) (dutyCycle * 65535));

  Serial.print(input); Serial.print(';');
  Serial.println(dutyCycle * 100);
}


double computePID(double inp) {
  currentTime = millis() / 1000;              //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                                // determine error
  cumError += error * elapsedTime;                // compute integral

  double out = KP * error + KI * cumError;// + kd * rateError;          //PID output

  previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
}
