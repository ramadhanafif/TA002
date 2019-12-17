/* KODE PENGENDALI PEMANAS
   BOARD: BLACKPILL
*/

#include <MapleFreeRTOS900.h>

//Pin Definition
#define SW PA0
#define TEMP_SENSOR PA7

//PID constants
#define KP 10
#define KI 0.011

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double input, output, setPoint;
double cumError;
double dutyCycle;

void TaskCompute(void* v) {
  setPoint = 40;                          //set point at zero degrees
  Serial.begin(115200);
  pinMode(TEMP_SENSOR, INPUT_ANALOG);
  for (;;) {
    input = analogRead(TEMP_SENSOR);                //read from rotary encoder connected to A0
    input = input / 12.409 + 5;

    output = computePID(input);

    dutyCycle = (double) output / 500.0;
    if (dutyCycle > 1.0)
      dutyCycle = 1;
    else if (dutyCycle < 0.0)
      dutyCycle = 0;

    Serial.print(input); Serial.print(';');
    Serial.print(output); Serial.print(';');
    Serial.println(dutyCycle * 1);
    vTaskDelay(1000);
  }
}

void TaskSwitch(void* v) {
  pinMode(SW, OUTPUT);
  for (;;) {
    digitalWrite(SW, HIGH);
    vTaskDelay(200 * dutyCycle);
    digitalWrite(SW, LOW);
    vTaskDelay(200 * (1 - dutyCycle));
  }
}

void setup() {
  xTaskCreate(TaskSwitch,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
  xTaskCreate(TaskCompute,
              "Task2",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  vTaskStartScheduler();
}

void loop() {
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
