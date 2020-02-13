#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS2 4
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensor2(&oneWire2);

double get_temp(DallasTemperature sensor) {
  sensor.requestTemperatures();
  return sensor.getTempCByIndex(0);
}

//Pin Definition
#define SW 2
#define PERIOD 400 //ms
//PID constants
#define KP 8 //10
#define KI 0.005 //0.011

double dutyCycle = 0;
unsigned int setPoint = 75;

void TaskPWM(void* v) {
  pinMode(SW, OUTPUT);
  for (;;) {
    digitalWrite(SW, HIGH);
    vTaskDelay(dutyCycle * PERIOD);
    digitalWrite(SW, LOW);
    vTaskDelay((1 - dutyCycle)*PERIOD);
  }
}

void TaskCompute(void* v) {
  Serial.begin(115200);
  Serial.println("TaskCompute started");
  sensor2.begin();

  double input = 0;
  double output = 0;
  double prevtime = millis();
  double cumerror = 0;

  for (;;) {
    input = get_temp(sensor2);

    output = computePID(input,setPoint, &prevtime, &cumerror);

    dutyCycle = (double) output / 500.0;
    if (dutyCycle > 1.0)
      dutyCycle = 1;
    else if (dutyCycle < 0.0)
      dutyCycle = 0;
    vTaskDelay(1000);
  }
}

void setup() {
  // put your setup code here, to run once:
  xTaskCreate(TaskCompute,
              "Task2",
              1024,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(TaskPWM,
              "Task2",
              1024,
              NULL,
              tskIDLE_PRIORITY + 3,
              NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}


//pemanas
double computePID(double inp, unsigned int setPoint, double* previousTime, double* cumError) {
  double error = 0;
  double elapsedTime = 0;
  double currentTime;

  double kp = 8; //10
  double ki = 0.005; //0.011

  currentTime = millis() / 1000;              //get current time
  elapsedTime = (currentTime - *previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                                // determine error
  *cumError += error * elapsedTime;                // compute integral

  double out = kp * error + ki * *cumError;// + kd * rateError;          //PID output

  *previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
}
