#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS2 4
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensor2(&oneWire2);

//Pin Definition
#define SW 2
//#define TEMP_SENSOR 15
#define BA -0.7
#define BB 1

#define TIME_ON 40
#define SET_POINT 90

#define PERIOD 400

TaskHandle_t xPWMHandle = NULL;
unsigned int state = 0;
unsigned int ssr = 2;
double input = 0;
unsigned int counter = 0;
double dutyCycle = 0;

double get_temp(DallasTemperature sensor) {
  sensor.requestTemperatures();
  return sensor.getTempCByIndex(0);
}

void TaskPrint(void* v) {
  Serial.begin(115200);
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, 1000);
    Serial.print(input); Serial.print(" ");
    Serial.print(state); Serial.print(" ");
    Serial.print(ssr); Serial.print(" ");
    Serial.println();
  }
}

void TaskCompute(void* v) {
  double setPoint = SET_POINT;                          //set point at zero degrees
  unsigned int prev_input;
  unsigned int time_on = 0;

  double prevtime = millis();
  double cumerror = 0;

  unsigned int temp_counter = 0;

  pinMode(SW, OUTPUT);
  sensor2.begin();
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, 1000);
    input = get_temp(sensor2);
    switch (state) {
      case 0: //PID
        {
          double output = computePID(input, setPoint, &prevtime, &cumerror);

          dutyCycle = output / 500.0;
          if (dutyCycle > 1.0)
            dutyCycle = 1;
          else if (dutyCycle < 0.0)
            dutyCycle = 0;

          if (abs(input - setPoint) <= 2.5) {
            temp_counter++;
          }

          if (temp_counter >= 60) {
            state++;
          }
          break;
        }
      case 1: //On Off
        {
          vTaskSuspend(xPWMHandle); //delete PWM.
          switch (ssr)
          {
            case 0: //nunggu sampe turun dari kelebihan suhu
              digitalWrite(SW, ssr);
              if (input <= setPoint - BB) {
                if (abs(prev_input - input) < 2) {
                  ssr = 1; //change state
                  counter = TIME_ON;
                }
              }
              break;
            case 1: //on TIME_ON s
              digitalWrite(SW, ssr);
              counter--;
              if (counter == 0)
              {
                ssr = 2;
              }
              break;
            case 2: //nunggu TIME_ON s
              digitalWrite(SW, 0);
              counter++;
              if (counter == TIME_ON)
              {
                ssr = 0;
              }
              break;
          }
          prev_input = input;

          break; //break state 1
        }
    }
  }
}

void TaskPWM(void* v) {
  pinMode(SW, OUTPUT);
  for (;;) {
    digitalWrite(SW, HIGH);
    vTaskDelay(dutyCycle * PERIOD);
    digitalWrite(SW, LOW);
    vTaskDelay((1 - dutyCycle)*PERIOD);
  }
}

void setup() {

  xTaskCreate(TaskCompute,
              "Task2",
              1024,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);

  xTaskCreate(TaskPrint,
              "Task2",
              1024,
              NULL,
              tskIDLE_PRIORITY + 3,
              NULL);

  xTaskCreate(TaskPWM,
              "Task2",
              1024,
              NULL,
              tskIDLE_PRIORITY + 1,
              &xPWMHandle);
}

void loop() {
}

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
