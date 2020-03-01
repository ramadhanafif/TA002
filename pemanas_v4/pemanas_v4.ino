#include <OneWire.h>
#include <DallasTemperature.h>
#include "MedianFilterLib.h"

//Definition
#define TEMP_SENSOR_PIN 23
#define SSR_PIN 26

#define BB 1

#define PMNS_WAIT_TIME 40
#define PMNS_ON_TIME 30
#define PMNS_SET_POINT_DEBUG 90

#define PMNS_STATE_START 0
#define PMNS_STATE_STEADY 1

OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensor(&oneWire);

unsigned int PMNS_pemanas_state = 0;
unsigned int PMNS_flag_pemanas_awal_done = 0;
double TempRead = 0;

double get_temp(DallasTemperature sensor) {
  MedianFilter<double> medianFilter(3);
  for (int x = 0 ; x < 2 ; x++) {
    sensor.requestTemperatures();
    medianFilter.AddValue(sensor.getTempCByIndex(0));
  }

  sensor.requestTemperatures();
  return medianFilter.AddValue(sensor.getTempCByIndex(0));
}
void TaskPrintPemanas(void* v) {
  Serial.begin(115200);
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, 1000);
    Serial.print(TempRead); Serial.print(" ");
    Serial.print(PMNS_pemanas_state); Serial.print(" ");
    Serial.println();
  }
}

void taskPMNS_MAIN(void* v) {
  double setPoint = PMNS_SET_POINT_DEBUG;
  double prevtime = millis();
  double cumerror = 0;

  //  unsigned int prev_TempRead;
  unsigned int temp_counter = 0;
  unsigned int PMNS_ssr = 2;
  unsigned int PMNS_counter = 0;

  const int pwm_freq = 2;
  const int pwm_ledChannel = 2;
  const int pwm_resolution = 16;

  unsigned int pwm_attach = 0;

  ledcSetup(pwm_ledChannel, pwm_freq, pwm_resolution);

  float dutyCycle = 0;

  pinMode(SSR_PIN, OUTPUT);
  sensor.begin();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, 1000);
    TempRead = get_temp(sensor);
    switch (PMNS_pemanas_state) {
      case PMNS_STATE_START: //PID
        {
          //PID calculation
          double output = PMNS_computePID(TempRead, setPoint, &prevtime, &cumerror);

          //Normalization PID to dutyCycle (0 - 4095)
          dutyCycle = output / 500.0;
          if (dutyCycle > 1.0)
            dutyCycle = 1;
          else if (dutyCycle < 0.0)
            dutyCycle = 0;
          dutyCycle *= 0xFFFF;

          //PWM pin initialization
          if (!pwm_attach)
          {
            ledcAttachPin(SSR_PIN, pwm_ledChannel);
            pwm_attach = 1;
          }

          //PWM update dutyCycle
          ledcWrite(pwm_ledChannel, dutyCycle);

          //Check for temperature steadiness
          if (abs(TempRead - setPoint) <= 2.5) {
            temp_counter++;
          }
          if (temp_counter >= 60) {
            PMNS_flag_pemanas_awal_done = 1;
          }
          break;
        }
      case PMNS_STATE_STEADY: //On Off
        {
          //Stop PWM, switch to bang-bang
          if (pwm_attach) {
            ledcDetachPin(SSR_PIN);
            pwm_attach = 0;
          }

          switch (PMNS_ssr)
          {
            case 0: //nunggu sampe turun dari kelebihan suhu
              digitalWrite(SSR_PIN, LOW);
              if (TempRead <= setPoint - BB) {
                PMNS_ssr = 1; //change state
                PMNS_counter = PMNS_ON_TIME;
              }
              break;
            case 1: //decrement from PMNS_ON_TIME until 0
              digitalWrite(SSR_PIN, HIGH);
              PMNS_counter--;
              if (PMNS_counter == 0)
              {
                PMNS_ssr = 2;
              }
              break;
            case 2: //nunggu selama WAIT TIME
              digitalWrite(SSR_PIN, LOW);
              PMNS_counter++;
              if (PMNS_counter == PMNS_WAIT_TIME)
              {
                PMNS_ssr = 0;
              }
              break;
          }
          break; //break state 1
        }
    }
  }
}


void setup() {

  xTaskCreate(
    taskPMNS_MAIN,
    "Task2",
    1 << 11,
    NULL,
    tskIDLE_PRIORITY + 2,
    NULL
  );

  xTaskCreate(
    TaskPrintPemanas,
    "Task2",
    1024,
    NULL,
    tskIDLE_PRIORITY + 3,
    NULL
  );
}

void loop() {
}

double PMNS_computePID(double inp, unsigned int setPoint, double* previousTime, double* cumError) {
  double error = 0;
  double elapsedTime = 0;
  double currentTime;

  double kp = 8; //10
  double ki = 0.005; //0.011

  currentTime = millis() / 1000;                      //get current time
  elapsedTime = (currentTime - *previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                             // determine error
  *cumError += error * elapsedTime;                   // compute integral

  double out = kp * error + ki * *cumError;           // + kd * rateError;          //PID output

  *previousTime = currentTime;                        //remember current time

  return out;                                         //have function return the PID output
}
