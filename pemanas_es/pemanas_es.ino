
//Pin Definition
#define SW 2
#define TEMP_SENSOR 15
#define BA 0.5
#define BB 2.5

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to GPIO15
#define ONE_WIRE_BUS 15
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

void TaskCompute(void* v) {

  double setPoint = 80;
  double stopPoint = 0.833 * setPoint - 14.892+10;
  double input = 0;
  uint8_t state = 0;
  uint8_t count = 0;
  unsigned int ssr = 0;

  Serial.begin(115200);
  pinMode(SW, OUTPUT);

  sensors.begin();

  Serial.print("STOP:"); Serial.println(stopPoint);
  for (;;) {
    sensors.requestTemperatures(); // Send the command to get temperatures
    input = sensors.getTempCByIndex(0);

    /*Pemanasan 0*/
    if (state == 0) {
      /*Relay Control*/
      if (input >= stopPoint) {
        ssr = 0;
      }
      else if (input <= stopPoint) {
        ssr = 1;
      }

      /*State Control*/
      if (input >= setPoint) {
        count++;
        Serial.print("menuju state steady: "); Serial.println(count);
        if (count > 15) { //stabil
          state++;
        }
      }
      /*---------------------end of state 0------------------*/

    }
    /*Pemanasan 1*/
    else if (state == 1) {
      if (input >= setPoint + BA) {
        ssr = 0;
      }
      else if (input <= setPoint - BB) {
        ssr = 1;
      }
    }
     /*---------------------end of state 1------------------*/
    Serial.print(input);
    Serial.print(';');
    Serial.print(state);
    Serial.print(';');
    Serial.println(ssr);

    digitalWrite(SW, ssr);
    vTaskDelay(800);
  }


}

void TaskPWM(void* pv) {
  for (;;) {
    digitalWrite(SW, ssr);
    vTaskDelay(50);
    digitalWrite(SW, ssr);
    vTaskDelay(50);
  }
}


void setup() {
  xTaskCreate(TaskCompute, "Task2", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
  //xTaskCreate(TaskPWM, "Task2", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void loop() {
}
