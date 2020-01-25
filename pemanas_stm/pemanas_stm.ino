#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS2 4
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensor2(&oneWire2);

//Pin Definition
#define SW 2
//#define TEMP_SENSOR 15
#define BA -0.7
#define BB 2

#define TIME_ON 60

unsigned int state = 0;
unsigned int ssr = 0;
unsigned int input = 0;
unsigned int counter = 0;

int8_t get_temp(DallasTemperature sensor) {
  sensor.requestTemperatures();
  return sensor.getTempCByIndex(0);
}

void TaskPrint(void* v) {
  Serial.begin(115200);
  for (;;) {
    Serial.print(input); Serial.print(" ");
    Serial.print(ssr); Serial.print(" ");
    Serial.println();
    vTaskDelay(1000);
  }
}

void TaskCompute(void* v) {
  double setPoint = 80;                          //set point at zero degrees
  unsigned int prev_input;
  unsigned int state = 0;
  unsigned int time_on = 0;
  
  pinMode(SW, OUTPUT);
  sensor2.begin();
  for (;;) {
    input = get_temp(sensor2);
    switch (ssr)
    {
      case 0:
        digitalWrite(SW, ssr);
        if (input <= setPoint - BB) {
          if (abs(prev_input - input) < 2) {
            ssr = 1; //change state
            counter = 30;
          }
        }
        break;
      case 1:
        digitalWrite(SW, ssr);
        counter--;
        if (counter == 0) 
        {
          ssr = 0;
        }ß
        break;
    }
    
    prev_input = input;
    vTaskDelay(1000);
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

}

void loop() {
}
