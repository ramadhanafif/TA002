/* KODE PENGENDALI PEMANAS v2
   BOARD: BLACKPILL
*/

//#include <MapleFreeRTOS900.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS1 4
OneWire oneWire1(ONE_WIRE_BUS1);
DallasTemperature sensor1(&oneWire1);

#define ONE_WIRE_BUS2 15
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensor2(&oneWire2);

//Pin Definition
#define SW 2
//#define TEMP_SENSOR 15
#define BA -0.7
#define BB 1

#define TIME_ON 30

unsigned int state = 0;
unsigned int ssr = 0;


void TaskCompute(void* v) {
<<<<<<< HEAD
  setPoint = 80;                          //set point at zero degrees
  Serial3.begin(115200);
  pinMode(TEMP_SENSOR, INPUT_ANALOG);
  for (;;) {
    input = analogRead(TEMP_SENSOR);                //read from rotary encoder connected to A0
    input = input / 12.409 ;
=======
  Serial.begin(115200);
>>>>>>> 439011c4d6c60515129d854a67925adfd55c8e91

  //  oneWire1.reset();
  //  oneWire2.reset();
  //
  //  oneWire1.search(addr);
  //  for (int i = 0; i < 8; i++)
  //  {
  //    Serial.print(addr[i], HEX);   //show on SM
  //  }
  //  oneWire2.search(addr);
  //  for (int i = 0; i < 8; i++)
  //  {
  //    Serial.print(addr[i], HEX);   //show on SM
  //  }

  double setPoint = 86;                          //set point at zero degrees
  double input = 0;

  unsigned int prev_input;

<<<<<<< HEAD
    dutyCycle = (double) output / 500.0;
    if (dutyCycle > 1.0)
      dutyCycle = 1;
    else if (dutyCycle < 0.0)
      dutyCycle = 0;

    Serial3.print(input); Serial3.print(';');
    Serial3.print(output); Serial3.print(';');
    Serial3.println(dutyCycle * 1);
    vTaskDelay(1000);
  }
}

void TaskSwitch(void* v) {
=======
>>>>>>> 439011c4d6c60515129d854a67925adfd55c8e91
  pinMode(SW, OUTPUT);
  sensor1.begin();
  sensor2.begin();

  unsigned int state = 0;
  unsigned int time_on = 0;
  for (;;) {
    sensor1.requestTemperatures();
    sensor2.requestTemperatures();

    input = sensor1.getTempCByIndex(0);

    if (setPoint - BB >= input) {
      if (abs(prev_input - input) < 2) {
        ssr = 1;
        for (int i = 0; i < 30; i++) {
          delay(1000);
          sensor1.requestTemperatures();

          Serial.print(sensor1.getTempCByIndex(0)); Serial.print(" ");
          //    Serial.print(sensor2.getTempCByIndex(0)); Serial.print(" ");
          Serial.print(ssr); Serial.print(" ");
          Serial.println(state);
          digitalWrite(SW, ssr);
        }
      }
    }
    ssr = 0;
    Serial.print(input); Serial.print(" ");
    //    Serial.print(sensor2.getTempCByIndex(0)); Serial.print(" ");
    Serial.print(ssr); Serial.print(" ");
    Serial.println(state);
    digitalWrite(SW, ssr);


    prev_input = input;
    vTaskDelay(800);
  }
}

void setup() {
  xTaskCreate(TaskCompute,
              "Task2",
              1024,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  //  vTaskStartScheduler();
}

void loop() {
}
