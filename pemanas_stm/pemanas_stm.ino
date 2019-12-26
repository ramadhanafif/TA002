/* KODE PENGENDALI PEMANAS v2
   BOARD: BLACKPILL
*/

#include <MapleFreeRTOS900.h>

//Pin Definition
#define SW PA0
#define TEMP_SENSOR PA7
#define BA 2.5
#define BB 2.5

void TaskCompute(void* v) {
  double setPoint = 80;                          //set point at zero degrees
  double input = 0;

  uint8_t upcount = 0,
          downcount = 0;
  Serial.begin(115200);
  pinMode(SW, OUTPUT);
  pinMode(TEMP_SENSOR, INPUT_ANALOG);
  for (;;) {
    input = analogRead(TEMP_SENSOR);                //read from rotary encoder connected to A0
    input = input / 12.409;

    if (setPoint + BA >= input) {
      upcount++; //mencegah lonjakan acak
      if (upcount > 10) {
        digitalWrite(SW, HIGH);
        upcount = 0;
        Serial.println(">>> PEMANAS ON");
      }
    }
    else if (setPoint - BB <= input) {
      downcount++;//mencegah lonjakan acak
      if (downcount > 0) {
        digitalWrite(SW, HIGH);
        downcount = 0;
        Serial.println(">>> PEMANAS OFF");
      }
    }
    vTaskDelay(10);
  }
}

void setup() {
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
