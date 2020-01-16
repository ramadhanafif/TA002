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
  double setPoint = 40;                          //set point at zero degrees
  double input = 0;

  uint8_t upcount = 0,
          downcount = 0;

  Serial.begin(115200);
  pinMode(SW, OUTPUT);
  pinMode(TEMP_SENSOR, INPUT_ANALOG);
  unsigned int state = 0;
  for (;;) {
    input = analogRead(TEMP_SENSOR);                //read from rotary encoder connected to A0
    input = input * 0.12207031;
    Serial.print(input); Serial.print(" ");

    if (setPoint + BA <= input) {
      upcount++; //mencegah lonjakan acak
      if (upcount > 10) {
        state = 0;
        upcount = 0;
      }
    }
    else if (setPoint - BB >= input) {
      downcount++;//mencegah lonjakan acak
      if (downcount > 0) {
        state = 1;
        downcount = 0;
      }
    }
    Serial.println(state);
    digitalWrite(SW, state);
    vTaskDelay(100);
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
