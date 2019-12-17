  #include <MapleFreeRTOS900.h>

static void vLEDFlashTask(void *pvParameters) {
  pinMode(PA5, OUTPUT);
  for (;;) {
    vTaskDelay(200);
    digitalWrite(PA5, HIGH);
    vTaskDelay(300);
    digitalWrite(PA5, LOW);
  }
}

static void vLEDFlashTask2(void *pvParameters) {
  pinMode(PB1, OUTPUT);
  for (;;) {
    vTaskDelay(100);
    digitalWrite(PB1, HIGH);
    vTaskDelay(500);
    digitalWrite(PB1, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:

  xTaskCreate(vLEDFlashTask,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(vLEDFlashTask2,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  vTaskStartScheduler();

}

void loop() {
}
