#define BUZZER_PIN 32

//Handler untuk memastikan perintah buzzer hanya sekali
BaseType_t isBuzzerRing = pdFALSE;

void setup() {
  int repeat_ring = 10;      //Pola buzzer diulangi 3 kali


  if (isBuzzerRing == pdFALSE)
    isBuzzerRing = xTaskCreate(ringBuzz,
                               "Buzzer test!",
                               1000,
                               (void*) repeat_ring, //repeat pattern 3 times
                               tskIDLE_PRIORITY,
                               NULL);
}

void loop() {}

void ringBuzz (void* repeat) {
  pinMode(BUZZER_PIN, OUTPUT);

  for (int i = 0; i < int (repeat); i++) {
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(150);

    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(200);
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(50);

    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(200);
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(50);

    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(500);
  }

  digitalWrite(BUZZER_PIN, HIGH);
  vTaskDelay(500);
  digitalWrite(BUZZER_PIN, LOW);
  vTaskDelete(NULL);
}
