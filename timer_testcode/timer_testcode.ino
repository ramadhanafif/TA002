volatile uint32_t timerCounter = 0;
volatile boolean  timer_int_flag = 0;

//Interrupt Function
void IRAM_ATTR onTimer() {
  timerCounter++;
  timer_int_flag = 1;
}

void setup() {
  Serial.begin(115200);
  hw_timer_t* timer = NULL;
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
}

void loop() {
  if (timer_int_flag == 1) {
    Serial.println(timerCounter);
    timer_int_flag = 0;
  }
}
