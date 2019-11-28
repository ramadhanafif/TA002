//pin
#define sensor 34
#define relay 4

//koefisien PID
#define kp 2.08
#define ki 1.67
#define kd 2.15;

//variable PID
float p, i, d, pid;
float error, errorx, sumerr;

//set point = 50
#define sp 100.0

//variable suhu
float vout, suhu;

//pwm relay
//uint32_t vv = 10; //persenan
float persen;

void setup()
{
  xTaskCreate(MyTask1, "Task1", 1000, NULL, 1, NULL);
  xTaskCreate(MyTask2, "Task2", 1000, NULL, 2, NULL);
}


void loop()
{
  // DO nothing
}


/* Task1 with priority 1 */
static void MyTask1(void* pvParameters)
{
  pinMode(relay, OUTPUT);
  while (1)
  {
    digitalWrite(relay, HIGH);
    delay(20 * persen);
    digitalWrite(relay, LOW);
    delay(20 * (100 - persen));
  }
}


/* Task2 with priority 2 */
static void MyTask2(void* pvParameters)
{
  Serial.begin(115200);
  while (1)
  {
    vout = analogRead(sensor); //pembacaan sensor
    suhu = (vout * 500) / 4095; //suhu dalam celcius

    //PID
    error = sp - suhu;
    p = error * kp;
    sumerr = error + errorx;
    i = ki * sumerr;
    d = error - errorx;
    pid = p + i + d;
    pid = 255.0 - pid;

    if (pid < 1) {
      pid = 0;
    }

    vTaskSuspendAll();
    persen = pid / 2.55;
    if (persen > 100) {
      persen = 100;
    }
    persen = 100 - persen;
    xTaskResumeAll();
    
    //display
    //  Serial.print("suhu= ");
    Serial.print(suhu);
    Serial.print(";");
    Serial.println(persen);

    delay(1000);

    errorx = error;
  }
}
