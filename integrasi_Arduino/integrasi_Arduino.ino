/* KODE PENGENDALI PEMANAS
   BOARD: BLACKPILL
*/

#include <MapleFreeRTOS900.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// include library Kalman Filter
#include "SimpleKalmanFilter.h"
// include library Median Filter
#include "MedianFilterLib.h"

//Semaphore and Mutex
SemaphoreHandle_t xPanas = NULL;
SemaphoreHandle_t xTimer = NULL;
SemaphoreHandle_t xAduk = NULL;

//pemanas
//Pin Definition
#define SW 13
#define TEMP_SENSOR 12
//PID constants
#define KP 10
#define KI 0.011

//pengaduk
#define pwm 11
#define dir1 10
#define dir2 9

//pemanas
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double input, output, setPoint;
double cumError;
double dutyCycle = 0;

//UI
// universal needs
int stateCondition = 0;
int temperatur = 25, kecepatan = 0;
unsigned int jam = 0;
unsigned int menit = 0;
boolean lastButtonState = LOW;
boolean currentButtonState;

// For the rotary encoder
int encoderPin1 = 2;
int encoderPin2 = 3;
int encoderSwitchPin = PB12; // push button rotary encoder
volatile int lastEncodedUI = 0;
volatile unsigned long encoderValueUI = 0;
long lastencoderValue = 0; // store the value of rotary encoder
int lastMSB = 0;
int lastLSB = 0;
// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

//pengaduk
// PID controller
int speed_req;    // in rpm
float speed_actual = 0;   // in rpm
double Kp = 9;
double Kd = 19;
double Ki = 0.03;
float error_mot = 0;
float last_error = 0;
float sum_error = 0;

int PWM_val = 0;
float pidTerm = 0;
volatile int lastEncoded = 0;

// for the encoder
#define enA 2
#define enB 3
double newposition;
double oldposition = 0;
double vel;
volatile double encoderValue = 0;

// SimpleKalmanFilter(e_mea, e_est, q);
// e_mea: Measurement Uncertainty
// e_est: Estimation Uncertainty
// q: Process Noise
SimpleKalmanFilter simpleKalmanFilter(3, 3, 0.1);

// MedianFilter<data_type>(window);
// window: Sample length
MedianFilter<float> medianFilter(5);


void TaskCompute(void* v) {
  if (xSemaphoreTake(xPanas, (TickType_t) portMAX_DELAY) == pdTRUE) {
    setPoint = temperatur;                          //set point at zero degrees
    pinMode(TEMP_SENSOR, INPUT_ANALOG);
    for (;;) {
      input = analogRead(TEMP_SENSOR);                //read from rotary encoder connected to A0
      input = input / 12.409;

      output = computePID(input);

      dutyCycle = (double) output / 500.0;
      if (dutyCycle > 1.0)
        dutyCycle = 1;
      else if (dutyCycle < 0.0)
        dutyCycle = 0;


      Serial.print(setPoint); Serial.print(';');
      Serial.print(input); Serial.println(';');
      //    Serial.println(dutyCycle * 1);
      vTaskDelay(1000);
    }
  }
}

void TaskSwitch(void* v) {
  pinMode(SW, OUTPUT);
  for (;;) {
    digitalWrite(SW, HIGH);
    vTaskDelay(200 * dutyCycle);
    digitalWrite(SW, LOW);
    vTaskDelay(200 * (1 - dutyCycle));
  }
}

void TaskTimer(void* v) {
  if (xSemaphoreTake(xTimer, (TickType_t) portMAX_DELAY) == pdTRUE) {
    unsigned int counter = 0;
    unsigned int durasi = jam * 3600 + menit * 60;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
      //Recursive part
      while (counter <= jam) {
        counter + 1;
      }
      vTaskDelayUntil( &xLastWakeTime, 1000); //Perlu dicoba untuk 0 jam 1 menit
    }
  }
}

void TaskUI(void* v) {
  // initialize the LCD
  lcd.begin();

  // initialize the rotary encoder
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);

  pinMode(encoderSwitchPin, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  digitalWrite(encoderSwitchPin, HIGH); //turn pullup resistor on

  //call updateEncoderUI() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(encoderPin1, updateEncoderUI, CHANGE);
  attachInterrupt(encoderPin2, updateEncoderUI, CHANGE);

  for (;;) {
    // local array
    char tempVal[4];
    char spdVal[4];
    char minVal[4];
    char hourVal[4];

    encoderValueUI = 0; // reset the value

    while (stateCondition == 0) {   // condition when temperature being set
      temperatur = (encoderValueUI / 4) + 25;

      // temperatur value must be between 25 up to 90 deg C
      if (temperatur < 25) {
        encoderValueUI = 0;
        temperatur = (encoderValueUI / 4) + 25;
        sprintf(tempVal, "%3d", temperatur);    // map int value to string
        lcd.setCursor(0, 0);                    // show character at column 0, row 0
        lcd.print("Temperatur: ");              // show string on LCD
        lcd.setCursor(12, 0);                   // show character at column 12, row 0
        lcd.print(tempVal);                     // show the value
        lcd.setCursor(16, 0);                   // show character at column 16, row 0
        lcd.print((char)223);                   // show string on LCD
        lcd.setCursor(17, 0);                   // show character at column 17, row 0
        lcd.print("C");                         // show string on LCD
        delayMicroseconds(1);
      } else if (temperatur > 90) {
        encoderValueUI = 65 * 4;
        temperatur = (encoderValueUI / 4) + 25;
        sprintf(tempVal, "%3d", temperatur);    // map int value to string
        lcd.setCursor(0, 0);                    // show character at column 0, row 0
        lcd.print("Temperatur: ");              // show string on LCD
        lcd.setCursor(12, 0);                   // show character at column 12, row 0
        lcd.print(tempVal);                     // show the value
        lcd.setCursor(16, 0);                   // show character at column 16, row 0
        lcd.print((char)223);                   // show string on LCD
        lcd.setCursor(17, 0);                   // show character at column 17, row 0
        lcd.print("C");                         // show string on LCD
        delayMicroseconds(1);
      } else {
        sprintf(tempVal, "%3d", temperatur);    // map int value to string
        lcd.setCursor(0, 0);                    // show character at column 0, row 0
        lcd.print("Temperatur: ");              // show string on LCD
        lcd.setCursor(12, 0);                   // show character at column 12, row 0
        lcd.print(tempVal);                     // show the value
        lcd.setCursor(16, 0);                   // show character at column 16, row 0
        lcd.print((char)223);                   // show string on LCD
        lcd.setCursor(17, 0);                   // show character at column 17, row 0
        lcd.print("C");                         // show string on LCD
        delayMicroseconds(1);
      }

      // Serial.println(temperatur);

      // push button action
      currentButtonState = digitalRead(encoderSwitchPin);
      delay(50);
      if (currentButtonState == HIGH && lastButtonState == LOW) {
        //button is not being pushed
        //do nothing
      } else if (currentButtonState == LOW && lastButtonState == HIGH) {
        //button is being pushed
        stateCondition ++;
        encoderValueUI = 0;
      }

      lastButtonState = currentButtonState;
    }

    while (stateCondition == 1) {   // condition when rpm motor being set
      // show the previous data
      lcd.setCursor(0, 0);                    // show character at column 0, row 0
      lcd.print("Temperatur: ");              // show string on LCD
      lcd.setCursor(12, 0);                   // show character at column 12, row 0
      lcd.print(tempVal);                     // show the value
      lcd.setCursor(16, 0);                   // show character at column 16, row 0
      lcd.print((char)223);                   // show string on LCD
      lcd.setCursor(17, 0);                   // show character at column 17, row 0
      lcd.print("C");                         // show string on LCD
      delayMicroseconds(1);

      kecepatan = encoderValueUI / 4;

      // kecepatan value mush be between 0 up to 70 rpm
      if (kecepatan < 30) {
        encoderValueUI = 30 * 4;
        kecepatan = encoderValueUI / 4;
        sprintf(spdVal, "%3d", kecepatan);      // map int value to string
        lcd.setCursor(0, 1);                    // show character at column 0, row 1
        lcd.print("Kecepatan : ");              // show string on LCD
        lcd.setCursor(12, 1);                   // show character at column 12, row 1
        lcd.print(spdVal);                      // show the value
        lcd.setCursor(16, 1);                   // show character at column 16, row 1
        lcd.print("RPM");                       // show string on LCD
        delayMicroseconds(1);
      } else if (kecepatan > 70) {
        encoderValueUI = 70 * 4;
        kecepatan = encoderValueUI / 4;
        sprintf(spdVal, "%3d", kecepatan);      // map int value to string
        lcd.setCursor(0, 1);                    // show character at column 0, row 1
        lcd.print("Kecepatan : ");              // show string on LCD
        lcd.setCursor(12, 1);                   // show character at column 12, row 1
        lcd.print(spdVal);                      // show the value
        lcd.setCursor(16, 1);                   // show character at column 16, row 1
        lcd.print("RPM");                       // show string on LCD
        delayMicroseconds(1);
      } else {
        sprintf(spdVal, "%3d", kecepatan);      // map int value to string
        lcd.setCursor(0, 1);                    // show character at column 0, row 1
        lcd.print("Kecepatan : ");              // show string on LCD
        lcd.setCursor(12, 1);                   // show character at column 12, row 1
        lcd.print(spdVal);                      // show the value
        lcd.setCursor(16, 1);                   // show character at column 16, row 1
        lcd.print("RPM");                       // show string on LCD
        delayMicroseconds(1);
      }

      // Serial.print(temperatur); Serial.print("  "); Serial.println(spdVal);

      // push button action
      currentButtonState = digitalRead(encoderSwitchPin);
      delay(50);
      if (currentButtonState == HIGH && lastButtonState == LOW) {
        //button is not being pushed
        //do nothing
      } else if (currentButtonState == LOW && lastButtonState == HIGH) {
        //button is being pushed
        stateCondition ++;
        encoderValueUI = 0;
      }

      lastButtonState = currentButtonState;
    }

    while (stateCondition == 2) {   // condition when hour is being set
      // show the previous data
      lcd.setCursor(0, 0);                    // show character at column 0, row 0
      lcd.print("Temperatur: ");              // show string on LCD
      lcd.setCursor(12, 0);                   // show character at column 12, row 0
      lcd.print(tempVal);                     // show the value
      lcd.setCursor(16, 0);                   // show character at column 16, row 0
      lcd.print((char)223);                   // show string "degree" on LCD
      lcd.setCursor(17, 0);                   // show character at column 17, row 0
      lcd.print("C");                         // show string on LCD
      delayMicroseconds(1);

      lcd.setCursor(0, 1);                    // show character at column 0, row 1
      lcd.print("Kecepatan : ");              // show string on LCD
      lcd.setCursor(12, 1);                   // show character at column 12, row 1
      lcd.print(spdVal);                      // show the value
      lcd.setCursor(16, 1);                   // show character at column 16, row 1
      lcd.print("RPM");                       // show string on LCD
      delayMicroseconds(1);

      jam = ((encoderValueUI / 4) + (25 * 1000));  // the value far away from zero
      menit = 0;

      // jam value mush be between 0 up to 23
      sprintf(hourVal, "%3u", (jam % 25));           // map int value to string
      sprintf(minVal, "%3u", menit);          // map int value to string
      lcd.setCursor(0, 2);                    // show character at column 0, row 2
      lcd.print("Durasi    : ");              // show string on LCD
      lcd.setCursor(12, 2);                   // show character at column 12, row 2
      lcd.print(hourVal);                     // show the value
      lcd.setCursor(16, 2);                   // show character at column 16, row 2
      lcd.print("JAM");                       // show string on LCD
      lcd.setCursor(12, 3);                   // show character at column 12, row 3
      lcd.print(minVal);                      // show the value
      lcd.setCursor(16, 3);                   // show character at column 16, row 3
      lcd.print("MEN");                       // show string on LCD
      delayMicroseconds(1);

      // push button action
      currentButtonState = digitalRead(encoderSwitchPin);
      delay(50);
      if (currentButtonState == HIGH && lastButtonState == LOW) {
        //button is not being pushed
        //do nothing
      } else if (currentButtonState == LOW && lastButtonState == HIGH) {
        //button is being pushed
        stateCondition ++;
        encoderValueUI = 0;
        jam = atoi(hourVal);
      }

      lastButtonState = currentButtonState;
    }

    while (stateCondition == 3) {       // condition when minute is being set
      // show the previous data
      lcd.setCursor(0, 0);                    // show character at column 0, row 0
      lcd.print("Temperatur: ");              // show string on LCD
      lcd.setCursor(12, 0);                   // show character at column 12, row 0
      lcd.print(tempVal);                     // show the value
      lcd.setCursor(16, 0);                   // show character at column 16, row 0
      lcd.print((char)223);                   // show string "degree" on LCD
      lcd.setCursor(17, 0);                   // show character at column 17, row 0
      lcd.print("C");                         // show string on LCD
      delayMicroseconds(1);

      lcd.setCursor(0, 1);                    // show character at column 0, row 1
      lcd.print("Kecepatan : ");              // show string on LCD
      lcd.setCursor(12, 1);                   // show character at column 12, row 1
      lcd.print(spdVal);                      // show the value
      lcd.setCursor(16, 1);                   // show character at column 16, row 1
      lcd.print("RPM");                       // show string on LCD
      delayMicroseconds(1);

      if (jam == 24) {
        menit = 0;
        sprintf(minVal, "%3u", (menit % 60));          // map int value to string
        lcd.setCursor(0, 2);                    // show character at column 0, row 2
        lcd.print("Durasi    : ");              // show string on LCD
        lcd.setCursor(12, 2);                   // show character at column 12, row 2
        lcd.print(hourVal);                     // show the value
        lcd.setCursor(16, 2);                   // show character at column 16, row 2
        lcd.print("JAM");                       // show string on LCD
        lcd.setCursor(12, 3);                   // show character at column 12, row 3
        lcd.print(minVal);                      // show the value
        lcd.setCursor(16, 3);                   // show character at column 16, row 3
        lcd.print("MEN");                       // show string on LCD
        delayMicroseconds(1);
      } else {
        menit = ((encoderValueUI / 4) + (60 * 100));  // the value far away from zero

        // jam value mush be between 0 up to 59
        sprintf(minVal, "%3u", (menit % 60));          // map int value to string
        lcd.setCursor(0, 2);                    // show character at column 0, row 2
        lcd.print("Durasi    : ");              // show string on LCD
        lcd.setCursor(12, 2);                   // show character at column 12, row 2
        lcd.print(hourVal);                     // show the value
        lcd.setCursor(16, 2);                   // show character at column 16, row 2
        lcd.print("JAM");                       // show string on LCD
        lcd.setCursor(12, 3);                   // show character at column 12, row 3
        lcd.print(minVal);                      // show the value
        lcd.setCursor(16, 3);                   // show character at column 16, row 3
        lcd.print("MEN");                       // show string on LCD
        delayMicroseconds(1);
      }

      // push button action
      currentButtonState = digitalRead(encoderSwitchPin);
      delay(50);
      if (currentButtonState == HIGH && lastButtonState == LOW) {
        //button is not being pushed
        //do nothing
      } else if (currentButtonState == LOW && lastButtonState == HIGH) {
        //button is being pushed
        stateCondition ++;
        encoderValueUI = 0;
        menit = atoi(minVal);

        //Release all semaphores
        xSemaphoreGive(xPanas);
        xSemaphoreGive(xAduk);
        xSemaphoreGive(xTimer);
      }

      lastButtonState = currentButtonState;
    }

    while (stateCondition == 4) {
      // push button action
      currentButtonState = digitalRead(encoderSwitchPin);
      delay(50);
      if (currentButtonState == HIGH && lastButtonState == LOW) {
        //button is not being pushed
        //do nothing
      } else if (currentButtonState == LOW && lastButtonState == HIGH) {
        //button is being pushed
        stateCondition ++;
        encoderValueUI = 0;
        menit = atoi(minVal);
      }

      lastButtonState = currentButtonState;
    }

    detachInterrupt(digitalPinToInterrupt(encoderPin1));
    detachInterrupt(digitalPinToInterrupt(encoderPin2));
  
    // lcd.clear();
  }

}

void setup() {
  Serial.begin(115200);

  //Create all semaphore
  xPanas = xSemaphoreCreateMutex();
  xTimer = xSemaphoreCreateMutex();
  xAduk = xSemaphoreCreateMutex();

  //Take all sempahore to block other function
  xSemaphoreTake(xPanas, (TickType_t) portMAX_DELAY);
  xSemaphoreTake(xTimer, (TickType_t) portMAX_DELAY);
  xSemaphoreTake(xAduk, (TickType_t) portMAX_DELAY);


  //pemanas
  xTaskCreate(TaskSwitch,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
  xTaskCreate(TaskCompute,
              "Task2",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(TaskUI,
              "Task3",
              configMINIMAL_STACK_SIZE + 128,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
  vTaskStartScheduler();

  //pengaduk
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  // pinMode(pwm, OUTPUT);
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);

  xTaskCreate(
    TaskSpeedRead_rpm
    ,  (const portCHAR *)"SpeedRead_rpm"    // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskPWMCalculator
    ,  (const portCHAR *)"PWMCalculator"    // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
}

void loop() {

}

// interrupt when any change happen
void updateEncoderUI() {
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncodedUI << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValueUI --;
  }

  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValueUI ++;
  }

  lastEncodedUI = encoded; //store this value for next time
}

//pemanas
double computePID(double inp) {
  currentTime = millis() / 1000;              //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                                // determine error
  cumError += error * elapsedTime;                // compute integral

  double out = KP * error + KI * cumError;// + kd * rateError;          //PID output

  previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
}

//pengaduk
void TaskPWMCalculator(void *pvParameters)  // This is a task.
{
  if (xSemaphoreTake(xAduk, (TickType_t) portMAX_DELAY) == pdTRUE) {
    //(void) pvParameters;

    pinMode(pwm, OUTPUT);
    speed_req = kecepatan;
    for (;;) // A Task shall never return or exit.
    {
      // speed from low rpm gear
      error_mot = speed_req - speed_actual;
      pidTerm = (Kp * error_mot) + (Kd * (error_mot - last_error)) + sum_error * Ki;
      last_error = error_mot;
      sum_error += error_mot;
      sum_error = constrain(sum_error, -4000, 4000);
      PWM_val = constrain(pidTerm, 39, 255);

      analogWrite(pwm, PWM_val);
      printMotorInfo();
      vTaskDelay(1);
    }
  }
}

void TaskSpeedRead_rpm(void *pvParameters)  // This is a task.
{
  //(void) pvParameters;

  pinMode(enA, INPUT);
  digitalWrite(enA, HIGH);       // turn on pullup resistor
  pinMode(enB, INPUT);
  digitalWrite(enB, HIGH);       // turn on pullup resistor
  attachInterrupt(enA, updateEncoderMotor, CHANGE);  // encoDER ON PIN 2
  attachInterrupt(enB, updateEncoderMotor, CHANGE);

  for (;;) // A Task shall never return or exit.
  {
    // motor use gear ratio 1 : 46.8512
    // speed from high rpm gear
    // Serial.println("haha");
    newposition = encoderValue / 50;
    vel = (newposition - oldposition);
    oldposition = newposition;

    float real_value = vel;

    // calculate the estimated value with Median Filter
    float median_value = medianFilter.AddValue(real_value);
    median_value = median_value * (1000 / 15) * 60 / 46.8512;

    // calculate the estimated RPM value with Kalman Filter
    float kalman_value = simpleKalmanFilter.updateEstimate(median_value);

    // filtered value for PID calculation
    speed_actual = kalman_value;

    detachInterrupt(digitalPinToInterrupt(enA));
    detachInterrupt(digitalPinToInterrupt(enB));

    vTaskDelay(1); // delay for 15 ms
  }
}

// function for printing data
void printMotorInfo() {
  Serial.print("Setpoint: ");    Serial.println(speed_req);
  Serial.print("Speed RPM: ");    Serial.println(speed_actual);
  Serial.print("error: ");     Serial.println(error_mot);
  Serial.print("last error: ");     Serial.println(last_error);
  Serial.print("sum error: ");     Serial.println(sum_error);
  Serial.print("PWM_val: ");      Serial.println(PWM_val);
  Serial.print("PID Term: ");     Serial.println(pidTerm);
  // // Serial.print(speed_req);
  // // Serial.print("\t");
  // Serial.println(speed_actual);
}

// interrupt when any change happen
void updateEncoderMotor() {
  int MSB = digitalRead(enB); //MSB = most significant bit
  int LSB = digitalRead(enA); //LSB = least significant bit
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue --;
  }

  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue ++;
  }

  lastEncoded = encoded; //store this value for next time
}
