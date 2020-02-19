/*---------------------------------------------------------------------*/
/*-----------------------------LIBRARIES-------------------------------*/
/*---------------------------------------------------------------------*/
#include "Wire.h"
#include "LiquidCrystal_I2C.h"    // include library LCD I2C
#include "SimpleKalmanFilter.h"   // include library Kalman Filter


/*---------------------------------------------------------------------*/
/*--------------------------CONSTANS & PINS----------------------------*/
/*---------------------------------------------------------------------*/
#define columnLength 20.0
#define temConstant 25
#define kecConstant 0
#define jamConstant 0
#define menConstant 0
#define constantEncoderVal 28116000
#define encoderPin1 19
#define encoderPin2 18
#define switchPinGreen 5
#define switchPinYellow 17
#define switchPinWhite 16
#define switchPinBlack 4
#define pwm 2 
#define encoderMotor 15


/*---------------------------------------------------------------------*/
/*------------------------------TIMER----------------------------------*/
/*---------------------------------------------------------------------*/
hw_timer_t * timer = NULL;


/*---------------------------------------------------------------------*/
/*---------------------------TASK HANDLES------------------------------*/
/*---------------------------------------------------------------------*/
// TaskHandle_t TaskHandle_Timer;
TaskHandle_t TaskHandle_Pause;
TaskHandle_t TaskHandle_Input;
TaskHandle_t TaskHandle_SpeadRead;

/*---------------------------------------------------------------------*/
/*-----------------------------VARIABLES-------------------------------*/
/*---------------------------------------------------------------------*/
// universal needs
int stateCondition = -1;
int temperatur = temConstant;
int kecepatan = kecConstant;
int jam = jamConstant;
int menit = menConstant;
unsigned int durasi = jam * 3600 + menit * 60;

boolean currentButtonStateGreen;
boolean lastButtonStateGreen = LOW;
boolean currentButtonStateYellow;
boolean lastButtonStateYellow = LOW;
boolean currentButtonStateWhite;
boolean lastButtonStateWhite = LOW;
boolean currentButtonStateBlack;
boolean lastButtonStateBlack = LOW;

volatile int lastEncoded = 0;
volatile unsigned long encoderValue = constantEncoderVal;
long lastencoderValue = 0; // store the value of rotary encoder
int lastMSB = 0;
int lastLSB = 0;

// timer
volatile int timerCounter;

// setting PWM properties
const int freq = 256000;
const int pwmChannel = 0;
const int resolution = 8;

// PID controller
int speed_req = 0;        // in rpm
float speed_actual = 0;   // in rpm
// double Kp = 12;
// double Kd = 12;
// double Ki = 0.7;
double Kp = 0.5;
double Kd = 0.01;
double Ki = 0.03;
float error = 0;
float last_error = 0;
float sum_error = 0;
int PWM_val = 0;
float pidTerm = 0;

// for the encoder
double newposition;
double oldposition = 0;
double vel;
volatile double encoderMotorValue = 0;

// flag for LCD state needs
boolean forward = 1;
boolean startProcess = 1;
boolean pauseState = 0;


/*---------------------------------------------------------------------*/
/*------------------------------OBJECTS--------------------------------*/
/*---------------------------------------------------------------------*/
// SimpleKalmanFilter(e_mea, e_est, q);
// e_mea: Measurement Uncertainty 
// e_est: Estimation Uncertainty 
// q: Process Noise
SimpleKalmanFilter simpleKalmanFilter(3, 3, 0.1);

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);


/*---------------------------------------------------------------------*/
/*-----------------------------CHAR LIBS-------------------------------*/
/*---------------------------------------------------------------------*/
// create arrow char
byte arrow[8] = {0x03, 0x07, 0x0F, 0x1F, 0x0F, 0x07, 0x03, 0x00};

// create char for loading bar
byte loadingBar[5][8] = {
  {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10},
  {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
  {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C},
  {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E},
  {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}
  };


/*---------------------------------------------------------------------*/
/*---------------------------------------------------------------------*/
/*---------------------------------------------------------------------*/
void IRAM_ATTR onTimer() {
  if ((stateCondition == 8) && (durasi != timerCounter)) {
    timerCounter++;
    Serial.println(timerCounter);
  } else if ((stateCondition == 8) && (durasi == timerCounter)) {
    // Serial.println("Timer Done");
    stateCondition = 10;
  }
}


/*---------------------------------------------------------------------*/
/*-------------------------------SETUP---------------------------------*/
/*---------------------------------------------------------------------*/
void setup() {
  Serial.begin (112500);

  // timer
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

  xTaskCreate(
    taskSpeedRead_rpm,        /* Task function. */
    "SpeedRead_rpm",          /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    3,                        /* Priority of the task. */
    &TaskHandle_SpeadRead );  /* Task handle. */

  xTaskCreate(
    taskPWMCalculator,        /* Task function. */
    "PWMCalculator",          /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    2,                        /* Priority of the task. */
    NULL);                    /* Task handle. */

  xTaskCreate(
    taskInput,                /* Task function. */
    "TaskInput",              /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    3,                        /* Priority of the task. */
    &TaskHandle_Input);       /* Task handle. */
  
  // xTaskCreate(
  //   taskTimer,                /* Task function. */
  //   "TaskTimer",              /* String with name of task. */
  //   10000,                    /* Stack size in bytes. */
  //   NULL,                     /* Parameter passed as input of the task */
  //   4,                        /* Priority of the task. */
  //   &TaskHandle_Timer);       /* Task handle. */

  xTaskCreate(
    taksPause,                /* Task function. */
    "TaskPause",              /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    1,                        /* Priority of the task. */
    &TaskHandle_Pause);       /* Task handle. */

  //  xTaskCreate(
  //    taskDisplay,      /* Task function. */
  //    "TaskTwo",        /* String with name of task. */
  //    10000,            /* Stack size in bytes. */
  //    NULL,             /* Parameter passed as input of the task */
  //    10,               /* Priority of the task. */
  //    NULL);            /* Task handle. */
  taskDisplay(NULL);
  // vTaskSuspend(TaskHandle_Timer);
  vTaskSuspend(TaskHandle_SpeadRead);
  vTaskSuspend(TaskHandle_Input);
  vTaskSuspend(TaskHandle_Pause);
}


/*---------------------------------------------------------------------*/
/*-------------------------------LOOP----------------------------------*/
/*---------------------------------------------------------------------*/
void loop() {
  // vTaskDelay(portMAX_DELAY);
}


/*---------------------------------------------------------------------*/
/*-------------------------------TASKS---------------------------------*/
/*---------------------------------------------------------------------*/
void taskInput( void * parameter )
{
  // initialize the rotary encoder
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(switchPinGreen, INPUT_PULLUP);
  pinMode(switchPinWhite, INPUT_PULLUP);
  pinMode(switchPinBlack, INPUT_PULLUP);

  for ( ; ; ) {
    // push button action
    currentButtonStateGreen = digitalRead(switchPinGreen);
    vTaskDelay(10);
    if (currentButtonStateGreen == HIGH && lastButtonStateGreen == LOW) {
      //button is not being pushed
      //do nothing
    } else if (currentButtonStateGreen == LOW && lastButtonStateGreen == HIGH) {
      //button is being pushed
      stateCondition ++;
      encoderValue = constantEncoderVal;
      forward = 1;
    }
    lastButtonStateGreen = currentButtonStateGreen;

    currentButtonStateBlack = digitalRead(switchPinBlack);
    vTaskDelay(10);
    if (currentButtonStateBlack == HIGH && lastButtonStateBlack == LOW) {
      //button is not being pushed
      //do nothing
    } else if (currentButtonStateBlack == LOW && lastButtonStateBlack == HIGH) {
      //button is being pushed
      stateCondition --;
      encoderValue = constantEncoderVal;
      forward = 0;
    }
    lastButtonStateBlack = currentButtonStateBlack;

    currentButtonStateWhite = digitalRead(switchPinWhite);
    vTaskDelay(10);
    if (currentButtonStateWhite == HIGH && lastButtonStateWhite == LOW) {
      //button is not being pushed
      //do nothing
    } else if (currentButtonStateWhite == LOW && lastButtonStateWhite == HIGH) {
      //button is being pushed
      stateCondition = -1;
      encoderValue = constantEncoderVal;
      temperatur = temConstant;
      kecepatan = kecConstant;
      jam = jamConstant;
      menit = menConstant;
      //encoderValue = constantEncoderVal;
    }
    lastButtonStateWhite = currentButtonStateWhite;

    // Serial.println("Task Input");
  }
}

void taksPause( void * paramaeter)
{
  pinMode(switchPinYellow, INPUT_PULLUP);

  for ( ; ; ) {
    // push button action
    currentButtonStateYellow = digitalRead(switchPinYellow);
    vTaskDelay(30);
    if (currentButtonStateYellow == HIGH && lastButtonStateYellow == LOW) {
      //button is not being pushed
      //do nothing
    } else if (currentButtonStateYellow == LOW && lastButtonStateYellow == HIGH) {
      //button is being pushed
      if (pauseState == 0) {
        stateCondition = 9;
        pauseState = 1;
      } else {
        stateCondition--;
        pauseState = 0;
      } 
    }
    lastButtonStateYellow = currentButtonStateYellow;

    // Serial.println("Task Pause");
  }
}

void taskDisplay( void * parameter)
{
  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(encoderPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderPin2, updateEncoder, CHANGE);
  
  // initialize the LCD
  lcd.begin();
  lcd.createChar(0, arrow);
  lcd.createChar(1, loadingBar[0]);
  lcd.createChar(2, loadingBar[1]);
  lcd.createChar(3, loadingBar[2]);
  lcd.createChar(4, loadingBar[3]);
  lcd.createChar(5, loadingBar[4]);

  unsigned int value = 0;

  for (;;) {
    switch (stateCondition) {
      case -1: {
        vTaskResume(TaskHandle_Input);
        // vTaskSuspend(TaskHandle_Timer);
        vTaskSuspend(TaskHandle_Pause);
        vTaskSuspend(TaskHandle_SpeadRead);
        speed_req = 0;
        lcd.clear();
        stateCondition++;
      }break;
      case 0: {
        vTaskResume(TaskHandle_Input);
        vTaskSuspend(TaskHandle_Pause);
        temperatur = ((encoderValue / 4) % 66) + 25;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
      }break;
      case 1: {
        kecepatan = (encoderValue / 4) % 71 + 10;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
      }break;
      case 2: {
        jam = (encoderValue / 4) % 25;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
      }break;
      case 3: {
        menit = (encoderValue / 4) % 60;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
      }break;
      case 4: {
        if (forward) {
          lcd.clear();
          stateCondition ++;
        } else {
          lcd.clear();
          stateCondition --;
        }
      }break;
      case 5: {
        lcd.setCursor(0, 0);
        lcd.print("Lanjutkan Pengadukan");
        lcd.setCursor(3, 1);
        lcd.print("Ya");
        lcd.setCursor(3, 2);
        lcd.print("Tidak");
        if (((encoderValue / 4) % 2) == 0) {
          lcd.setCursor(9, 1);
          lcd.write(byte(0));
          lcd.setCursor(9, 2);
          lcd.print(" ");
          startProcess = 1;
          // Serial.println(startProcess);
        }
        else {
          lcd.setCursor(9, 1);
          lcd.print(" ");
          lcd.setCursor(9, 2);
          lcd.write(byte(0));
          startProcess = 0;
          // Serial.println(startProcess);
        }
      }break;
      case 6: {
        if (startProcess) {
          stateCondition = 7;
          lcd.clear();
        }
        else {
          forward = 1;
          stateCondition = -1;
        }
      }break;
      case 7: {
        // local variable
        unsigned int piece;
        double percent;

        lcd.setCursor(0,0);
        lcd.print("Memanaskan");

        // calculation from sensor read
        vTaskDelay(10);
        value += 1;
        value = constrain(value, 0, 99);

        if (value == 99) {
          stateCondition++;
          value = 0;
        }

        percent = value;
        double position = (columnLength / 100 * percent);

        lcd.setCursor(position,2);
        piece = (int)(position * 5) % 5;
    
        // drawing charater's colums
        // if (piece == 0) {
        //   lcd.write(byte(1));
        // } else if (piece == 1) {
        //   lcd.write(byte(2));
        // } else if (piece == 2) {
        //   lcd.write(byte(3));
        // } else if (piece == 3) {
        //   lcd.write(byte(4));
        // } else {
        //   lcd.write(byte(5));          
        // }
        
        switch (piece) {
          case 0: {
            lcd.write(byte(1));
          }break;
          case 1: {
            lcd.write(byte(2));
          }break;
          case 2: {
            lcd.write(byte(3));
          }break;
          case 3: {
            lcd.write(byte(4));
          }break;
          case 4: {
            lcd.write(byte(5));
          }break;
        }
      }break;
      case 8: {  // ini kalo gw taro di bawahnya case 7 ga bisa, tolong benerin!!
        // Serial.print(temperatur);
        // Serial.print(" ");
        // Serial.print(kecepatan);
        // Serial.print(" ");
        // Serial.print(jam);
        // Serial.print(" ");
        // Serial.println(menit);
        lcd.clear();
        vTaskResume(TaskHandle_SpeadRead);
        durasi = jam * 3600 + menit * 60;
        speed_req = kecepatan;        
        // vTaskResume(TaskHandle_Timer);
        vTaskResume(TaskHandle_Pause);
        vTaskSuspend(TaskHandle_Input);
      }break;
      case 9: {  // pause
        speed_req = 0;
        // vTaskSuspend(TaskHandle_Timer);
        vTaskSuspend(TaskHandle_SpeadRead);
        Serial.println("Timer Pause");
      }break;
      case 10: { // timer done
        speed_req = 0;
        // vTaskSuspend(TaskHandle_Timer);
        vTaskSuspend(TaskHandle_SpeadRead);
        vTaskSuspend(TaskHandle_Pause);
        Serial.println("Timer Done");        
      }break;      
    }
    // Serial.println("Task Display");
    vTaskDelay(100);
  }
}

void taskPWMCalculator(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwm, pwmChannel);

  for (;;) {  // A Task shall never return or exit.
    // PID calculation
    if (speed_req == 0) {
      // PWM signal
      ledcWrite(pwmChannel, 0);
    } else {
      error = speed_req - speed_actual;
      pidTerm = (Kp * error) + (Kd * (error - last_error)) + sum_error * Ki + 185;
      last_error = error;
      sum_error += error;
      sum_error = constrain(sum_error, -2000, 2000);
      PWM_val = constrain(pidTerm, 0, 255);
      
      // PWM signal
      ledcWrite(pwmChannel, PWM_val);

      // printMotorInfo();
      // Serial.println("Task PWM Calculator");
    }
    vTaskDelay(40);
  }   
}

void taskSpeedRead_rpm(void *pvParameters)  // This is a task.
{
  pinMode(encoderMotor, INPUT_PULLUP);
  attachInterrupt(encoderMotor, updateEncoderMotor, CHANGE);

  TickType_t xLastWakeTimeSpeedRead;
  const TickType_t xFrequency = 20;   // program will run every 20ms

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTimeSpeedRead = xTaskGetTickCount();

  for (;;) {  // A Task shall never return or exit.
    vTaskDelayUntil( &xLastWakeTimeSpeedRead, xFrequency );       

    // motor use gear ratio 1 : 46.8512
    // speed from high speed gear
    // every 1 rotation encoderMotorValue equal to 22
    newposition = encoderMotorValue / 22;
    vel = (newposition - oldposition);
    oldposition = newposition;
    
    float real_valueRPS = vel * 50;    // rps
    // Serial.print(real_valueRPS);
    // Serial.print(" ");

    // filtering the sensor read
    real_valueRPS = simpleKalmanFilter.updateEstimate(real_valueRPS);

    // speed from slow speed gear
    float real_valueRPM = (real_valueRPS / 46.8512) * 60;
    speed_actual = real_valueRPM;

    // Serial.println("Task Speed Read");
    //  printMotorInfo();
  }
}

// void taskTimer(void* v) {
//   unsigned int counterTick = 0;
//   TickType_t xLastWakeTimeTimer = xTaskGetTickCount();
//   for (;;) {
//     vTaskDelayUntil( &xLastWakeTimeTimer, 1000);
//     //Recursive part
//     if (counterTick == durasi) {
//       vTaskSuspend(NULL);
//     }
//     if (counterTick < durasi) {
//       counterTick++;
//     } else {
//       stateCondition = 10;
//     }

//     // Serial.println("Task Timer");
//   }
// }


/*---------------------------------------------------------------------*/
/*------------------------------FUNCTIONS------------------------------*/
/*---------------------------------------------------------------------*/
// print the status to LCD
void printToLCD(int buffTemp, int buffKec, int buffJam, int buffMin, int buffSC) {
  // local array
  char tempVal[4];
  char spdVal[4];
  char minVal[4];
  char hourVal[4];

  // converting integer to string
  sprintf(tempVal, "%3d", buffTemp);
  sprintf(spdVal, "%3d", buffKec);
  sprintf(hourVal, "%3u", (buffJam % 25));
  sprintf(minVal, "%3u", (buffMin % 60));

  lcd.setCursor(0, 0);                    // show character at column 0, row 0
  lcd.print("Temperatur: ");              // show string on LCD
  lcd.setCursor(12, 0);                   // show character at column 12, row 0
  lcd.print(tempVal);                     // show the value
  lcd.setCursor(16, 0);                   // show character at column 16, row 0
  lcd.print((char)223);                   // show string "degree" on LCD
  lcd.setCursor(17, 0);                   // show character at column 17, row 0
  lcd.print("C");                         // show string on LCD
  if (buffSC == 0) {
    lcd.setCursor(19, 0);
    lcd.write(byte(0));
  } 
  else {
    lcd.setCursor(19, 0);
    lcd.print(" ");
  }

  lcd.setCursor(0, 1);                    // show character at column 0, row 1
  lcd.print("Kecepatan : ");              // show string on LCD
  lcd.setCursor(12, 1);                   // show character at column 12, row 1
  lcd.print(spdVal);                      // show the value
  lcd.setCursor(16, 1);                   // show character at column 16, row 1
  lcd.print("RPM");                       // show string on LCD
  if (buffSC == 1) {
    lcd.setCursor(19, 1);
    lcd.write(byte(0));
  } else {
    lcd.setCursor(19, 1);
    lcd.print(" ");
  }

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
  if (buffSC == 2) {
    lcd.setCursor(19, 2);
    lcd.write(byte(0));
  } else {
    lcd.setCursor(19, 2);
    lcd.print(" ");
  }

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
  if (buffSC == 3) {
    lcd.setCursor(19, 3);
    lcd.write(byte(0));
  } else {
    lcd.setCursor(19, 3);
    lcd.print(" ");
  }
}

// function for printing data
void printMotorInfo() {
  // Serial.print("Setpoint: ");    Serial.println(speed_req);
  // Serial.print("Speed RPM: ");    Serial.println(speed_actual);
  // Serial.print("error: ");     Serial.println(error);
  // Serial.print("last error: ");     Serial.println(last_error);
  // Serial.print("sum error: ");     Serial.println(sum_error);
  // Serial.print("PWM_val: ");      Serial.println(PWM_val);
  // Serial.print("PID Term: ");     Serial.println(pidTerm);
  Serial.print(speed_req);
  Serial.print("\t");
  Serial.println(speed_actual);
}

// interrupt when any change happen
void updateEncoderMotor(){
  encoderMotorValue ++;
}

// interrupt when any change happen
void updateEncoder() {
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
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
