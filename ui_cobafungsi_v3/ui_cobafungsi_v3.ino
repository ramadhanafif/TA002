/*---------------------------------------------------------------------*/
/*-----------------------------LIBRARIES-------------------------------*/
/*---------------------------------------------------------------------*/
#include "Wire.h"                 // I2C untuk LCD
#include "LiquidCrystal_I2C.h"    // include library LCD I2C
#include "SimpleKalmanFilter.h"   // include library Kalman Filter

#include <OneWire.h>
#include <DallasTemperature.h>

/*---------------------------------------------------------------------*/
/*--------------------------CONSTANTS & PINS----------------------------*/
/*---------------------------------------------------------------------*/
#define columnLength 20
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

/*MAIN State Definitions*/
#define STATE_INIT -1
#define STATE_INPUT_TEMP 0
#define STATE_INPUT_RPM 1
#define STATE_INPUT_JAM 2
#define STATE_INPUT_MENIT 3
#define STATE_WAIT_NEXT 4
#define STATE_CONFIRM 5
#define STATE_START_PROCESS 6
#define STATE_PANAS_AWAL 7
#define STATE_START_ROT 8
#define STATE_PAUSE 9
#define STATE_DONE 10


/*---------------------------------------------------------------------*/
/*------------------------------TIMER----------------------------------*/
/*---------------------------------------------------------------------*/
hw_timer_t * timer = NULL;


/*---------------------------------------------------------------------*/
/*---------------------------TASK HANDLER------------------------------*/
/*---------------------------------------------------------------------*/
TaskHandle_t TaskHandle_Pause;
TaskHandle_t TaskHandle_Input;
TaskHandle_t TaskHandle_SpeadRead;

/*---------------------------------------------------------------------*/
/*-----------------------------VARIABLES-------------------------------*/
/*---------------------------------------------------------------------*/
// universal needs
int stateCondition = STATE_INIT;
int temperatur = temConstant;
int kecepatan = kecConstant;
int jam = jamConstant;
int menit = menConstant;
unsigned int durasi = 0;

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
const int MTR_freq = 256000;
const int MTR_pwmChannel = 0;
const int MTR_resolution = 8;

// PID controller
int MTR_speed_req = 0;    // in rpm
float MTR_speed_actual = 0;   // in rpm
double MTR_Kp = 0.5;
double MTR_Kd = 0.01;
double MTR_Ki = 0.03;
float MTR_error = 0;
float MTR_last_error = 0;
float MTR_sum_error = 0;
int MTR_PWM_val = 0;
float MTR_pidTerm = 0;

// for the encoder
double MTR_newposition;
double MTR_oldposition = 0;
double MTR_vel;
volatile double MTR_encoderMotorValue = 0;

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

// create frame for loading bar
byte frame[4][8] = {
  {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F},
  {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10},
  {0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
}


/*---------------------------------------------------------------------*/
/*----------------------------TIMER ISR--------------------------------*/
/*---------------------------------------------------------------------*/
void IRAM_ATTR onTimer() {
  if ((stateCondition == STATE_START_ROT) && (durasi != timerCounter)) {
    timerCounter++;
    Serial.println(timerCounter);
  } else if ((stateCondition == STATE_START_ROT) && (durasi == timerCounter)) {
    // Serial.println("Timer Done");
    stateCondition = STATE_DONE;
    timerCounter++;
  }
}


/*---------------------------------------------------------------------*/
/*-------------------------------SETUP---------------------------------*/
/*---------------------------------------------------------------------*/
void setup() {
  Serial.begin (112500);

  // timer
  timer = timerBegin(0, 80000, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);

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
    NULL,                     /* Prameter passed as input of the task */
    3,                        /* Priority of the task. */
    &TaskHandle_Input);       /* Task handle. */

  xTaskCreate(
    taskInput,                /* Task function. */
    "TaskInput",              /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Prameter passed as input of the task */
    3,                        /* Priority of the task. */
    &TaskHandle_Input);       /* Task handle. */

  xTaskCreate(
    TaskPMNS,                 /* Task function. */
    "TaskPMNS",               /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    4,                        /* Priority of the task. */
    NULL);                    /* Task handle. */

  xTaskCreate(
    TaskPWMPMNS,              /* Task function. */
    "TaskPWMPMNS",            /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    4,                        /* Priority of the task. */
    NULL);                    /* Task handle. */

  xTaskCreate(
    taksPause,                /* Task function. */
    "TaskPause",              /* String with name of task. */
    10000,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    1,                        /* Priority of the task. */
    &TaskHandle_Pause);       /* Task handle. */

  taskDisplay(NULL);
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
      stateCondition = STATE_INIT;
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
        stateCondition = STATE_PAUSE;
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
  lcd.createChar(0, arrow);           // arrow
  lcd.createChar(1, loadingBar[0]);   // loading bar |
  lcd.createChar(2, loadingBar[1]);   // loading bar ||
  lcd.createChar(3, loadingBar[2]);   // loading bar |||
  lcd.createChar(4, loadingBar[3]);   // loading bar ||||
  lcd.createChar(5, loadingBar[4]);   // loading bar |||||
  lcd.createCHar(6, frame[0]);        // frame right
  lcd.createCHar(6, frame[1]);        // frame bottom
  lcd.createCHar(6, frame[2]);        // frame left
  lcd.createCHar(6, frame[3]);        // frame top

  unsigned int value = 0; //ini value apa

  for (;;) {
    switch (stateCondition) {
      case STATE_INIT: {
          vTaskResume(TaskHandle_Input);
          vTaskSuspend(TaskHandle_Pause);
          vTaskSuspend(TaskHandle_SpeadRead);
          MTR_speed_req = 0;
          lcd.clear();
          stateCondition++;
        } break;
      case STATE_INPUT_TEMP: {
          vTaskResume(TaskHandle_Input);
          vTaskSuspend(TaskHandle_Pause);
          temperatur = ((encoderValue / 4) % 66) + 25;
          printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        } break;
      case STATE_INPUT_RPM: {
          kecepatan = (encoderValue / 4) % 71 + 10;
          printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        } break;
      case STATE_INPUT_JAM: {
          jam = (encoderValue / 4) % 25;
          printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        } break;
      case STATE_INPUT_MENIT: {
          menit = (encoderValue / 4) % 60;
          printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        } break;
      case STATE_WAIT_NEXT: {
          if (forward) {
            lcd.clear();
            stateCondition ++;
          } else {
            lcd.clear();
            stateCondition --;
          }
        } break;
      case STATE_CONFIRM: {
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
        } break;
      case STATE_START_PROCESS: {
          if (startProcess) {
            stateCondition = STATE_PANAS_AWAL;
            lcd.clear();
          }
          else {
            forward = 1;
            stateCondition = STATE_INIT;
          }
        } break;
      case STATE_PANAS_AWAL: {
          // print frame for loading bar
          lcd.setCursor(0, 1);
          for(int i = 0; i < 20; i++) {
            lcd.write(byte(9));
          }
          lcd.setCursor(0, 2);
          lcd.write(byte(8));
          lcd.setCursor(19, 2);
          lcd.write(byte(6));
          lcd.setCursor(0, 3);
          for(int i = 0; i < 20; i++) {
            lcd.write(byte(7));
          }
          
          // local variable
          unsigned int piece;
          double percent;

          lcd.setCursor(0, 0);
          lcd.print("Memanaskan");

          //PERINTAH PANAS MASUK SINI


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

          lcd.setCursor(position, 2);
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
              } break;
            case 1: {
                lcd.write(byte(2));
              } break;
            case 2: {
                lcd.write(byte(3));
              } break;
            case 3: {
                lcd.write(byte(4));
              } break;
            case 4: {
                lcd.write(byte(5));
              }
              break;
          }
        } break;
      case STATE_START_ROT: {  // ini kalo gw taro di bawahnya case 7 ga bisa, tolong benerin!!
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
          timerAlarmEnable(timer);
          MTR_speed_req = kecepatan;
          // vTaskResume(TaskHandle_Timer);
          vTaskResume(TaskHandle_Pause);
          vTaskSuspend(TaskHandle_Input);
        } break;
      case STATE_PAUSE: {  // pause
          MTR_speed_req = 0;
          // vTaskSuspend(TaskHandle_Timer);
          vTaskSuspend(TaskHandle_SpeadRead);
          // Serial.println("Timer Pause");
        } break;
      case STATE_DONE: { // timer done
          MTR_speed_req = 0;
          // vTaskSuspend(TaskHandle_Timer);
          vTaskSuspend(TaskHandle_SpeadRead);
          vTaskSuspend(TaskHandle_Pause);
          // Serial.println("Timer Done");
        } break;
    }
    // Serial.println("Task Display");
    vTaskDelay(100);
  }
}

void taskPWMCalculator(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  ledcSetup(MTR_pwmChannel, MTR_freq, MTR_resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwm, MTR_pwmChannel);

  for (;;) {  // A Task shall never return or exit.
    // PID calculation
    if (MTR_speed_req == 0) {
      // PWM signal
      ledcWrite(MTR_pwmChannel, 0);
    } else {
      MTR_error = MTR_speed_req - MTR_speed_actual;
      MTR_pidTerm = (MTR_Kp * MTR_error) + (MTR_Kd * (MTR_error - MTR_last_error)) + MTR_sum_error * MTR_Ki + 185;
      MTR_last_error = MTR_error;
      MTR_sum_error += MTR_error;
      MTR_sum_error = constrain(MTR_sum_error, -2000, 2000);
      MTR_PWM_val = constrain(MTR_pidTerm, 0, 255);

      // PWM signal
      ledcWrite(MTR_pwmChannel, MTR_PWM_val);

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
    // every 1 rotation MTR_encoderMotorValue equal to 22
    MTR_newposition = MTR_encoderMotorValue / 22;
    MTR_vel = (MTR_newposition - MTR_oldposition);
    MTR_oldposition = MTR_newposition;

    float real_valueRPS = MTR_vel * 50;    // rps
    // Serial.print(real_valueRPS);
    // Serial.print(" ");

    // filtering the sensor read
    real_valueRPS = simpleKalmanFilter.updateEstimate(real_valueRPS);

    // speed from slow speed gear
    float real_valueRPM = (real_valueRPS / 46.8512) * 60;
    MTR_speed_actual = real_valueRPM;

    // Serial.println("Task Speed Read");
    //  printMotorInfo();
  }
}


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
  // Serial.print("Setpoint: ");    Serial.println(MTR_speed_req);
  // Serial.print("Speed RPM: ");    Serial.println(MTR_speed_actual);
  // Serial.print("MTR_error: ");     Serial.println(MTR_error);
  // Serial.print("last error: ");     Serial.println(MTR_last_error);
  // Serial.print("sum error: ");     Serial.println(MTR_sum_error);
  // Serial.print("MTR_PWM_val: ");      Serial.println(MTR_PWM_val);
  // Serial.print("PID Term: ");     Serial.println(MTR_MTR_pidTerm);
  Serial.print(MTR_speed_req);
  Serial.print("\t");
  Serial.println(MTR_speed_actual);
}

// interrupt when any change happen
void updateEncoderMotor() {
  MTR_encoderMotorValue ++;
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
