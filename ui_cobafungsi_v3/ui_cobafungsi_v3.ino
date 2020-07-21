#define ENABLE_PRINT_DEBUG 1

/*---------------------------------------------------------------------*/
/*-----------------------------LIBRARIES-------------------------------*/
/*---------------------------------------------------------------------*/
#include "Wire.h"                 // I2C untuk LCD
#include "LiquidCrystal_I2C.h"    // include library LCD I2C
#include "SimpleKalmanFilter.h"   // include library Kalman Filter

//Dibawah ini library untuk pemanas
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MedianFilterLib.h"

/*---------------------------------------------------------------------*/
/*--------------------------CONSTANTS & PINS----------------------------*/
/*---------------------------------------------------------------------*/
#define columnLength  20
#define temConstant   25
#define kecConstant    0
#define jamConstant    0
#define menConstant    0
#define constantEncoderVal 28116000

#define encoderPin1     25//19
#define encoderPin2     26//18
#define switchPinGreen  27//5
#define switchPinYellow 13//17
#define switchPinWhite  12//16
#define switchPinBlack  14// 4
#define pwm             0// 2
#define encoderMotor    2// 15

//5,23,18,19

/*MAIN State Definitions*/
#define STATE_INIT         -1
#define STATE_INPUT_TEMP    0
#define STATE_INPUT_RPM     1
#define STATE_INPUT_JAM     2
#define STATE_INPUT_MENIT   3
#define STATE_WAIT_NEXT     4
#define STATE_CONFIRM       5
#define STATE_START_PROCESS 6
#define STATE_PANAS_AWAL    7
#define STATE_START_ROT     8
#define STATE_PAUSE         9
#define STATE_DONE         10

/* PEMANAS Definitions*/
#define TEMP_SENSOR_PIN   18//23
#define SSR_PIN           5//26

#define BB 0

#define PMNS_WAIT_TIME        45
#define PMNS_ON_TIME          20
#define PMNS_PERIOD_PWM       400
#define PMNS_SET_POINT_DEBUG  80

#define PMNS_STATE_START  0
#define PMNS_STATE_STEADY 1


#define BUZZER_PIN 33

MedianFilter<double> medianFilter(3);
/*---------------------------------------------------------------------*/
/*------------------------------TIMER----------------------------------*/
/*---------------------------------------------------------------------*/
hw_timer_t * timer = NULL;


/*---------------------------------------------------------------------*/
/*---------------------------TASK HANDLER------------------------------*/
/*---------------------------------------------------------------------*/
//Task Definitions
#define RUNNING 1
#define RESUME 1
#define SUSPEND 0
#define SUSPENDED 0

TaskHandle_t TaskHandle_Pause;
TaskHandle_t TaskHandle_Input;
TaskHandle_t TaskHandle_SpeadRead;
TaskHandle_t TaskHandle_PMNS;
TaskHandle_t TaskHandle_PWMCalculator;

bool IsRun_PMNS = RUNNING;
bool IsRun_SpeedRead_rpm = RUNNING;
bool IsRun_Input = RUNNING;
bool IsRun_Pause = RUNNING;
bool IsRun_PWMCalculator = RUNNING;

#define STACK_SIZE_PAUSE          1024
#define STACK_SIZE_PWMCalculator  1024
#define STACK_SIZE_INPUT          1024
#define STACK_SIZE_SPEEDREAD      1024
#define STACK_SIZE_PMNS           5000

#define PRIORITY_TASK_PAUSE           1
#define PRIORITY_TASK_PWMCalculator   2
#define PRIORITY_TASK_INPUT           3
#define PRIORITY_TASK_SPEEDREAD       3
#define PRIORITY_TASK_PMNS            4

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
volatile unsigned int timerCounter;

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
boolean flagForClearLCD = 0; // variable for clear the lcd at the STATE_START_ROT

// PEMANAS
unsigned int PMNS_pemanas_state = 0;
unsigned int PMNS_flag_pemanas_awal_done = 0;
double TempRead = 0;

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

//PEMANAS
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensor(&oneWire);

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
};


/*---------------------------------------------------------------------*/
/*----------------------------TIMER ISR--------------------------------*/
/*---------------------------------------------------------------------*/
void IRAM_ATTR onTimer() {
  if ((stateCondition == STATE_START_ROT) && (durasi != timerCounter)) {
    // Serial.println(timerCounter);
  } else if ((stateCondition == STATE_START_ROT) && (durasi == timerCounter)) {
    // Serial.println("Timer Done");
    stateCondition = STATE_DONE;
  }
  timerCounter++;
}


/*---------------------------------------------------------------------*/
/*-------------------------------SETUP---------------------------------*/
/*---------------------------------------------------------------------*/
void setup() {
  Serial.begin (112500);

  // timer
  pinMode(BUZZER_PIN, OUTPUT);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);

  xTaskCreate(
    taskSpeedRead_rpm,        /* Task function. */
    "SpeedRead_rpm",          /* String with name of task. */
    STACK_SIZE_SPEEDREAD,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    PRIORITY_TASK_SPEEDREAD,                        /* Priority of the task. */
    &TaskHandle_SpeadRead );  /* Task handle. */

#if ENABLE_PRINT_DEBUG
  xTaskCreate(
    taskPrint,        /* Task function. */
    "printing",          /* String with name of task. */
    2024,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    1,                        /* Priority of the task. */
    NULL );  /* Task handle. */
#endif

  xTaskCreate(
    taskPWMCalculator,        /* Task function. */
    "PWMCalculator",          /* String with name of task. */
    STACK_SIZE_PWMCalculator,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    PRIORITY_TASK_PWMCalculator,                        /* Priority of the task. */
    &TaskHandle_PWMCalculator);                    /* Task handle. */

  xTaskCreate(
    taskInput,                /* Task function. */
    "TaskInput",              /* String with name of task. */
    STACK_SIZE_INPUT,                    /* Stack size in bytes. */
    NULL,                     /* Prameter passed as input of the task */
    PRIORITY_TASK_INPUT,                        /* Priority of the task. */
    &TaskHandle_Input);       /* Task handle. */

  xTaskCreate(
    taskPMNS_MAIN,                 /* Task function. */
    "taskPMNS_MAIN",               /* String with name of task. */
    STACK_SIZE_PMNS,                    /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    PRIORITY_TASK_PMNS,                        /* Priority of the task. */
    &TaskHandle_PMNS);

  xTaskCreate(
    taskPause,                /* Task function. */
    "TaskPause",              /* String with name of task. */
    STACK_SIZE_PAUSE,         /* Stack size in bytes. */
    NULL,                     /* Parameter passed as input of the task */
    PRIORITY_TASK_PAUSE,                        /* Priority of the task. */
    &TaskHandle_Pause);       /* Task handle. */

  taskDisplay(NULL);
  // vTaskControl(TaskHandle_SpeadRead,&IsRun_SpeedRead_rpm,SUSPEND);
  // vTaskSuspend(TaskHandle_Input);
  // vTaskSuspend(TaskHandle_Pause);
  // vTaskSuspend(TaskHandle_PMNS);
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

  vTaskControl(TaskHandle_Input, &IsRun_Input, SUSPEND);
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

void taskPause( void * paramaeter)
{
  pinMode(switchPinYellow, INPUT_PULLUP);

  vTaskControl(TaskHandle_Pause, &IsRun_Pause, SUSPEND);
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

  unsigned int value = 0; //ini value apa

  for (;;) {
    switch (stateCondition) {
      case STATE_INIT: {
          vTaskControl(TaskHandle_Input, &IsRun_Input, RESUME);
          MTR_speed_req = 0;
          lcd.clear();
          stateCondition++;
        } break;
      case STATE_INPUT_TEMP: {
          temperatur = ((encoderValue / 4) % 66) + 25;
          printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
          // Serial.print(temperatur);
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
          // !!!!! buat bypass aja !!!!!
          // stateCondition++;
          
          // print frame for loading bar
          // lcd.createChar(1, frame[0]);        // frame right
          // lcd.createChar(2, frame[1]);        // frame bottom
          // lcd.createChar(3, frame[2]);        // frame left
          // lcd.createChar(4, frame[3]);        // frame top

          // lcd.setCursor(1, 1);
          // for (int i = 0; i < 18; i++) {
          //   lcd.write(byte(2));
          // }
          // lcd.setCursor(0, 2);
          // lcd.write(byte(1));
          // lcd.setCursor(19, 2);
          // lcd.write(byte(3));
          // lcd.setCursor(1, 3);
          // for (int i = 0; i < 18; i++) {
          //   lcd.write(byte(4));
          // }

          // buffer variable
          char bufferForPrintTemp[4];
          char bufferForprintTempRead[4];

          // convert to string
          sprintf(bufferForPrintTemp, "%3d", temperatur);
          sprintf(bufferForprintTempRead, "%3f", TempRead);


          lcd.setCursor(0, 0);
          lcd.print("Memanaskan");
          lcd.setCursor(0, 1);
          lcd.print("Suhu target: ");
          lcd.setCursor(14, 1);
          lcd.print(bufferForPrintTemp);
          lcd.setCursor(0, 2);
          lcd.print("Suhu aktual: ");
          lcd.setCursor(14, 2);
          lcd.print(bufferForprintTempRead);


          //PERINTAH PANAS MASUK SINI
          vTaskControl(TaskHandle_PMNS, &IsRun_PMNS, RESUME);
          PMNS_pemanas_state = PMNS_STATE_START;

          if (PMNS_flag_pemanas_awal_done == 1) {
            stateCondition = STATE_START_ROT;
            PMNS_pemanas_state = PMNS_STATE_STEADY;
            // percent = 100;
          }

          // // calculation from sensor read
          // vTaskDelay(10);
          // value += 1;
          // value = constrain(value, 0, 99);

          // if (value == 99) {
          //   stateCondition++;
          //   value = 0;
          // }

          // Serial.print(TempRead); Serial.print(" ");
          // Serial.println(percent);

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
        } break;
      case STATE_START_ROT: {
          if (!flagForClearLCD) {
            lcd.clear();
            flagForClearLCD = 1; 
          }

          // buffer variable
          char tempActual[4];
          char speedActual[4];
          char setTemp[4];
          char setSpeed[4];

          // convert to string
          sprintf(tempActual, "%3f", TempRead);
          sprintf(speedActual, "%3f", MTR_speed_actual);
          sprintf(tempActual, "%3d", temperatur);
          sprintf(speedActual, "%3d", kecepatan);

          lcd.setCursor(0, 0);
          lcd.print("Set point: ");
          lcd.setCursor(12, 0);
          lcd.print(setSpeed);
          lcd.print(setTemp);
          lcd.setCursor(0, 1);
          lcd.print("Kecepatan  : ");
          lcd.setCursor(14, 1);
          lcd.print(speedActual);
          lcd.setCursor(0, 2);
          lcd.print("Suhu actual: ");
          lcd.setCursor(14, 2);
          lcd.print(speedActual);
          lcd.setCursor(0, 3);
          lcd.print("Sisa waktu : ");
          lcd.setCursor(14, 3);
       

          vTaskControl(TaskHandle_SpeadRead, &IsRun_SpeedRead_rpm, RESUME);
          vTaskControl(TaskHandle_PWMCalculator, &IsRun_PWMCalculator, RESUME);
          durasi = jam * 3600 + menit * 60;

          timerAlarmEnable(timer);
          MTR_speed_req = kecepatan;
          vTaskControl(TaskHandle_Pause, &IsRun_Pause, RESUME);
          vTaskControl(TaskHandle_Input, &IsRun_Input, SUSPEND);
        } break;
      case STATE_PAUSE: {  // pause
          MTR_speed_req = 0;
        } break;
      case STATE_DONE: { // timer done
          MTR_speed_req = 0;
          vTaskControl(TaskHandle_SpeadRead, &IsRun_SpeedRead_rpm, SUSPEND);
          vTaskControl(TaskHandle_Pause, &IsRun_Pause, SUSPEND);
          if ((timerCounter - durasi) % 10 == 0)
          {
            for (int m = 0; m < 3; m++) {
              digitalWrite(BUZZER_PIN, HIGH);
              vTaskDelay(100);
              digitalWrite(BUZZER_PIN, LOW);
              vTaskDelay(50);
            }
          }
        } break;
    }
    vTaskDelay(100);
  }
}

void taskPWMCalculator(void *pvParameters)  // This is a task.
{

  ledcSetup(MTR_pwmChannel, MTR_freq, MTR_resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwm, MTR_pwmChannel);

  vTaskControl(TaskHandle_PWMCalculator, &IsRun_PWMCalculator, SUSPEND);
  for (;;) {  // A Task shall never return or exit.
    // PID calculation
    if (MTR_speed_req == 0) {
      // PWM signal
      ledcWrite(MTR_pwmChannel, 0);
    } else {
      MTR_error = MTR_speed_req - MTR_speed_actual;
      MTR_pidTerm = (MTR_Kp * MTR_error) + (MTR_Kd * (MTR_error - MTR_last_error)) + MTR_sum_error * MTR_Ki + 195;
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

  vTaskControl(NULL, &IsRun_SpeedRead_rpm, SUSPEND);


  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTimeSpeedRead = xTaskGetTickCount();
  for (;;) {  // A Task shall never return or exit.
    vTaskDelayUntil( &xLastWakeTimeSpeedRead, xFrequency );

    // motor use gear ratio 1 : 46.8512
    // gear pulley system using gear ratio 20 : 36 = 1,8
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
    float real_valueRPM = ((real_valueRPS / 1,8) / 46.8512) * 60;
    MTR_speed_actual = real_valueRPM;

    // Serial.println("Task Speed Read");
    //  printMotorInfo();
  }
}

void taskPMNS_MAIN(void* v) {
  double prevtime = millis();
  double cumerror = 0;

  //  unsigned int prev_TempRead;
  unsigned int temp_counter = 0;
  unsigned int PMNS_ssr = 2;
  unsigned int PMNS_counter = 0;

  const int pwm_freq = 2;
  const int pwm_ledChannel = 2;
  const int pwm_resolution = 16;

  unsigned int pwm_attach = 0;

  ledcSetup(pwm_ledChannel, pwm_freq, pwm_resolution);

  float dutyCycle = 0;

  pinMode(SSR_PIN, OUTPUT);
  sensor.begin();

  vTaskControl(TaskHandle_PMNS, &IsRun_PMNS, SUSPEND);

  double setPoint = temperatur;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, 1000);
    TempRead = get_temp(sensor);
    switch (PMNS_pemanas_state) {
      case PMNS_STATE_START: //PID
        {
          //PID calculation
          double output = PMNS_computePID(TempRead, setPoint, &prevtime, &cumerror);

          //Normalization PID to dutyCycle (0 - 0xFFFF)
          dutyCycle = output / 500.0;
          if (dutyCycle > 1.0)
            dutyCycle = 1;
          else if (dutyCycle < 0.0)
            dutyCycle = 0;
          dutyCycle *= 0xFFFF;

          //PWM pin initialization
          if (!pwm_attach)
          {
            ledcAttachPin(SSR_PIN, pwm_ledChannel);
            pwm_attach = 1;
          }

          //PWM update dutyCycle
          ledcWrite(pwm_ledChannel, dutyCycle);

          //Check for temperature steadiness
          if (abs(TempRead - setPoint) <= 2.5) {
            temp_counter++;
          }
          if (temp_counter >= 2) {
            PMNS_flag_pemanas_awal_done = 1;
          }
          break;
        }
      case PMNS_STATE_STEADY: //On Off
        {
          //Stop PWM, switch to bang-bang
          if (pwm_attach) {
            ledcDetachPin(SSR_PIN);
            pwm_attach = 0;
          }

          switch (PMNS_ssr)
          {
            case 0: //nunggu sampe turun dari kelebihan suhu
              digitalWrite(SSR_PIN, LOW);
              if (TempRead <= setPoint - BB) {
                PMNS_ssr = 1; //change state
                PMNS_counter = PMNS_ON_TIME;
              }
              break;
            case 1: //decrement from PMNS_ON_TIME until 0
              digitalWrite(SSR_PIN, HIGH);
              PMNS_counter--;
              if (PMNS_counter == 0)
              {
                PMNS_ssr = 2;
              }
              break;
            case 2: //nunggu selama WAIT TIME
              digitalWrite(SSR_PIN, LOW);
              PMNS_counter++;
              if (PMNS_counter == PMNS_WAIT_TIME)
              {
                PMNS_ssr = 0;
              }
              break;
          }
          break; //break state 1
        }
    }
  }
}

#if ENABLE_PRINT_DEBUG
void taskPrint(void* v) {
  char data[100];

  Serial.begin(115200);
  Serial.write(0x2);//Start of text
  Serial.println("State,Set Temp,Read Temp,Set RPM,Read RPM,Set Detik,Jalan Detik");
  Serial.write(0x6);//End of Transmission
  for (;;) {
    //FORMAT DATA: STATE;SP TEMP;TEMP;SP RPM;RPM;SP SEKON;SEKON
    sprintf(data, "%d,%d,%f,%d,%f,%u,%u",
            stateCondition,
            temperatur, TempRead,
            kecepatan, MTR_speed_actual,
            durasi, timerCounter);
    Serial.println(data);
    vTaskDelay(1500);
  }
}
#endif
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

double get_temp(DallasTemperature sensor) {
  for (int x = 0 ; x < 2 ; x++) {
    sensor.requestTemperatures();
    medianFilter.AddValue(sensor.getTempCByIndex(0));
  }

  sensor.requestTemperatures();
  return medianFilter.AddValue(sensor.getTempCByIndex(0));
}

double PMNS_computePID(double inp, unsigned int setPoint, double* previousTime, double* cumError) {
  double error = 0;
  double elapsedTime = 0;
  double currentTime;

  double kp = 8; //10
  double ki = 0.005; //0.011

  currentTime = millis() / 1000;                      //get current time
  elapsedTime = (currentTime - *previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                             // determine error
  *cumError += error * elapsedTime;                   // compute integral

  double out = kp * error + ki * *cumError;           // + kd * rateError;          //PID output

  *previousTime = currentTime;                        //remember current time

  return out;                                         //have function return the PID output
}

void vTaskControl(TaskHandle_t xHandle, bool* statusVar, unsigned int command) {
  if (*statusVar == RUNNING) {
    if (command == RESUME) {
      //do nothing
      *statusVar = RUNNING;
    }
    else if (command == SUSPEND) {
      *statusVar = SUSPENDED;
      return vTaskSuspend(xHandle);
    }
  }
  else if (*statusVar == SUSPENDED) {
    if (command == RESUME) {
      *statusVar = RUNNING;
      return vTaskResume(xHandle);
    }
    else if (command == SUSPEND) {
      //do nothing
      *statusVar = SUSPENDED;
    }
  }
}
