// include freeRTOS library
// #include <Arduino_FreeRTOS.h>

// include library Kalman Filter
#include "SimpleKalmanFilter.h"

#define pwm 18 
#define encoderMotor 17

// setting PWM properties
const int freq = 256000;
const int pwmChannel = 0;
const int resolution = 8;

// PID controller
int speed_req = 5;    // in rpm
float speed_actual = 0;   // in rpm
double Kp = 5;
double Kd = 0.5;
double Ki = 0.1;
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

// SimpleKalmanFilter(e_mea, e_est, q);
// e_mea: Measurement Uncertainty 
// e_est: Estimation Uncertainty 
// q: Process Noise
SimpleKalmanFilter simpleKalmanFilter(3, 3, 0.1);

void TaskSpeedRead_rpm( void *pvParameters );

void TaskPWMCalculator( void *pvParameters );

void setup() {
    Serial.begin(115200);
    // pinMode(dir1, OUTPUT);
    // pinMode(dir2, OUTPUT);
    // pinMode(pwm, OUTPUT);
    // digitalWrite(dir1, LOW);
    // digitalWrite(dir2, HIGH);
    // configure LED PWM functionalitites

    xTaskCreate(
    TaskSpeedRead_rpm
    ,  "SpeedRead_rpm"    // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

    xTaskCreate(
    TaskPWMCalculator
    ,  "PWMCalculator"    // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
}

void loop() {
    delay(1000);

    // ------------------------- Task Running -------------------------- //
}

void TaskPWMCalculator(void *pvParameters)  // This is a task.
{
    (void) pvParameters;

    ledcSetup(pwmChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(pwm, pwmChannel);

    for (;;) // A Task shall never return or exit.
    {
        // PID calculation
        error = speed_req - speed_actual;
        pidTerm = (Kp * error) + (Kd * (error - last_error)) + sum_error * Ki + 120;
        last_error = error;
        sum_error += error;
        sum_error = constrain(sum_error, -2000, 2000);
        PWM_val = constrain(pidTerm, 0, 255);
        
        // PWM signal
        ledcWrite(pwmChannel, PWM_val);

        // printMotorInfo();
        vTaskDelay(40);
    }   
}

void TaskSpeedRead_rpm(void *pvParameters)  // This is a task.
{
    (void) pvParameters;

    pinMode(encoderMotor, INPUT_PULLUP);
    attachInterrupt(encoderMotor, updateEncoderMotor, CHANGE);    // encoderMotorValue will increase whenever any CHANGE

    for (;;) // A Task shall never return or exit.
    {
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
        float real_valueRPM = (real_valueRPS / (1.8 * 46.8512)) * 60;
        speed_actual = real_valueRPM;
        Serial.print(speed_req);
        Serial.print(" ");
        Serial.println(speed_actual);

        vTaskDelay(20);       
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
