// include freeRTOS library
#include <Arduino_FreeRTOS.h>

// include library Kalman Filter
#include <SimpleKalmanFilter.h>

// include library Median Filter
#include "MedianFilterLib.h"

#define pwm 11 
#define dir1 4 
#define dir2 5

// PID controller
int speed_req = 50;    // in rpm
float speed_actual = 0;   // in rpm
double Kp = 9;
double Kd = 19;
double Ki = 0.03;
float error = 0;
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

void TaskSpeedRead_rpm( void *pvParameters );

void setup() {
    Serial.begin(115200);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwm, OUTPUT);

    xTaskCreate(
    TaskSpeedRead_rpm
    ,  (const portCHAR *)"SpeedRead_rpm"    // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
}

void loop() {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);

    // speed from low rpm gear

    error = speed_req - speed_actual;
    pidTerm = (Kp * error) + (Kd * (error - last_error)) + sum_error * Ki;
    last_error = error;
    sum_error += error;
    PWM_val = constrain(pidTerm, 39, 255);
    
    analogWrite(pwm, PWM_val);
    printMotorInfo();
    delay(20);
}

void TaskSpeedRead_rpm(void *pvParameters)  // This is a task.
{
    (void) pvParameters;

    pinMode(enA, INPUT);
    digitalWrite(enA, HIGH);       // turn on pullup resistor
    pinMode(enB, INPUT);
    digitalWrite(enB, HIGH);       // turn on pullup resistor
    attachInterrupt(0, updateEncoder, CHANGE);  // encoDER ON PIN 2
    attachInterrupt(1, updateEncoder, CHANGE);

    for (;;) // A Task shall never return or exit.
    {
        // motor use gear ratio 1 : 46.8512
        // speed from high rpm gear
        newposition = encoderValue / 48;
        vel = (newposition - oldposition);
        oldposition = newposition;
        
        float real_value = vel;

        // calculate the estimated value with Median Filter
        float median_value = medianFilter.AddValue(real_value);
        median_value = median_value * (1000/15) * 60 / 46.8512;

        // calculate the estimated RPM value with Kalman Filter
        float kalman_value = simpleKalmanFilter.updateEstimate(median_value);

        // filtered value for PID calculation
        speed_actual = kalman_value; 

        vTaskDelay(1); // delay for 15 ms
    }
}

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
void updateEncoder(){
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