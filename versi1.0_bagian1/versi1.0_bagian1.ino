#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//#include <math.h>
#include "ArduinoJson.h"

// universal needs
int stateCondition = 0;
int temperatur = 25, kecepatan = 0;
unsigned int jam = 0; 
unsigned int menit = 0;
boolean lastButtonState = LOW;
boolean currentButtonState;

// Fot the rotary encoder
int encoderPin1 = 2;
int encoderPin2 = 3;
int encoderSwitchPin = 4; // push button rotary encoder
volatile int lastEncoded = 0;
volatile unsigned long encoderValue = 0;

long lastencoderValue = 0; // store the value of rotary encoder

int lastMSB = 0;
int lastLSB = 0;

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup()
{
    Serial.begin (9600);

    // making JSON file for transmiting data
    // StaticJsonDocument<200> doc;


	// initialize the LCD
	lcd.begin();

    // initialize the rotary encoder
    pinMode(encoderPin1, INPUT);
    pinMode(encoderPin2, INPUT);

    pinMode(encoderSwitchPin, INPUT);

    digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

    digitalWrite(encoderSwitchPin, HIGH); //turn pullup resistor on

    //call updateEncoder() when any high/low changed seen
    //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder, CHANGE);
}

void loop()
{
    // local array
    char tempVal[4];
    char spdVal[4];
    char minVal[4];
    char hourVal[4];

    encoderValue = 0; // reset the value 

    while (stateCondition == 0) {   // condition when temperature being set
        temperatur = (encoderValue / 4) + 25;
        
        // temperatur value must be between 25 up to 90 deg C
        if (temperatur < 25) {
            encoderValue = 0;
            temperatur = (encoderValue / 4) + 25;
            sprintf(tempVal, "%3d", temperatur);    // map int value to string
            lcd.setCursor(0,0);                     // show character at column 0, row 0
            lcd.print("Temperatur: ");              // show string on LCD
            lcd.setCursor(12,0);                    // show character at column 12, row 0
            lcd.print(tempVal);                     // show the value
            lcd.setCursor(16,0);                    // show character at column 16, row 0
            lcd.print((char)223);                   // show string on LCD
            lcd.setCursor(17,0);                    // show character at column 17, row 0
            lcd.print("C");                         // show string on LCD
            delayMicroseconds(1);
        } else if (temperatur > 90) {
            encoderValue = 65 * 4;
            temperatur = (encoderValue / 4) + 25;
            sprintf(tempVal, "%3d", temperatur);    // map int value to string
            lcd.setCursor(0,0);                     // show character at column 0, row 0
            lcd.print("Temperatur: ");              // show string on LCD
            lcd.setCursor(12,0);                    // show character at column 12, row 0
            lcd.print(tempVal);                     // show the value
            lcd.setCursor(16,0);                    // show character at column 16, row 0
            lcd.print((char)223);                   // show string on LCD
            lcd.setCursor(17,0);                    // show character at column 17, row 0
            lcd.print("C");                         // show string on LCD
            delayMicroseconds(1);
        } else {
            sprintf(tempVal, "%3d", temperatur);    // map int value to string
            lcd.setCursor(0,0);                     // show character at column 0, row 0
            lcd.print("Temperatur: ");              // show string on LCD
            lcd.setCursor(12,0);                    // show character at column 12, row 0
            lcd.print(tempVal);                     // show the value
            lcd.setCursor(16,0);                    // show character at column 16, row 0
            lcd.print((char)223);                   // show string on LCD
            lcd.setCursor(17,0);                    // show character at column 17, row 0
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
        } else if (currentButtonState == LOW && lastButtonState == HIGH){
            //button is being pushed
            stateCondition ++;
            encoderValue = 0; 
        }

        lastButtonState = currentButtonState;       
    }

    while (stateCondition == 1) {   // condition when rpm motor being set
        // show the previous data
        lcd.setCursor(0,0);                     // show character at column 0, row 0
        lcd.print("Temperatur: ");              // show string on LCD
        lcd.setCursor(12,0);                    // show character at column 12, row 0
        lcd.print(tempVal);                     // show the value
        lcd.setCursor(16,0);                    // show character at column 16, row 0
        lcd.print((char)223);                   // show string on LCD
        lcd.setCursor(17,0);                    // show character at column 17, row 0
        lcd.print("C");                         // show string on LCD
        delayMicroseconds(1);

        kecepatan = encoderValue / 4;

        // kecepatan value mush be between 0 up to 70 rpm
        if (kecepatan < 30) {
            encoderValue = 30 * 4;
            kecepatan = encoderValue / 4;
            sprintf(spdVal, "%3d", kecepatan);      // map int value to string
            lcd.setCursor(0,1);                     // show character at column 0, row 1
            lcd.print("Kecepatan : ");              // show string on LCD
            lcd.setCursor(12,1);                    // show character at column 12, row 1
            lcd.print(spdVal);                      // show the value
            lcd.setCursor(16,1);                    // show character at column 16, row 1
            lcd.print("RPM");                       // show string on LCD
            delayMicroseconds(1);
        } else if (kecepatan > 70) {
            encoderValue = 70 * 4;
            kecepatan = encoderValue / 4;
            sprintf(spdVal, "%3d", kecepatan);      // map int value to string
            lcd.setCursor(0,1);                     // show character at column 0, row 1
            lcd.print("Kecepatan : ");              // show string on LCD
            lcd.setCursor(12,1);                    // show character at column 12, row 1
            lcd.print(spdVal);                      // show the value
            lcd.setCursor(16,1);                    // show character at column 16, row 1
            lcd.print("RPM");                       // show string on LCD
            delayMicroseconds(1);
        } else {
            sprintf(spdVal, "%3d", kecepatan);      // map int value to string
            lcd.setCursor(0,1);                     // show character at column 0, row 1
            lcd.print("Kecepatan : ");              // show string on LCD
            lcd.setCursor(12,1);                    // show character at column 12, row 1
            lcd.print(spdVal);                      // show the value
            lcd.setCursor(16,1);                    // show character at column 16, row 1
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
        } else if (currentButtonState == LOW && lastButtonState == HIGH){
            //button is being pushed
            stateCondition ++;
            encoderValue = 0; 
        }

        lastButtonState = currentButtonState;
    }

    while (stateCondition == 2) {   // condition when hour is being set
        // show the previous data
        lcd.setCursor(0,0);                     // show character at column 0, row 0
        lcd.print("Temperatur: ");              // show string on LCD
        lcd.setCursor(12,0);                    // show character at column 12, row 0
        lcd.print(tempVal);                     // show the value
        lcd.setCursor(16,0);                    // show character at column 16, row 0
        lcd.print((char)223);                   // show string "degree" on LCD
        lcd.setCursor(17,0);                    // show character at column 17, row 0
        lcd.print("C");                         // show string on LCD
        delayMicroseconds(1);

        lcd.setCursor(0,1);                     // show character at column 0, row 1
        lcd.print("Kecepatan : ");              // show string on LCD
        lcd.setCursor(12,1);                    // show character at column 12, row 1
        lcd.print(spdVal);                      // show the value
        lcd.setCursor(16,1);                    // show character at column 16, row 1
        lcd.print("RPM");                       // show string on LCD
        delayMicroseconds(1);

        jam = ((encoderValue / 4) + (25 * 1000));  // the value far away from zero
        menit = 0;

        // jam value mush be between 0 up to 23
        sprintf(hourVal, "%3u", (jam % 25));           // map int value to string
        sprintf(minVal, "%3u", menit);          // map int value to string
        lcd.setCursor(0,2);                     // show character at column 0, row 2
        lcd.print("Durasi    : ");              // show string on LCD
        lcd.setCursor(12,2);                    // show character at column 12, row 2
        lcd.print(hourVal);                     // show the value
        lcd.setCursor(16,2);                    // show character at column 16, row 2
        lcd.print("JAM");                       // show string on LCD
        lcd.setCursor(12,3);                    // show character at column 12, row 3
        lcd.print(minVal);                      // show the value
        lcd.setCursor(16,3);                    // show character at column 16, row 3
        lcd.print("MEN");                       // show string on LCD                   
        delayMicroseconds(1);

        // push button action
        currentButtonState = digitalRead(encoderSwitchPin);
        delay(50);
        if (currentButtonState == HIGH && lastButtonState == LOW) {
            //button is not being pushed
            //do nothing
        } else if (currentButtonState == LOW && lastButtonState == HIGH){
            //button is being pushed
            stateCondition ++;
            encoderValue = 0;
            jam = atoi(hourVal); 
        }

        lastButtonState = currentButtonState;
    }

    while (stateCondition == 3) {       // condition when minute is being set
        // show the previous data
        lcd.setCursor(0,0);                     // show character at column 0, row 0
        lcd.print("Temperatur: ");              // show string on LCD
        lcd.setCursor(12,0);                    // show character at column 12, row 0
        lcd.print(tempVal);                     // show the value
        lcd.setCursor(16,0);                    // show character at column 16, row 0
        lcd.print((char)223);                   // show string "degree" on LCD
        lcd.setCursor(17,0);                    // show character at column 17, row 0
        lcd.print("C");                         // show string on LCD
        delayMicroseconds(1);

        lcd.setCursor(0,1);                     // show character at column 0, row 1
        lcd.print("Kecepatan : ");              // show string on LCD
        lcd.setCursor(12,1);                    // show character at column 12, row 1
        lcd.print(spdVal);                      // show the value
        lcd.setCursor(16,1);                    // show character at column 16, row 1
        lcd.print("RPM");                       // show string on LCD
        delayMicroseconds(1);

        if (jam == 24) {
            menit = 0;
            sprintf(minVal, "%3u", (menit % 60));          // map int value to string
            lcd.setCursor(0,2);                     // show character at column 0, row 2
            lcd.print("Durasi    : ");              // show string on LCD
            lcd.setCursor(12,2);                    // show character at column 12, row 2
            lcd.print(hourVal);                     // show the value
            lcd.setCursor(16,2);                    // show character at column 16, row 2
            lcd.print("JAM");                       // show string on LCD
            lcd.setCursor(12,3);                    // show character at column 12, row 3
            lcd.print(minVal);                      // show the value
            lcd.setCursor(16,3);                    // show character at column 16, row 3
            lcd.print("MEN");                       // show string on LCD                   
            delayMicroseconds(1);
        } else {
            menit = ((encoderValue / 4) + (60 * 100));  // the value far away from zero

            // jam value mush be between 0 up to 59
            sprintf(minVal, "%3u", (menit % 60));          // map int value to string
            lcd.setCursor(0,2);                     // show character at column 0, row 2
            lcd.print("Durasi    : ");              // show string on LCD
            lcd.setCursor(12,2);                    // show character at column 12, row 2
            lcd.print(hourVal);                     // show the value
            lcd.setCursor(16,2);                    // show character at column 16, row 2
            lcd.print("JAM");                       // show string on LCD
            lcd.setCursor(12,3);                    // show character at column 12, row 3
            lcd.print(minVal);                      // show the value
            lcd.setCursor(16,3);                    // show character at column 16, row 3
            lcd.print("MEN");                       // show string on LCD                   
            delayMicroseconds(1);
        }
        
        // push button action
        currentButtonState = digitalRead(encoderSwitchPin);
        delay(50);
        if (currentButtonState == HIGH && lastButtonState == LOW) {
            //button is not being pushed
            //do nothing
        } else if (currentButtonState == LOW && lastButtonState == HIGH){
            //button is being pushed
            stateCondition ++;
            encoderValue = 0;
            menit = atoi(minVal); 
        }

        lastButtonState = currentButtonState;
    }

    while (stateCondition == 4) {
        StaticJsonDocument<200> doc;
        doc["temperature"] = temperatur;
        doc["kecepatan"] = kecepatan;
        doc["jam"] = jam;
        doc["menit"] = menit;
        serializeJsonPretty(doc, Serial);
        Serial.println();

        // push button action
        currentButtonState = digitalRead(encoderSwitchPin);
        delay(50);
        if (currentButtonState == HIGH && lastButtonState == LOW) {
            //button is not being pushed
            //do nothing
        } else if (currentButtonState == LOW && lastButtonState == HIGH){
            //button is being pushed
            stateCondition ++;
            encoderValue = 0;
            menit = atoi(minVal); 
        }

        lastButtonState = currentButtonState;
    }
    
	// lcd.clear();
}

// interrupt when any change happen
void updateEncoder(){
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