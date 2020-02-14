#include <Wire.h>
#include <LiquidCrystal_I2C.h>

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

// universal needs
int stateCondition = 0;
int temperatur = temConstant;
int kecepatan = kecConstant;
int jam = jamConstant;
int menit = menConstant;

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

// flag
boolean forward = 1;
boolean startProcess = 1;

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

byte arrow[8] = {
  B00011,
  B00111,
  B01111,
  B11111,
  B01111,
  B00111,
  B00011,
};

void setup() {
  Serial.begin (112500);

  xTaskCreate(
    taskInput,          /* Task function. */
    "TaskOne",        /* String with name of task. */
    10000,            /* Stack size in bytes. */
    NULL,             /* Parameter passed as input of the task */
    1,                /* Priority of the task. */
    NULL);            /* Task handle. */

  xTaskCreate(
    taskDisplay,          /* Task function. */
    "TaskTwo",        /* String with name of task. */
    10000,            /* Stack size in bytes. */
    NULL,             /* Parameter passed as input of the task */
    0,                /* Priority of the task. */
    NULL);            /* Task handle. */

}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void taskInput( void * parameter )
{
  // initialize the rotary encoder
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(switchPinGreen, INPUT_PULLUP);
  pinMode(switchPinYellow, INPUT_PULLUP);
  pinMode(switchPinWhite, INPUT_PULLUP);
  pinMode(switchPinBlack, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH);
  digitalWrite(encoderPin2, HIGH);

  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(encoderPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderPin2, updateEncoder, CHANGE);

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
      stateCondition = 0;
      encoderValue = constantEncoderVal;
      temperatur = temConstant;
      kecepatan = kecConstant;
      jam = jamConstant;
      menit = menConstant;
      //encoderValue = constantEncoderVal;
    }
    lastButtonStateWhite = currentButtonStateWhite;
  }
}

void taskDisplay( void * parameter)
{
  // initialize the LCD
  lcd.begin();
  lcd.createChar(0, arrow);

  for (;;) {
    switch (stateCondition) {
      case 0:
        temperatur = ((encoderValue / 4) % 66) + 25;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        break;
      case 1:
        kecepatan = (encoderValue / 4) % 71;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        break;
      case 2:
        jam = (encoderValue / 4) % 25;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        break;
      case 3:
          menit = (encoderValue / 4) % 60;
        printToLCD(temperatur, kecepatan, jam, menit, stateCondition);
        break;
      case 4:
          if (forward) {
            lcd.clear();
            stateCondition ++;
          } else {
            lcd.clear();
            stateCondition --;
          }
        break;
      case 5:
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
          Serial.println(startProcess);
        }
        else {
          lcd.setCursor(9, 1);
          lcd.print(" ");
          lcd.setCursor(9, 2);
          lcd.write(byte(0));
          startProcess = 0;
          Serial.println(startProcess);
        }
        break;
      case 6:
        //Serial.println(startProcess);
        if (startProcess) {
          stateCondition = 7;
        }
        else {
          forward = 0;
          stateCondition = 4;
        }
      case 7:
        Serial.println(stateCondition);
        Serial.println(forward);
        lcd.clear();
        break;
    }
    vTaskDelay(100);
  }
}

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
