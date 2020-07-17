#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define CS_SD 4

#define RXs 2
#define TXs 3

#define BRATE 57600

#define VER_ADDR 0 //Default EEprom Address

SoftwareSerial mySerial(RXs, TXs); // RX, TX
File myFile;

void setup()
{
  /*INISIALISASI SERIAL, MY SERIAL, SD CARD, FILENAME*/
  Serial.begin(57600);
  mySerial.begin(57600);

  if (!SD.begin(CS_SD)) {
    Serial.println("SD INIT FAILED");
    while (1);
  }

  //Filename SET
  uint8_t logversion = EEPROM.read(VER_ADDR);

  String filename = "datalog" + String(logversion) + ".csv";
  while (SD.exists(filename)) {
    logversion++;
    filename = "datalog" + String(logversion) + ".csv";
  }
  EEPROM.update(VER_ADDR, logversion);
  
  myFile = SD.open(filename, FILE_WRITE);

  Serial.println("DATA LOGGER RUNNING");
}

void loop() // run over and over
{
  if (mySerial.available()) {
    String datain;
    datain = (mySerial.readStringUntil('\n'));
    Serial.println(datain);
  }
}
