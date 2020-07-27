#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

/*
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (CS_SD)
*/

#define CS_SD 4

#define RXs 2
#define TXs 3

#define BRATE 9600

SoftwareSerial mySerial(RXs, TXs); // RX, TX
File myFile;

String FilenameCreate(int version) {
  return ("Datalog" + String(version) + ".csv");
}
String filename;

void setup()
{
  /*INISIALISASI SERIAL, MY SERIAL, SD CARD, FILENAME*/
  Serial.begin(BRATE);
  mySerial.begin(BRATE);

  if (!SD.begin(CS_SD)) {
    Serial.println("SD INIT FAILED");
    while (1);
  }
  Serial.println("SD INIT OK");

  //Find filename
  uint8_t logversion = 0;
  filename = FilenameCreate(logversion);
  while (SD.exists(filename)) {
    logversion++;
    filename = FilenameCreate(logversion);
  }
  Serial.println("Data log created: " + filename);

  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {
    Serial.println("file ERROR");
    while (1);
  }

  Serial.println("Data logger running");
}

String datain;

void loop() // run over and over
{
  if (mySerial.available()) {
    datain = (mySerial.readStringUntil('\n'));
    Serial.println("arduino;" + datain);
    myFile.print(datain);
    myFile.flush();
  }
}


// void readFromSD(String filename) {
//   File sdfile;
//   sdfile = SD.open(filename);

//   if (sdfile) {
//     Serial.println("Opening " + filename + " success");

//     //Read per character
//     while (sdfile.available()) {
//       Serial.write(sdfile.read());
//     }
//     sdfile.close();
//   }
//   else {
//     Serial.println("ERROR opening " + filename + "!");
//     return;
//   }
// }
