#include <RingBuf.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>

/*
  SD card read/write
 
 This shows how to read and write data to and from an SD card file   
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11, pin 7 on Teensy with audio board
 ** MISO - pin 12
 ** CLK - pin 13, pin 14 on Teensy with audio board
 ** CS - pin 4, pin 10 on Teensy with audio board
 

 
 This example code is in the public domain.
   
 */
 

#include <SPI.h>
SdFat sd;
SdFile myFile;


# define PIN_CS 10
# define SD_SPI_SPEED 20

void setup()
{

  
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect.
  }


  Serial.print("Initializing SD card...");

  if (!sd.begin(PIN_CS,SD_SPI_SPEED)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  

  // open the file for reading:
   if (!myFile.open("test.txt", O_READ)) {
    Serial.print("Reading failed");
    while(1){}

  }
  Serial.println("Reading has begun");
    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      
      Serial.write(myFile.read());
    }
    // close the file:
 
    myFile.close();
    

}

void loop(){
  }
