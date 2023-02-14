
#include <RingBuf.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>


#define SEALEVELPRESSURE_HPA (1013.25)
#define BNO055_SAMPLERATE_DELAY_MS (100)
# define PIN_CS 10
# define SD_SPI_SPEED 20


Adafruit_BNO055 myIMU = Adafruit_BNO055(55,0x28, &Wire1);
Adafruit_BMP3XX bmp;

SdFat sd;
SdFile myFile;

void setup() {
// setup of two 12C connections
  Wire.setSCL(19);
  Wire.setSDA(18);
  Wire.begin();
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  Wire1.begin();
  long starttime = 0;
  long endtime = 0;

    // serial com
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect.
  }


  Serial.print("Initializing SD card...");
// Intialization of sd card
  if (!sd.begin(PIN_CS,SD_SPI_SPEED)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  if(sd.exists("test.txt")){
    sd.remove("test.txt");
    }
  
  
  
  // if the file opened okay, write to it:
  if (!myFile.open("test.txt", O_WRITE | O_CREAT)) {
    Serial.print("Writing failed");
    while(1){}  }

  Serial.println("Adafruit BMP390 test");
// Intialization of BMP_Sensor
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
// Initialization of BNO_55
  if (!myIMU.begin()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
delay(1000);
myIMU.setExtCrystalUse(true);

// setup of a _ minute loop to print data from all sensors onto sdfile. 
starttime = micros();
endtime = starttime;

while((endtime - starttime) <=60000000){ // do this loop for up to 1000mS


  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  
  myFile.print("Temperature = ");
  myFile.print(bmp.temperature);
  myFile.println(" *C");

  myFile.print("Pressure = ");
  myFile.print(bmp.pressure / 100.0);
  myFile.println(" hPa");

  myFile.print("Approx. Altitude = ");
  myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  myFile.println(" m");

  myFile.println();
 

  // BNO 55 calibration
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);

// Using quaternions for better orientation interpolation
  imu::Quaternion quat=myIMU.getQuat();

  double q0 = quat.w();
  double q1 = quat.x();
  double q2 = quat.y();
  double q3 = quat.z();

  double roll=-atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  double pitch=asin(2*(q0*q2-q3*q1));
  double yaw=-atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

  myFile.print("Roll = ");
  myFile.print(roll);

  myFile.print("  Pitch = ");
  myFile.print(pitch);


  myFile.print("  Yaw = ");
  myFile.print(yaw);
  myFile.println(" ");

  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.println(system);

  delay(BNO055_SAMPLERATE_DELAY_MS);

  
  endtime = micros();
 }
 // Always close sdfile after data collection to prevent corruption.
myFile.close();
}

void loop() {
 
}
