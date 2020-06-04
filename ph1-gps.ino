/*
  Renault Clio2 Ph1 GPS Speedo
  Rhys Oxenham <rdoxenham@gmail.com>
  May 2020
*/

#include <AH_AD9850.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <math.h>

#define CLK     2
#define FQUP    3
#define BitData 4
#define RESET   5

AH_AD9850 AD9850(CLK, FQUP, BitData, RESET);
String inString = "";
int prevFreq = 0;

SoftwareSerial mySerial(9, 8);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false

int speed_mph = 0;
int hz_array[] = {0, 2, 4, 6, 8, 10, 13, 15, 17, 19, 21, 23, 25, 28, 30, 32, 34, 36, 39, 41, 43, 45, 48, 50, 52, 54, 57, 59, 61, 64, 66, 68, 70, 73, 75, 77, 79, 81, 84, 86, 88, 90, 92, 94, 96, 98, 101, 103, 105, 107, 109, 111, 114, 116, 118, 120, 123, 125, 127, 130, 132, 134, 136, 138, 140, 142, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220, 222, 224, 227, 229, 231, 233, 235, 237, 239, 241, 243, 245, 247, 249, 251, 253, 255, 257, 259, 261, 263, 266, 268, 270, 272, 274, 276, 278, 280, 282, 284, 287, 289, 291, 293, 295, 297, 299, 301, 304, 306, 308, 310, 312, 315, 317, 319, 321, 323, 325, 327, 330, 332, 334, 336, 338, 340, 342, 344, 347, 349, 351, 353, 355, 358, 360, 362};
int period = 2000;
unsigned long time_now = 0;

void setup()
{
  Serial.begin(115200);
  // pin13 is used for bitbashing
  pinMode(13, OUTPUT);
  AD9850.reset();
  AD9850.powerDown();

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate
  //GPS.sendCommand(PGCMD_ANTENNA);
  // Turn off ABS light
  delay(1000);
  needle_sweep();
}

void loop()
{
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    if (GPS.fix) {
      // GPS.speed is in knots, so converting to mph
      int speed = round(GPS.speed * 1.150779);
      if(speed > 170)
      {
        speed = 170;
      }
      AD9850.set_frequency(0, 0, hz_array[speed]);
    }
    else
    {
      AD9850.set_frequency(0, 0, hz_array[0]);
    }
  }
}

void needle_sweep()
{
  AD9850.set_frequency(0, 0, hz_array[170]);
  delay(3000);
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  AD9850.set_frequency(0, 0, hz_array[0]);
  delay(3000);
}

void loop_through()
{
  for(int counter = 0; counter <= 170; counter = counter + 10)
  {
    time_now = millis();
    while (millis() < time_now + period)
    {
      AD9850.set_frequency(0, 0, hz_array[counter]);
    }
  }
}

void gather_data()
{
  while (Serial.available() > 0)
  {
    int inChar = Serial.read();
    if (isDigit(inChar))
    {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
  }
  int freq = inString.toInt();
  inString = "";
  if(freq != prevFreq)
  {
    Serial.println(freq);
    AD9850.set_frequency(0, 0, prevFreq);
    prevFreq = freq;
  }
  
  delay(1000);
}
