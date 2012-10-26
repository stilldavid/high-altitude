// This Arduino sketch reads DS18B20 "1-Wire" digital
// temperature sensors.
// Tutorial:
// http://www.hacktronics.com/Tutorials/arduino-1-wire-tutorial.html

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <util/crc16.h>

#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Streaming.h>
#include <TinyGPS.h>

// Data wire is plugged into pin 10 on the Arduino
#define ONE_WIRE_BUS 10

// the pin the ntx2 is on
#define RADIOPIN 3
SoftwareSerial dummy_serial(255, 255);

// the sentence
char s[100] = "0";

// keep track of how many iterations we've done
unsigned int count = 0;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

float inside_temp = 0.0;
float outside_temp = 0.0;
char itbuf[12], otbuf[12];

// Assign the addresses of your 1-Wire temp sensors.
DeviceAddress insideThermometer = { 0x28, 0xAB, 0xC1, 0xAA, 0x03, 0x00, 0x00, 0x03 };
DeviceAddress outsideThermometer = { 0x28, 0x89, 0x10, 0x02, 0x02, 0x00, 0x00, 0x56 };

// GPS Stuffs
const int GPS_RX = 5;
const int GPS_TX = 6;

SoftwareSerial gps_serial(GPS_RX, GPS_TX);
TinyGPS gps;
typedef struct {
  float flat, flon, falt;
  float fkph, fcourse;
  unsigned long fix_age, time, date;
  unsigned long chars;
  unsigned short sentences, failed_checksum;
} gpsd;
gpsd g; // instantiate above struct
char latbuf[12], lonbuf[12], ialtbuf[10], coursebuf[12], kphbuf[12];
long int ialt = 123;

int hour, minute, second = 0;

/***********************************
************************************
***********************************/

void setup(void) {
  // start serial port
  Serial.begin(9600);

  gps_serial.begin(9600);

  // Start up the library
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(insideThermometer, 10);
  sensors.setResolution(outsideThermometer, 10);

  pinMode(RADIOPIN, OUTPUT);
}

void loop(void) {
  bool fed = false;
  unsigned int feeder = 0;

  // feed gps
  while(!fed && feeder < 1000) {
    fed = feedgps();
    feeder++;
  }

  // retrieves +/- lat/long in 100000ths of a degree
  gps.f_get_position(&g.flat, &g.flon);

  // time in hhmmsscc, date in ddmmyy
  gps.get_datetime(&g.date, &g.time, &g.fix_age);
  hour = (g.time / 1000000);
  minute = ((g.time - (hour * 1000000)) / 10000);
  second = ((g.time - ((hour * 1000000) + (minute * 10000))));
  second = second / 100;

  // returns speed in 100ths of a knot
  g.fkph = gps.f_speed_kmph();

  // course in 100ths of a degree
  g.fcourse = gps.f_course();

  g.falt = gps.f_altitude();

  // make stuff pretty
  floatToString(latbuf, g.flat, 4, 0);
  floatToString(lonbuf, g.flon, 4, 0);
  floatToString(coursebuf, g.fcourse, 1, 0);
  floatToString(kphbuf, g.fkph, 1, 0);
  ialt = long(g.falt);

  Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());

  sensors.requestTemperatures();

  inside_temp = getTemperature(insideThermometer);
  outside_temp = getTemperature(outsideThermometer);

  floatToString(itbuf, inside_temp, 2, 0);
  floatToString(otbuf, outside_temp, 2, 0);

  snprintf(s, sizeof(s), "$$KI6YMZ,%d,%d:%d:%d,%s,%s,%ld,%s,%s,%s,%s", count, hour, minute, second, latbuf, lonbuf, ialt, kphbuf, coursebuf, itbuf, otbuf);

  unsigned int CHECKSUM = gps_CRC16_checksum(s);  // Calculates the checksum
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(s,checksum_str);

  noInterrupts();
  rtty_txstring(s);
  interrupts();

  Serial.println(s);

  // give gps time to feed
  delay(1500);

  // ITERATE!
  count++;
}


/***********************************
************************************
***********************************/


// GPS
bool feedgps() {
  char c;
  while (gps_serial.available()) {
    c = gps_serial.read();
    //Serial.print(c);
    if (gps.encode(c))
      return true;
  }
  return false;
}

float getTemperature(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
    return -99;
  } else {
    return tempC;
  }
}

void rtty_txbit(int bit) {
  if (bit) {
    // high
    digitalWrite(RADIOPIN, HIGH);
  } else {
    // low
    digitalWrite(RADIOPIN, LOW);
  }

  // delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
}

/**
 * Simple function to sent each bit of a char to 
 * rtty_txbit function. 
 * NB The bits are sent Least Significant Bit first
 *
 * All chars should be preceded with a 0 and 
 * proceded with a 1. 0 = Start bit; 1 = Stop bit
 */
void rtty_txbyte(char c) {

  int i;

  rtty_txbit (0); // Start bit

  // Send bits for for char LSB first

  for(i=0; i<7; i++) { // Change this here 7 or 8 for ASCII-7 / ASCII-8
    if(c & 1)
      rtty_txbit(1);
    else
      rtty_txbit(0);

    c = c >> 1;
  }

  rtty_txbit(1); // Stop bit
  rtty_txbit(1); // Stop bit
}

/**
 * Simple function to sent a char at a time to 
 * rtty_txbyte function. 
 * NB Each char is one byte (8 Bits)
 */
void rtty_txstring(char * string) {
  dummy_serial.read();

  char c;
  c = *string++;
  while ( c != '\0') {
    rtty_txbyte (c);
    c = *string++;
  }
}


uint16_t gps_CRC16_checksum(char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

// this makes me cry.
char* floatToString(char * outstr, double val, byte precision, byte widthp){
  char temp[16];
  byte i;
  // compute the rounding factor and fractional multiplier
  double roundingFactor = 0.5;
  unsigned long mult = 1;
  for (i = 0; i < precision; i++)
  {
    roundingFactor /= 10.0;
    mult *= 10;
  }

  temp[0]='\0';
  outstr[0]='\0';

  if(val < 0.0){
    strcpy(outstr,"-\0");
    val = -val;
  }

  val += roundingFactor;

  strcat(outstr, itoa(int(val),temp,10));  //prints the int part
  if( precision > 0) {
    strcat(outstr, ".\0"); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
      mult *=10;

    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;

    while(frac1 /= 10)
      padding--;

    while(padding--)
      strcat(outstr,"0\0");

    strcat(outstr,itoa(frac,temp,10));
  }

  // generate space padding
  if ((widthp != 0)&&(widthp >= strlen(outstr))){
    byte J=0;
    J = widthp - strlen(outstr);

    for (i=0; i< J; i++) {
      temp[i] = ' ';
    }

    temp[i++] = '\0';
    strcat(temp,outstr);
    strcpy(outstr,temp);
  }

  return outstr;
}
