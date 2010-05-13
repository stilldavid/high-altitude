/**********************************************

Much of this code is from various projects that
were kind enough to post their own source code.

  Terry Baume, Project Horus
  http://projecthorus.org/

  Robert Harrison, Pegasus HAB Project
  http://www.pegasushabproject.org.uk/
  
  James Coxon, All Knowing HAB Guru
  irc.freenode.net#highaltitude
  
  Daniel Richman, Alien Project
  http://alienproject.wordpress.com/
  
And many more...

**********************************************/

#include <Streaming.h>
#include <NewSoftSerial.h>
#include <OneWire.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

byte led_state = 0;
unsigned int count = 0;
char s[100] = "0";

// COMPASS
const int compass = (0x32 >> 1); 
int heading = 0;

// USB WEATHER
char weather[100];
const int WEATHER_RX = 3;
const int WEATHER_TX = 2;
NewSoftSerial weather_serial(WEATHER_RX, WEATHER_TX);

// GPS
const int GPS_RX = 6;
const int GPS_TX = 7;
#define BUFFERSIZE 1000
char buffer[1000];
NewSoftSerial gps_serial(GPS_RX, GPS_TX);
TinyGPS gps;
typedef struct {
  long lat, lon, alt;
  unsigned long fix_age, time, date, speed, course;
  unsigned long chars;
  unsigned short sentences, failed_checksum;
} gpsd;
gpsd g; // instantiate above struct


// RADIO
#define RADIO_SPACE 10
#define RADIO_MARK  11
#define ASCII_BIT 8
NewSoftSerial dummy_serial = NewSoftSerial(255, 255);

// TEMPERATURE DS18S20
#define TEMP_PIN 12
OneWire ds(TEMP_PIN);
int outside_temp = 0;

/***********************************
************************************
***********************************/

// GPS
bool feedgps() {
  while (gps_serial.available()) {
    if (gps.encode(gps_serial.read()))
      return true;
  }
  return false;
}

// RADIO
void rtty_txstring (char * string) {
  dummy_serial.read();
  char c;
  c = *string++;
  while ( c != '\0') {
    rtty_txbyte (c);
    c = *string++;
  }
}

void rtty_txbyte (char c) {
  int i;
  rtty_txbit (0); // Start bit
  // Send bits for for char LSB first	
  for (i=0; i<ASCII_BIT; i++) {
    if (c & 1) 
      rtty_txbit(1); 
    else
      rtty_txbit(0);	
    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit 2
}

void rtty_txbit (int bit) {
  if (bit) {
    digitalWrite(RADIO_SPACE, LOW);
    digitalWrite(RADIO_MARK,  HIGH);
  } else {    
    digitalWrite(RADIO_SPACE, HIGH);
    digitalWrite(RADIO_MARK,  LOW);
  }
  // This works out to a baud rate of 50 bps. Somehow.
  delay(19);
  delayMicroseconds(250);
}

// WEATHER BOARD
// read until we have a full sentence
void set_weather() {
  weather_serial.read();
  delay(1000); // needs this long to buffer a whole sentence
  int i = 0;
  int stop = 0;
  int started = 0;
  int retries = 0;
  while (retries < 1300) { // wait at most 1.3 seconds for a valid reading (in case we were in the middle of one)
    if (weather_serial.available() > 0) {
      weather[i] = weather_serial.read();
      if('#' == weather[i]) {
        i = 0;
        weather[0] = '#';
        started = 1;
      }
      if('$' == weather[i]) {
        if(started) {
          weather[i+1] = '\0';
          return;
        } else {
          i = 0; // $ but not started yet
        }
      }
      i++;
    } else {
      delay(1);
      i = 0;
      retries++;
    }
  }
  return;
}

// COMPASS
int get_heading() { 
  byte headingData[6]; 
  Wire.beginTransmission(compass); 
  Wire.send(0x50); 
  Wire.endTransmission(); 
  delay(2); 
  Wire.requestFrom(compass, 6); 
  int i = 0;
  int retries = 0;
  while(retries < 10) {
    if(Wire.available()) {
      while(i < 6) { 
        headingData[i] = Wire.receive(); 
        i++; 
      } 
      return (headingData[0]*256 + headingData[1])/10; 
    } else {
      retries ++;
    }
  }
  return -1;
}

// TEMPERATURE
int get_temp() {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole;

  OneWireReset(TEMP_PIN);
  OneWireOutByte(TEMP_PIN, 0xcc);
  OneWireOutByte(TEMP_PIN, 0x44); // perform temperature conversion, strong pullup for one sec

  OneWireReset(TEMP_PIN);
  OneWireOutByte(TEMP_PIN, 0xcc);
  OneWireOutByte(TEMP_PIN, 0xbe);

  LowByte = OneWireInByte(TEMP_PIN);
  HighByte = OneWireInByte(TEMP_PIN);
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) { // Negative
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100;
  return Whole;
}

void OneWireReset(int Pin) {// reset.  Should improve to act as a presence pulse
  digitalWrite(Pin, LOW);
  pinMode(Pin, OUTPUT); // bring low for 500 us
  delayMicroseconds(500);
  pinMode(Pin, INPUT);
  delayMicroseconds(500);
}

void OneWireOutByte(int Pin, byte d) { // output byte d (least sig bit first).
  byte n;
  for(n=8; n!=0; n--) {
    if ((d & 0x01) == 1) {  // test least sig bit
      digitalWrite(Pin, LOW);
      pinMode(Pin, OUTPUT);
      delayMicroseconds(5);
      pinMode(Pin, INPUT);
      delayMicroseconds(60);
    } else {
      digitalWrite(Pin, LOW);
      pinMode(Pin, OUTPUT);
      delayMicroseconds(60);
      pinMode(Pin, INPUT);
    }
    d=d>>1; // now the next bit is in the least sig bit position.
  }
}

byte OneWireInByte(int Pin) { // read byte, least sig byte first
  byte d, n, b;
  for (n=0; n<8; n++) {
    digitalWrite(Pin, LOW);
    pinMode(Pin, OUTPUT);
    delayMicroseconds(5);
    pinMode(Pin, INPUT);
    delayMicroseconds(5);
    b = digitalRead(Pin);
    delayMicroseconds(50);
    d = (d >> 1) | (b<<7); // shift d to right and insert b in most sig bit position
  }
  return(d);
}

// BLINKY FUNCTIONS AND OTHER HELPERS
void blink() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
}

void toggle() {
  digitalWrite(13, led_state); 
  led_state = !led_state;
}

// Converts a HEX string to an int
int atoh(char c) {
  if (c >= 'A' && c <= 'F')
    return c - 55;
  else if (c >= 'a' && c <= 'f')
    return c - 87;
  else
    return c - 48;
}

char *ftoa(char *a, double f, int precision) {
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  int heiltal = (int)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  int desimal = abs((int)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

uint8_t xor_checksum(char *string) {
	size_t i;
	uint8_t XOR;
	uint8_t c;
	XOR = 0;
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++) {
		c = string[i];
		XOR ^= c;
	}
	return XOR;
}


/***********************************
************************************
***********************************/

void setup() {
  pinMode(RADIO_SPACE, OUTPUT);
  pinMode(RADIO_MARK, OUTPUT);
    
  weather_serial.begin(9600);
  gps_serial.begin(4800);
  Serial.begin(9600); // This is for debugging and the openLog
  Wire.begin();
  delay(3000); // artificial delay to wait for openLog
}

void loop() {
  bool fed = false;
  char checksum[10];
    
  // Set all our variables
  set_weather();
  outside_temp = get_temp();
  heading = get_heading();
  while(!fed)
    fed = feedgps();
    
  // retrieves +/- lat/long in 100000ths of a degree
  gps.get_position(&g.lat, &g.lon, &g.fix_age);

  // time in hhmmsscc, date in ddmmyy
  gps.get_datetime(&g.date, &g.time, &g.fix_age);

  // returns speed in 100ths of a knot
  g.speed = gps.speed();

  // course in 100ths of a degree
  g.course = gps.course();
  
  g.alt = gps.altitude();
  
  Serial << g.alt << endl;
      
  // transmit the data
  snprintf(s, sizeof(s), "$$KI6YMZ,%i,%lu,%li,%li,%li,%lu,%lu,%i", count, g.time, g.lat, g.lon, g.alt, g.speed, g.course, outside_temp);
  snprintf(checksum, sizeof(checksum), "*%04X\n", xor_checksum(s));
	memcpy(s + strlen(s), checksum, strlen(checksum) + 1);
  
  rtty_txstring(s);
  
  // print to Serial (the OpenLog)
  if('#' == weather[0]) // not the checksum I'd like to see from the weather board...
    Serial << s << weather << "," << heading << endl;
  else
    Serial << weather << "," << heading << " - bad" << endl;
  
  blink();
  count++;

}



