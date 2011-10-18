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
#include <Wire.h>
#include <TinyGPS.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <util/crc16.h>

byte led_state = 0;
unsigned int count = 0;
char s[100] = "0";

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
int tmp102Address = 0x48;
byte res;
int val;
int outside_temp = 0;

// buzzer
#define BUZZER 5
#define UPPER_CEILING 6000
#define LOWER_CEILING 5000

// uBlox nav mode
uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};

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

// TEMPERATURE
int get_temp() {
  Wire.requestFrom(tmp102Address, 2);

  byte MSB = Wire.receive();
  byte LSB = Wire.receive();

  int TemperatureSum = ((MSB << 8) | LSB) >> 4; //it's a 12bit int, using two's compliment for negative

  int celsius = TemperatureSum*0.0625;

  return celsius;
}

// BLINKY FUNCTIONS AND OTHER HELPERS
void blink() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
}

void beep() {
  digitalWrite(BUZZER, LOW);
  delay(300);
  digitalWrite(BUZZER, HIGH);
  delay(300);
}

void toggle() {
  digitalWrite(13, led_state);
  led_state = !led_state;
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


// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.print(MSG[i], BYTE);
  }
  Serial.println();
}

// Get the current NAV5 mode
int getUBXNAV5() {

  uint8_t b;
  uint8_t byteID = 0;
  int startTime = millis();

  // Poll/query message
  uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 };

  // First few bytes of the poll response
  uint8_t response[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF};

  // Interrogate Mr GPS...
  sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));

  // Process his response...
  while (1) {

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      return -1;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // 8th byte is the nav mode
      if (byteID == 8) {
        return b;
      }
      // Make sure the response matches the expected preamble
      else if (b == response[byteID]) {
        byteID++;
      } else {
        byteID = 0; // Reset and look again, invalid order
      }

    }
  }

}

uint16_t gps_CRC16_checksum (char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}


/***********************************
************************************
***********************************/

void setup() {
  pinMode(RADIO_SPACE, OUTPUT);
  pinMode(RADIO_MARK, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  beep(); // to show we're started

  gps_serial.begin(4800);

  //Turning off all GPS NMEA strings apart from GPGGA on the uBlox modules
  gps_serial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  gps_serial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  gps_serial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  gps_serial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  gps_serial.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  delay(2000);
  // Set the navigation mode (Airborne, 1G)
  rtty_txstring("Setting uBlox nav mode: ");
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  delay(100);
  if(getUBXNAV5() != 6) {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    rtty_txstring("Resending UBX Command");
  }
  else {
    rtty_txstring("Ublox in Airborne Mode");
  }

  Serial.begin(9600); // This is for debugging and the openLog
  Wire.begin();
  delay(3000); // artificial delay to wait for openLog

  beep();
  beep();
  beep();
}

void loop() {
  bool fed = false;
  char checksum[10];
  int n;
  int upper_ceiling_reached = 0;
  int lower_ceiling_reached =  0;

  // Set all our variables
  outside_temp = get_temp();
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

  // transmit the data
  snprintf(s, sizeof(s), "$$KI6YMZ,%i,%lu,%li,%li,%li,%lu,%lu,%i", count, g.time, g.lat, g.lon, g.alt, g.speed, g.course, outside_temp);

  //snprintf(checksum, sizeof(checksum), "*%02X\n", xor_checksum(s));
  snprintf(checksum, sizeof(checksum), "*%04X\n", gps_CRC16_checksum(s));

  memcpy(s + strlen(s), checksum, strlen(checksum) + 1);

  rtty_txstring(s);

  Serial << s << endl;

  blink();
  count++;
}

