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
#include <WString.h>

byte led_state = 0;

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

// RADIO
#define RADIO_SPACE 10
#define RADIO_MARK  11
#define ASCII_BIT 8

/***********************************
************************************
***********************************/

// GPS
int readLine(void) {
  char c;
  byte bufferIndex = 0;
  boolean startLine = 0;
  byte retries = 0;
  while (retries < 20) {
    if (gps_serial.available() > 0) {
      c = gps_serial.read();
      if (c == -1) { delay(2); continue; }
      if (c == '\n') continue;
      if (c == '$') startLine = 1;
      if ((bufferIndex == BUFFERSIZE-1) || (c == '\r')) {
        if (startLine) {
          buffer[bufferIndex] = 0;
          return 1;
        }
      }
      if (startLine)
        buffer[bufferIndex++] = c;
    } else {
      retries++;
      delay(50);
    }
  }
  return 0;
}

// Returns a specific field from the buffer
void getField(int getId, char *field, int maxLen) {
  byte bufferIndex = 0;
  byte fieldId = 0;
  byte i = 0;
  while (bufferIndex < sizeof(buffer)) {
    if (fieldId == getId) {
      // End of string, or string overflow
      if (buffer[bufferIndex] == ',' || i > (maxLen - 2)) {
        field[i] = 0;	// Null terminate
        return;
      }
      // Buffer chars to field
      field[i++] = buffer[bufferIndex++];
    } else {
      // Advance field on comma
      if (buffer[bufferIndex] == ',') {
        bufferIndex++;  // Advance in buffer
        fieldId++;			// Increase field position counter
      } else {
        bufferIndex++;	// Advance in buffer
      }
    }
  }
  // Null terminate incase we didn't already..
  field[i] = 0;
}

// Polls for an NMEA sentence of type requested
// Validates checksum, silently retries on failed checksums
int getNMEA(char *getType) {
  char type[7];
  byte retries = 0;
  while (retries < 2) {
    if (readLine() && validateChecksum()) {;
      getField(0, type, sizeof(type));
      if (strcmp(type, getType) == 0) {
        return 1;
      }
    } else {
      retries++;
    }
  }
  Serial << "NMEA: Failed to read from GPS!" << endl;
  return 0;
}

// Validates the checksum on an NMEA string
// Returns 1 on valid checksum, 0 otherwise
int validateChecksum(void) {
  char gotSum[2];
  gotSum[0] = buffer[strlen(buffer) - 2];
  gotSum[1] = buffer[strlen(buffer) - 1];
  // Check that the checksums match up
  if ((16 * atoh(gotSum[0])) + atoh(gotSum[1]) == getCheckSum(buffer)) 
    return 1;
  else
    return 0;
}

// Calculates the checksum for a given string
// returns as integer
int getCheckSum(char *string) {
  int i; int XOR;	int c;
  // Calculate checksum ignoring any $'s in the string
  for (XOR = 0, i = 0; i < strlen(string); i++) {
    c = (unsigned char)string[i];
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  return XOR;
}

// Returns the groundspeed in km/h
int getSpeed(void) {
  char field[10];
  getField(7, field, sizeof(field));
  int speed = atoi(field);
  return speed;
}

// Return the fix type from a GGA string
int getFixType(void) {
  char field[5];
  getField(6, field, sizeof(field));
  int fixType = atoi(field);
  return fixType;
}

// Return the altitude in meters from a GGA string
long getAlt(void) {
  char field[10];
  getField(9, field, sizeof(field));
  long altitude = atol(field);
  return altitude;
}

// Returns the number of satellites being tracked from a GGA string
int getSats(void) {
  char field[3];
  getField(7, field, sizeof(field));
  int numSats = atoi(field);
  return numSats;
}

// Read the latitude in decimal format from a GGA string
double getLat(void) {
  char field[12];
  getField(2, field, sizeof(field));			// read the latitude
  double latitude = atof(field);					// convert to a double (precise)
  int deg = (int) latitude / 100;				// extract the number of degrees
  double min = latitude - (100 * deg);			// work out the number of minutes
  latitude = deg + (double) min/60.0;			// convert to decimal format
  getField(3, field, sizeof(field));			// get the hemisphere (N/S)
  if (strcmp(field, "S") == 0) latitude *= -1;	// sign the decimal latitude correctly
  return latitude;
}

// Read the longitude in decimal format from a GGA string
double getLong(void) {
  char field[12];
  getField(4, field, sizeof(field));			// read the longitude
  double longitude = atof(field);					// convert to a double
  int deg = (int) longitude / 100;				// extract the number of degrees
  double min = longitude - (100 * deg);			// work out the number of minutes
  longitude = deg + (double) min/60.00;			// convert to decimal format
  getField(5, field, sizeof(field));			// get the E/W status
  if (strcmp(field, "W") == 0) longitude *= -1; // sign decimal latitude correctly
  return longitude;
}

// Converts UTC time to the correct timezone
void convertTime(int *time) {
  // How many hours off GMT are we?
  float offset = 6;
  long sectime = ((long)(time[0]) * 3600) + (time[1] * 60) + time[2];
  sectime += (offset * 3600.0);
  // Did we wrap around?
  if (sectime < 0) sectime += 86400;
  if (sectime > 86400) sectime -= 86400;
  // Convert back to time
  time[0] = (int)(sectime / 3600);
  time[1] = (int)((sectime % 3600) / 60);
  time[2] = (int)((sectime % 3600) % 60);
}

// Parses a time field from a GGA string
void parseTime(char *field, int *time) {
  char tmp[3]; tmp[2] = 0; // Init tmp and null terminate
  tmp[0] = field[0]; tmp[1] = field[1]; time[0] = atoi(tmp); // Hours
  tmp[0] = field[2]; tmp[1] = field[3]; time[1] = atoi(tmp); // Minutes
  tmp[0] = field[4]; tmp[1] = field[5]; time[2] = atoi(tmp); // Seconds
}

// Gets the hours, minutes and seconds from a GGA string
void getTime(int *time) {
  char field[12];
  getField(1, field, sizeof(field));
  parseTime(field, time);
  convertTime(time);
}

// RADIO
void rtty_txstring (char * string) {
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
  for (i=0;i<ASCII_BIT;i++) {
    if (c & 1) 
      rtty_txbit(1); 
    else
      rtty_txbit(0);	
    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit) {
  if (bit) {
    digitalWrite(RADIO_SPACE, LOW);
    digitalWrite(RADIO_MARK,  HIGH);
  } else  {    
    digitalWrite(RADIO_SPACE, HIGH);
    digitalWrite(RADIO_MARK,  LOW);
  }
  // This works out to a baud rate of 50 bps
  delay(19);
  delayMicroseconds(250);
}

// WEATHER BOARD
// read until we have a full sentence
void set_weather() {
  int i = 0;
  int stop = 0;
  int started = 0;
  int retries = 0;
  while (retries < 350) { // 200 * 5ms = 1s = worst case to get first bit from weather board, plus 10 in case we were in the middle of a reading
    if (weather_serial.available() > 0) {
      weather[i] = weather_serial.read();
      if('#' == weather[i]) {
        i = 0;
        weather[0] = '#';
        started = 1;
      }
      if(started && '$' == weather[i]) {
        weather[i+1] = '\0';
        return;
      }
      i++;
    } else {
      delay(3); // TODO: why this number?
      started = 0;
      i = 0;
      retries++;
    }
  }
  //Serial << "broke" << endl; // This means we have waited a full second and nothing :(
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
  
  // This is a play area for right now.
  // NOT PRODUCTION CODE, OBVIOUSLY
  
  // Set everything for this time around the loop
  set_weather();
  heading = get_heading();
  
  if('#' == weather[0])
    Serial << weather << " - " << heading << endl;
  else
    Serial << "broken";

  //while (!getNMEA("$GPGGA")) { ; }
  
  //Serial << getLat() << ", " << getLong() << ", " << getAlt() << endl;
  // process data and go to the loop
  
  delay(100);
  rtty_txstring("weather");
  delay(100);

  blink();
  delay(800);

}



