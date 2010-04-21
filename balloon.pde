#include <NewSoftSerial.h>
#include <Wire.h>


// setup some stuff
const int compass = (0x32 >> 1); 
int heading = 0;

char weather[100];
const int weather_rx = 9;
const int weather_tx = 8;

byte led_state = 0;

// set up a new serial port
NewSoftSerial weather_serial(weather_rx, weather_tx);

void setup() {
  weather_serial.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  delay(3000);
}

void blink() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
}

void toggle() {
  digitalWrite(13, led_state); 
  led_state = !led_state;
}


int get_heading() { 
  byte headingData[6]; 
  Wire.beginTransmission(compass); 
  Wire.send(0x50); 
  Wire.endTransmission(); 
  delay(2); 
  Wire.requestFrom(compass, 6); 
  int i = 0; 
  while(Wire.available() && i < 6) 
  { 
    headingData[i] = Wire.receive(); 
    i++; 
  } 
  return (headingData[0]*256 + headingData[1])/10; 
}

void set_weather() {
  int i = 0;
  int stop = 0;
  int started = 0;
  
  do {
    weather[i] = weather_serial.read();
    if('#' == weather[i]) {
      i = 0;
      started = 1;
    }
    if(started && '$' == weather[i])
      stop = 1;
    i++;
  } while(!stop);
  return;
}

void loop() {

  set_weather();
  Serial.println(weather);
  blink();
  delay(1500);

  heading = get_heading();
  Serial.println(heading);
  blink();
  delay(1000);

}



