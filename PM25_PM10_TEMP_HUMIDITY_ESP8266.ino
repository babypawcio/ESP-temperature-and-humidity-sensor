#include <SoftwareSerial.h>                                                                                                   // Serial display library for PMS7003 data
#include <LiquidCrystal_I2C.h>                                                                                                // 20 x 4 LCD display library
#include "PMS.h"                                                                                                              // PMS7003 PM2.5 & PM10 laser sensor library
#include "DHT.h"                                                                                                              // DHT Temp & Humidity sensor libraty

#define PMS7003_TX          D5                                                                                                // GPIO12
#define PMS7003_RX          D6                                                                                                // GPIO14
#define PMS7003_PREAMBLE_1  0x42                                                                                              // From PMS7003 datasheet
#define PMS7003_PREAMBLE_2  0x4D
#define PMS7003_DATA_LENGTH 31                                                

#define SENSOR D3                                                                                                             // DHT Sensor PIN
#define MODEL DHT11                                                                                                           // DHT Sensor model (DHT11)

const static int pause = 10000;                                                                                               // Instructions delay
int PM25, PM10;                                                                                                               // Values for PM2.5 & PM10 readings

DHT dht(SENSOR, MODEL);                                                                                                       // Initializing/Constructing DHT sensor object

LiquidCrystal_I2C lcd(0x27,2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);                                                                 // Initializing/Constructing LCD display object 

SoftwareSerial _serial(PMS7003_TX, PMS7003_RX);                                                                               // PMS object construction |  RX, TX connections with ESP8266

void readSensor();                                                                                                            // Reading data from PMS7003

void setup()                                                                                                                  // Initial settings
{
  lcd.begin(20, 4);                                                                                                           // Initializing 20x4 LCD display
  lcd.setCursor(2,1);                                                                                                         // Setting cursor on 2nd row & 6th row (indexing from 0)
  lcd.print(F("Uruchamianie..."));                                                                                            // Displaying text on an LCD screen
  lcd.setCursor(1,2);
  lcd.print(F("10 sek opoznienia"));

  Serial.begin(115200);                                                                                                       // Serial monitor default settings for debugging on an ESP8266
  _serial.begin(9600);                                                                                                        // "Serial monitor" settings for communicating with a PMS7003 sensor
  dht.begin();                                                                                                                // Initializing a DHT sensor

  delay(pause);

  lcd.clear();                                                                                                                // Clearing screen before showing values from the loop function
}

void loop()
{
  float humidity = dht.readHumidity();                                                                                        // Reading humidity from the DHT sensor
  float temperature = dht.readTemperature();                                                                                  // Reading temperature from the DHT sensor

  readSensor();                                                                                                               // Working/Correct PMS function
  Serial.print(F("\nPM2.5 : "));
  Serial.println(PM25);
  Serial.print(F("\nPM10 : "));
  Serial.println(PM10);

  lcd.setCursor(0, 0);
  lcd.print("PM 2.5 : " + String(PM25) + " (ug/m3)");                                                                         // Printing PMx values on the display
  lcd.setCursor(0, 1);
  lcd.print("PM 10  : " + String(PM10) + " (ug/m3)");

  lcd.setCursor(0, 2);
  if (isnan(temperature)) {                                                                                                    // If the temperature is not read correctly
    lcd.print(F("Temperatura : NaN"));
    return;
  }

  else {
    lcd.print("Temperatura : " + String(temperature) + "C");                                                                   // If the temperature variable value is right
  }

  lcd.setCursor(0, 3);
  if (isnan(humidity)) {                                                                                                       // If the humidity is not read correctly
    lcd.print(F("Wilgotnosc : NaN"));
    return;
  }

    else {
      lcd.print("Wilgotnosc  : " + String(humidity) + "%");                                                                    // If the humidity variable value is right
    }

  delay(pause);                                                                                                                // Pause for 10 seconds before next "iteration"
}

void readSensor() {                                                                                                            // Function that changes PM25 and PM10 variables values to accurate data given from PMS sensor
                                                                                                                               // Stolen from the internet, can't exactly tell what it does
                                                                                                                               
  int checksum = 0;
  unsigned char pms[32] = {0,};

  while( _serial.available() &&                                                                                                // Solve trouble caused by delay function                
      _serial.read() != PMS7003_PREAMBLE_1 &&
      _serial.peek() != PMS7003_PREAMBLE_2 ) {
  }		

  if( _serial.available() >= PMS7003_DATA_LENGTH ){                                                                             // If serial data is not smaller than data length, then do:
    pms[0] = PMS7003_PREAMBLE_1;                                                                                                // first element is PMS Preamble
    checksum += pms[0];                                                                                                         // Increase checksum

    for(int j=1; j<32 ; j++){                                                                                                   // For max 32 value length word
      pms[j] = _serial.read();                                                                                                  // J indexed element is element read from serial sensor
      if(j < 30)                                                                                                                // If index is smaller than 30, then increase checksum by J indexed element
        checksum += pms[j];
    }
    _serial.flush();
    if( pms[30] != (unsigned char)(checksum >> 8)                                                                               // If thirty first element is not a checksum positive character moved by 8 bits to the right (divided by 2^8)
      || pms[31] != (unsigned char)(checksum) ){                                                                                // Or if thirty second element is not a checksum positive character, then
      Serial.println("Checksum error");
      return;
    }
    if( pms[0] != 0x42 || pms[1] != 0x4d ) {                                                                                    // If first character is not 66 / B or second character is not 77 / M, then
      Serial.println("Packet error");
      return;
    }

    PM25 = makeWord(pms[12],pms[13]);                                                                                           // PM25 value is 16 bit word created from 2 bit words (from value under pms[12] and another one from pms[13])
    PM10 = makeWord(pms[14],pms[15]);                                                                                           // PM10 value is 16 bit word created from 2 bit words (from value under pms[14] and another one from pms[15])
  }		
}