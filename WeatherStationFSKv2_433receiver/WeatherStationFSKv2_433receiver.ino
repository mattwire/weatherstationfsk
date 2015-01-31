
/// FSK weather station receiver
/// Receive packets echoes to serial.
/// Updates DCF77 time.
/// Supports Alecto WS3000, WS4000, Fine Offset WH1080 and similar 868MHz stations
/// National Geographic 265 requires adaptation of frequency to 915MHz band.
/// input handler and send functionality in code, but not implemented or used.
/// 2013-03-10<info@sevenwatt.com> http://opensource.org/licenses/mit-license.php
/// v2: adds sensor network functionality: transmits received data using the
/// standard JeeLabs network protocol

/// BMP085 sensor connected to jeenode port 2 pins2..5: SDA, GND, 3.3V, SCL

/// Readings: DHT,<Temp C>,<Humidity>,<Heat Index> ( http://en.wikipedia.org/wiki/Heat_index )
///           BMP,<Temp C>,<Pressure P>
///           NRS,<Address>,<UnitCode>,<Cmd:on/off/dim>,<Dimlevel>,<Period>
///           WS4,<ID>,<Temp C>,<rel Humidity>,<Wind Velocity>,<Wind Max>,<Wind compass>,<Wind bearing>,<Rain>
///           DCF,<DCF Time>

#include <JeeLib.h>
#include <RF12.h>
#include <Time.h>
#include <PortsBMP085.h>
#include <NewRemoteReceiver.h>
#include "DHT.h"

#define DHTPIN 6     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

//Define the printed output
//#define LOGRAW 1 //comment = disable, uncomment = enable logging received package group_id 212 (0xD4)
// #define LOGPKT 1 //comment = disable, uncomment = enable logging unique package passing crc
#define LOGDCF 1 //comment = disable, uncomment = enable updating time and logging of DCF77 values
#define LOGDAT 1 //comment = disable, uncomment = enable logging of parsed sensor data
#define SENDRFM12

//WH1080 V2 protocol defines
#define MSG_WS4000 40
#define MSG_WS3000 42
#define LEN_WS4000 10
#define LEN_WS3000 9
#define LEN_MAX 10

#define SERIAL_BAUD 9600
//#define LED_PIN     9   // activity LED, comment out to disable

#define NODE_ID 22 //any node works wih modified driver. Use 31 for unmodified driver.
#define GROUP_ID 173
#define DEST_NODE_ID 1 //1=central node, 0=broadcast

// BMP085 defines
PortI2C two (2);
BMP085 psensor (two, 3); // ultra high resolution
MilliTimer timer;
struct { int16_t temp; int32_t pres; } payload;

// Delay loop vars for sampling BMP085 sensor
unsigned long currMillis;
unsigned long prevMillis = 0;
long interval = 5000;

// Sensor averaging
const int numReadings = 5;
int16_t tempReadings[numReadings];
int32_t presReadings[numReadings];
int readingsIndex = 0;
int32_t presTotal = 0;
int16_t tempTotal = 0;
boolean bmpValid = 0; // Set to 1 after 5 readings

static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
#endif
}

//send parameters
static byte sendlen;
static byte sendbuf[RF12_MAXDATA]/*, testCounter*/;
static unsigned long ok_ts;
static byte packet_found = 0;
static byte ok_cnt = 0;
static byte pkt_cnt = 0;
static uint8_t packet[10];
static uint8_t msgformat = 0;
static uint8_t pktlen = LEN_MAX;
static uint8_t txcnt = 0;

//void configureWH1080 () {
//    rf12_restore(NODE_ID, RF12_868MHZ, 0xD4);
//    rf12_control(0x80E7); // 868 Mhz;enable tx register; enable RX
//                          // fifo buffer; xtal cap 12pf
//    rf12_control(0xA67C); // 868.300 MHz
//    //rf12_control(0xE105); // Wakeup timer 10ms
//    //rf12_control(0xC80E); // disable low dutycycle, but set at D=7 (3%) 
//    rf12_control(0xC49F); // AFC keep during receive VDI=high; enable AFC; enable
//                                // frequency offset register; +15 -16
//    rf12_control(0xC26A); // manual, fast mode, digital filter, DQD=2 
//    rf12_control(0xC613); // 17.24 kbps
//    rf12_control(0xC006); // 1.00MHz, 2.8V
//    rf12_control(0x94A0); // VDI; FAST; 134khz; Gain -0db; DRSSI 103dbm 
//    //rf12_control(0xCED4); // Synchron word = 0x2DD4
//    rf12_control(0xCA81); // FIFO intliength=8, sync on 0x2DD4.,
//                          // reset non-sensitive, disable FIFO
//    rf12_control(0x820D); // disable receiver 
//    rf12_control(0xCC67); // pll settings command
//    rf12_control(0xB800); // TX register write command not used
//    rf12_control(0x82DD); // enable receiver 
//    rf12_control(0xCA83); // FIFO intliength=8, sync on 0x2DD4.,
//                          // reset non-sensitive, enable FIFO
//    rf12_setFixedLength(LEN_MAX);  
//}

void configureWH1080 () {
    rf12_restore(NODE_ID, RF12_868MHZ, 0xD4); // group 212
    rf12_setBitrate(0x13);                    // 17.24 kbps
    rf12_control(0xA67C);                     // 868.300 MHz
    rf12_setFixedLength(LEN_MAX);             // receive fixed number of bytes  
}

static void do_tests() {
    Serial.println(F("Test time packet WS3000"));
    uint8_t testbuf1[] = {0x6D, 0x7A, 0x49, 0x04, 0x21, 0x13, 0x83, 0x04, 0xD6};
    byte crc_ok = testbuf1[8] == _crc8(testbuf1, 8);
    Serial.print(crc_ok ? F("crc  ok ") : F("crc nok "));
    update_time((uint8_t*)testbuf1);
    Serial.println(F("Test sensor packet WS3000"));
    //crc nok
    //uint8_t testbuf2[] = {0x5D, 0x70, 0x31, 0x78, 0x07, 0x0A, 0x03, 0x0C, 0xF4};
    //crc ok
    uint8_t testbuf2[] = {0x5D, 0x70, 0x2D, 0x41, 0x02, 0x05, 0x03, 0x0C, 0x4C};
    crc_ok = testbuf2[8] == _crc8(testbuf2, 8);
    Serial.print(crc_ok ? F("crc  ok ") : F("crc nok "));
    decodeSensorData(MSG_WS3000, testbuf2);
    //Serial.println(F("Test time packet WS4000"));
    //char testbuf[] = {0xB4, 0x00, 0x56, 0x03, 0x31, 0x13, 0xC3, 0x03, 0x45, 0x4C};
    //update_time((uint8_t*)testbuf);
    //Serial.println(F("Test sensor packet WS4000"));
    //char testbuf3[] = {0xA4, 0xF0, 0x3C, 0x48, 0x00, 0x00, 0x03, 0xC6, 0x04, 0x53};
    //decodeSensorData(MSG_WS4000, testbuf3);
    ////negative temperature
    //char testbuf4[] = {0xA4, 0xF8, 0x3C, 0x48, 0x00, 0x00, 0x03, 0xC6, 0x04, 0x53};
    //decodeSensorData(MSG_WS4000, testbuf4);  
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.print("\n[weatherstationFSK_BMP085_DHT22_433rx]\n");
    
    // Initialise readings
    for (int i = 0; i < numReadings; i++) {
      tempReadings[i] = 0; 
      presReadings[i] = 0;
    }
    
    activityLed(0);
    rf12_initialize(NODE_ID, RF12_868MHZ, GROUP_ID); //reset RFM12B and initialize
    configureWH1080();
    
    psensor.getCalibData();
    dht.begin();
    // See example ShowReceivedCode for info on this
    NewRemoteReceiver::init(1, 2, showCode);
    //test
    //do_tests();
}

void loop() {
    if (rf12_recvDone()) {
        //check crc
        pkt_cnt++;
        byte crc_ok = 0;
        uint8_t mt = rf12_buf[1] >> 4;
        switch (mt) {
          case 0x5:
          case 0x6: {
            crc_ok = rf12_buf[9] == _crc8(&rf12_buf[1], 8);
            if (crc_ok) ok_cnt++;
            msgformat = MSG_WS3000;
            pktlen = LEN_WS3000;
            break;
          }
          case 0xA:
          case 0xB: {
            crc_ok = rf12_buf[10] == _crc8(&rf12_buf[1], 9);
            if (crc_ok) ok_cnt++;
            msgformat = MSG_WS4000;
            pktlen = LEN_WS4000;
            break;
          }
          default: break; //crc_ok=0;
        }
        
#ifdef LOGRAW
        //Log all packages. Packages may be missed due to short intervals
        Serial.print(crc_ok ? " ok " : "nok ");
        for (byte i = 1; i < LEN_MAX+1; i++) {
            Serial.print(' ');
            Serial.print(rf12_buf[i] >> 4, HEX);
            Serial.print(rf12_buf[i] & 0x0F, HEX);
        }
        Serial.println();
        //Serial.println(millis());
#endif
        
        //save the first crc_ok package of a burst
        if ((!packet_found) && crc_ok){
          //start one second interval to count upto six identical packets.
          ok_ts=millis();
          packet_found = 1; //true
          memcpy(packet, (char *)&rf12_buf[1], 10);
        }
    }
    
    //Report if transmission is finished (38ms after first package detected)
    if (packet_found && ((long)(millis()-ok_ts) > 50)) {
      uint8_t mt = packet[0] >> 4;
#ifdef LOGDCF
      //Set time if time packet received
      if (mt == 0xB || mt == 0x6) {
        update_time(/*msgformat,*/ packet);
      }
#endif

#ifdef LOGPKT
      //Transmission of repeated packages is done. Report results.
      timestamp();
      Serial.print(" pkt_cnt: ");
      Serial.print(pkt_cnt);
      Serial.print(" ok_cnt: ");
      Serial.print(ok_cnt);
      Serial.print(" pkt: ");      
      for (byte i = 0; i < pktlen; i++) {
          //Serial.print(' ');
          Serial.print(packet[i] >> 4, HEX);
          Serial.print(packet[i] & 0x0F, HEX);
          //Serial.print((int) rf12_data[i]);
      }
      Serial.println();
#endif
#ifdef LOGDAT
      if (mt == 0xA || mt == 0x5) {
        //timestamp();
        decodeSensorData(msgformat, packet);
      }
#endif
#ifdef SENDRFM12
      //send the packet to the central node
      sendlen = 0;
      sendbuf[sendlen++] = txcnt++;
      sendbuf[sendlen++] = msgformat;
      sendbuf[sendlen++] = pktlen;
      memcpy(&sendbuf[sendlen], packet, pktlen);
      sendlen += pktlen;
      
      
      memcpy(&sendbuf[sendlen], &payload, sizeof(payload));
      sendlen += sizeof(payload);
      rf12_restore(NODE_ID, RF12_868MHZ, GROUP_ID);
      while (!rf12_canSend())
          rf12_recvDone(); // ignores incoming
      activityLed(1);
      rf12_sendStart(DEST_NODE_ID, sendbuf, sendlen, 1);
      configureWH1080();      
      //reset administration
      ok_cnt = pkt_cnt = 0;
      packet_found = 0; //false
      activityLed(0);
    }
#endif
    
    currMillis = millis();
    if (currMillis - prevMillis > interval) {
      // Reset delay
      prevMillis = currMillis;
      
      int32_t traw = psensor.measure(BMP085::TEMP);
      int32_t praw = psensor.measure(BMP085::PRES);
      
      // Sensor averaging
      // Subtract the oldest reading
      tempTotal = tempTotal - tempReadings[readingsIndex];
      presTotal = presTotal - presReadings[readingsIndex];
      // Get new values
      psensor.calculate(tempReadings[readingsIndex], presReadings[readingsIndex]);
      // Add new values to total
      tempTotal = tempTotal + tempReadings[readingsIndex];
      presTotal = presTotal + presReadings[readingsIndex];
      //set next index
      readingsIndex++;
      if (readingsIndex >= numReadings) {
        bmpValid = 1; // Set valid flag after first 5 readings
        readingsIndex = 0;
      }

      // Calc average
      payload.temp = tempTotal / numReadings;
      payload.pres = presTotal / numReadings;      

#ifdef LOGDAT
      if (bmpValid) {
        Serial.print("BMP,");
        Serial.print((double)payload.temp/10);
        Serial.print(",");
        Serial.print((double)payload.pres/100);
        Serial.println();
      }
#endif

      float h = dht.readHumidity();
      float t = dht.readTemperature();
      float hi = dht.convertFtoC(dht.computeHeatIndex(dht.convertCtoF(t),h));
      Serial.print("DHT,");
      Serial.print(t);
      Serial.print(",");
      Serial.print(h);
      Serial.print(",");
      Serial.print(hi);
      Serial.println();
    }
}

// Callback function is called only when a valid code is received.
void showCode(NewRemoteCode receivedCode) {
  Serial.print("NRS,");
  Serial.print(receivedCode.address);

  if (receivedCode.groupBit) {
    Serial.print("G,");
  } 
  else {
    Serial.print(receivedCode.unit);
    Serial.print(",");
  }

  switch (receivedCode.switchType) {
    case NewRemoteCode::off:
      Serial.print("off,");
      break;
    case NewRemoteCode::on:
      Serial.print("on,");
      break;
    case NewRemoteCode::dim:
      Serial.print("dim,");
      break;
  }

  if (receivedCode.dimLevelPresent) {
    Serial.print(receivedCode.dimLevel);
  }
  Serial.print(",");
  Serial.print(receivedCode.period);
  Serial.println();
}


/*
* Function taken from Luc Small (http://lucsmall.com), itself
* derived from the OneWire Arduino library. Modifications to
* the polynomial according to Fine Offset's CRC8 calulations.
*/
uint8_t _crc8(volatile uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	// Indicated changes are from reference CRC-8 function in OneWire library
	while (len--) {
		uint8_t inbyte = *addr++;
		uint8_t i;
		for (i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
			crc <<= 1; // changed from right shift
			if (mix) crc ^= 0x31;// changed from 0x8C;
			inbyte <<= 1; // changed from right shift
		}
	}
	return crc;
}

void timestamp()
{
  Serial.print(year()); 
  Serial.print("-");
  printDigits(month());
  Serial.print("-");
  printDigits(day());
  Serial.print(" "); 
  printDigits(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
  Serial.print(" ");
}

void printDigits(int digits){
  // utility function for digital clock display: leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

int BCD2bin(uint8_t BCD) {
  return (10 * (BCD >> 4 & 0xF) + (BCD & 0xF));
}

void update_time(uint8_t* tbuf) {
  setTime(BCD2bin(tbuf[2] & 0x3F),BCD2bin(tbuf[3]),BCD2bin(tbuf[4]),BCD2bin(tbuf[7]),BCD2bin(tbuf[6] & 0x1F),BCD2bin(tbuf[5]));
  Serial.print("DCF,");
  timestamp();
  Serial.println();
}

void decodeSensorData(uint8_t fmt, uint8_t* sbuf) {
    char *compass[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    uint8_t windcompass = 0;
    // station id
    uint8_t stationid = (sbuf[0] << 4) | (sbuf[1] >>4);
    // temperature
    uint8_t sign = (sbuf[1] >> 3) & 1;
    int16_t temp = ((sbuf[1] & 0x07) << 8) | sbuf[2];
    if (sign)
      temp = (~temp)+sign;
    double temperature = temp * 0.1; // Convert to C
    //humidity
    uint8_t humidity = sbuf[3] & 0x7F; // %
    //wind speed
    double windspeed = sbuf[4] * 0.34; // m/s
    //wind gust
    double windgust = sbuf[5] * 0.34; // m/s
    //rainfall
    double rain = (((sbuf[6] & 0x0F) << 8) | sbuf[7]) * 0.3;
    
    double windbearing = 0;
    if (fmt == MSG_WS4000) {
      //wind bearing
      windcompass = sbuf[8] & 0x0F;
      windbearing = windcompass * 22.5; // Convert 0-15 to degrees
    }

    if (fmt == MSG_WS4000) {
      Serial.print("WS4,");
      Serial.print(stationid);
      Serial.print(",");
      Serial.print(temperature);
      Serial.print(",");
      Serial.print(humidity);
      Serial.print(",");
      Serial.print(windspeed);
      Serial.print(",");
      Serial.print(windgust);
      Serial.print(",");
      Serial.print(compass[windcompass]);
      Serial.print(",");
      Serial.print(windbearing);
      Serial.print(",");
      Serial.println(rain);
    }
    if (fmt == MSG_WS3000) {
      Serial.print("WS3 ID=");
      Serial.print(stationid);
      Serial.print(" T=");
      Serial.print(temperature);
      Serial.print(" relH=");
      Serial.print(humidity);
      Serial.print(" Wvel=");
      Serial.print(windspeed);
      Serial.print(" Wmax=");
      Serial.print(windgust);
      Serial.print(" Rain=");
      Serial.println(rain);
    }
}
