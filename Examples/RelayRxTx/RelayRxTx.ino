//
// Mark Crossley 2017
//

#include <Arduino.h>

#include <SPI.h>
#include <EEPROM.h>

#include <Wire.h>

#include "DavisRFM69.h"
#include "PacketFifo.h"
#include "MsTimer2.h"

#define LED 9                // Moteino has LED on pin 9
#define LED_INTERVAL 1000L   // LED flash duration micros

#define SERIAL_BAUD 115200

#define TX_ID 2 // 0..7, Davis transmitter ID, set to a different value than all other transmitters
                // IMPORTANT: set it ONE LESS than you'd set it on the ISS via the DIP switch; 1 here is 2 on the ISS/Davis console

#define NUM_RX_STATIONS 2   // Number of stations we are going to listen for

#define TX_LED true         // Flash the LED for every received packet
#define RX_LED true         // Flash the LED for every transmitted packet

#define SERBUF_LEN 10       // Maximum length of commands on serial interface
#define S_OK  true
#define S_ERR false

#define CONFIG_VERSION "rt2" // ID of the settings block
#define CONFIG_START 32      // Where to store the config data in EEPROM

// Example settings structure
struct StoreStruct {
  // This is for detection if they are our settings
  char version[4];
  // The variables of our settings
  byte output;
  char filter[8];
  float timer;
  int frequency;
} storage = {
  CONFIG_VERSION,
  // The default values
  1,
  {'1','1','1','1','1','1','1','1'},
  1,
  0
};

// Payload data structure for transmitting data
// Unconnected data contents are...
// uv:      40-00-00-ff-c5-00
// rainsec: 50-00-00-ff-71-00
// solar:   60-00-00-ff-c5-00
// temp:    80-00-00-ff-c5-00
// gust:    90-00-00-00-05-00
// rh:      a0-00-00-00-05-00
// rain:    e0-00-00-80-03-00
// unknown: c0-00-00-00-01-00
// for an unconnected wind sensor wind speed and direction are both 0
typedef struct __attribute__((packed)) Payload {
  byte wind[3];
  byte uv[6];
  byte rainsecs[6];
  byte solar[6];
  byte temp[6];
  byte windgust[6];
  byte hum[6];
  byte rain[6];
};

typedef struct __attribute__((packed)) PayloadStation {
  byte wind;
  byte uv;
  byte rain;
  byte solar;
  byte temp;
  byte hum;
};

// Observed sequence of transmitted ISS value types.
// The upper nibble is important, the lower nibble is the transmitter ID + battery flag.
// Type values for a standard VP2 ISS:
//     0x80 0xe0 0x40 0xa0 0x60 0x50 0x90 [0xc0]
//     temp rain uv   rh   sol  rsec gust [unkown]
// Wind speed and direction is transmitted in every packet at byte offsets 1 and 2.
static const byte txseq[20] = {
  0x8, 0xe, 0x5, 0x4,
  0x8, 0xe, 0x5, 0x9,
  0x8, 0xe, 0x5, 0x6,  // last packet on this row is 0xc on new transmitters, set it to what you want
  0x8, 0xe, 0x5, 0xa,
  0x8, 0xe, 0x5, 0x6
};


// DavisRFM69 radio base class;
DavisRFM69 radio;

// txPeriod is a function of the ID and some constants, in millisecs
// starts at 2.5625 and increments by 0.625 up to 3.0 for every increment in TX_ID
// DAVIS_INTV_CORR is defined in DavisRFM69.h

uint32_t txPeriod;
uint32_t lastTx;
uint32_t txPackets = 0;
byte     seqIndex;      // current packet type index into txseq array
byte     channel;       // next transmission channel
byte     digits0 = 1;   // used for print formatting
byte     digits1 = 1;   // used for print formatting
byte     digits2 = 1;   // used for print formatting
bool     ledOn;         // tracks LED status
uint32_t ledTimer;      // last time LED was switched on

// initialise the sensor payloads with 'sensor not connected' data
Payload payloads = {
//{0x03, 0xA8, 0x03},                    // dummy test wind data
  {0, 0, 0},                             // wind
  {0x40, 0x00, 0x00, 0xff, 0xc5, 0x00},  // uv
  {0x50, 0x00, 0x00, 0xff, 0x71, 0x00},  // rainsecs
  {0x60, 0x00, 0x00, 0xff, 0xc5, 0x00},  // solar
  {0x80, 0x00, 0x00, 0xff, 0xc5, 0x00},  // temp
  {0x90, 0x00, 0x00, 0x00, 0x05, 0x00},  // gust
  {0xa0, 0x00, 0x00, 0x00, 0x05, 0x00},  // humidity
  {0xe0, 0x00, 0x00, 0x80, 0x03, 0x00}   // rain
};

// station id associated with each payload
PayloadStation payloadStations = {
  255, 255, 255, 255, 255, 255   // wind, uv, rain, solar, temp, hum
};

// Stations to receive from
// id, type, active
Station stations[NUM_RX_STATIONS] = {
  {0, STYPE_ISS, true},   // Anemometer txmr in my case
  {1, STYPE_ISS, true}    // The 'real' ISS
};

byte battery[NUM_RX_STATIONS];  // Array to hold battery status from each transmitter

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Get the default values for the config
  loadConfig();

  radio.setStations(stations, NUM_RX_STATIONS);
  radio.initialize(FREQ_BAND_US);
  radio.setBandwidth(RF69_DAVIS_BW_NARROW);
  radio.setTimerCalibation(storage.timer);
  radio.setFrequencyCalibation(storage.frequency);

  // set the payload transmitter ids to match our transmitter id
  payloads.uv[0]       = payloads.uv[0]       + TX_ID;
  payloads.rainsecs[0] = payloads.rainsecs[0] + TX_ID;
  payloads.solar[0]    = payloads.solar[0]    + TX_ID;
  payloads.temp[0]     = payloads.temp[0]     + TX_ID;
  payloads.windgust[0] = payloads.windgust[0] + TX_ID;
  payloads.hum[0]      = payloads.hum[0]      + TX_ID;
  payloads.rain[0]     = payloads.rain[0]     + TX_ID;



  txPeriod = (long)((41 + TX_ID) * 1000.0 * storage.timer / 16.0);
  seqIndex = 0;
  channel = 0;
  ledOn = false;
  ledTimer = 0;

  for (byte i=0; i < NUM_RX_STATIONS; i++) {
    battery[i] = 0;
  }

  MsTimer2::set(txPeriod, sendNextPacket);
  MsTimer2::start();

  lastTx = micros();

  Serial.println("Setup done.");
  // print a header for the serial data log
  Serial.println("time\tRx/Tx\traw_packet_data              \tid\tpacket_counts\tchan\trssi\tfei\tdelta_t\tbatt\twind\tdir\tsensor\tvalue");
}


// Main loop
void loop() {
  while (radio.fifo.hasElements()) {
    decode_packet(radio.fifo.dequeue());
  }

  // Any serial commands to process?
  processSerial();

  // Time to switch the LED off?
  if (ledOn && micros() - ledTimer > LED_INTERVAL) {
    ledOnOff(false);
  }
}



void decode_packet(RadioData* rd) {
  int val;
  byte* packet = rd->packet;
  byte *ptr;
  bool copyData = false;
  byte id = packet[0] & 7;

  // TODO: Need to detect loss of reception from a transimitter and then send 'not connected' packets for the sensors associated
  //       with that transmitter id until contact is regained. At present the code 'flat-lines' with the last received values

  if (id == TX_ID) {  // Transmission packet
   if (TX_LED) {
      ledOnOff(true);
    }
    // dump it to the serial port
    printIt(rd->tim, rd->packet, 'T', rd->channel, 0, 0, 0, 0, rd->delta, 0);
  } else {           // Received packet
    if (RX_LED) {
      ledOnOff(true);
    }

    // save wind data from every packet type - if valid
    if ((packet[2] | (packet[4] & 2)) != 0 || id == payloadStations.wind) {
      payloads.wind[0] = packet[1];
      payloads.wind[1] = packet[2];
      payloads.wind[2] = packet[4];
      if (id != payloadStations.wind) {
        payloadStations.wind = id;
      }
    }

    // save battery status
    battery[id] = packet[0] & 8;


    switch (packet[0] >> 4) {
      case VP2P_UV:
        val = word(packet[3], packet[4]) >> 6;
        if (val < 0x3ff || id == payloadStations.uv) {
          copyData = true;
          ptr = payloads.uv;
          if (id != payloadStations.uv) {
            payloadStations.uv = id;
          }
       }
       break;

      case VP2P_SOLAR:
        val = word(packet[3], packet[4]) >> 6;
        if (val < 0x3fe || id == payloadStations.solar) {
          copyData = true;
          ptr = payloads.solar;
          if (id != payloadStations.solar) {
            payloadStations.solar = id;
          }
        }
        break;

      case VP2P_RAIN:
        if (packet[3] != 0x80 || id == payloadStations.rain) {
          copyData = true;
          ptr = payloads.rain;
          if (id != payloadStations.rain) {
            payloadStations.rain = id;
          }
        }
        break;

      case VP2P_RAINSECS:
        // PROBLEM - We need to store the data from the station when it resets to the 'not connected' value if it isn't raining,
        //         - but NOT from any other stations that may be reporting 'not connected' because they aren't rain stations!
        val = (packet[4] & 0x30) << 4 | packet[3];
        // is the packet from the station that previuously reported rain, OR do we have some rain data
        if (val != 0x3ff || id == payloadStations.rain) {
          // we have some rain data
          copyData = true;
          ptr = payloads.rainsecs;
          if (id != payloadStations.rain) {
            payloadStations.rain = id;
          }
        }
        break;

      case VP2P_TEMP:
        if (packet[3] != 0xff || id == payloadStations.temp) {
          copyData = true;
          ptr = payloads.temp;
          if (id != payloadStations.temp) {
            payloadStations.temp = id;
          }
        }
        break;

      case VP2P_HUMIDITY:
        val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
        if (val > 0 || id == payloadStations.hum) {
          copyData = true;
          ptr = payloads.hum;
          if (id != payloadStations.hum) {
            payloadStations.hum = id;
          }
        }
        break;

      case VP2P_WINDGUST:
        if (packet[2] > 0 || id == payloadStations.wind) {  // 0 -> no sensor
          copyData = true;
          ptr = payloads.windgust;
          if (id != payloadStations.wind) {
            payloadStations.wind = id;
          }
        }
        break;

      case VP2P_SOIL_LEAF:
        break;
      case VUEP_VCAP:
        break;
      case VUEP_VSOLAR:
        break;
    }

    if (copyData) {
      memcpy(ptr + 1, (byte *)packet + 1, 5);
    }

    // dump it to the serial port
    printIt(rd->tim, rd->packet, 'R', rd->channel, stations[id].packets, stations[id].missedPackets, rd->rssi, rd->fei, rd->delta, stations[id].numResyncs);
  }
}

void printHex(volatile byte* packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
    Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print('-');
  }
}


void sendNextPacket() {
  byte *ptr;
  uint32_t now, delta;

  // disable interrrupts so receiving packets does not distrupt the transmission.
  noInterrupts();

  // turn off receiver to prevent reception while we prepare to transmit
  radio.setMode(RF69_MODE_STANDBY);

  if (TX_LED) {
    ledOnOff(true);
  }

  switch (txseq[seqIndex]) {
    case VP2P_UV:
      ptr = payloads.uv;
      break;
    case VP2P_SOLAR:
      ptr = payloads.solar;
      break;
    case VP2P_RAIN:
      ptr = payloads.rain;
      break;
    case VP2P_RAINSECS:
      ptr = payloads.rainsecs;
      break;
    case VP2P_TEMP:
      ptr = payloads.temp;
      break;
    case VP2P_HUMIDITY:
      ptr = payloads.hum;
      break;
    case VP2P_WINDGUST:
      ptr = payloads.windgust;
      break;
  }

  // set the battery status, if any are set, then set it in relayed data
  // if any station has the flag set, then re-transmit it
  bitClear(ptr[0], 3);
  for (byte i = 0; i < NUM_RX_STATIONS; i++) {
    ptr[0] = ptr[0] | battery[i];
  }

  // Add in the latest wind data
  ptr[1] = payloads.wind[0];
  ptr[2] = payloads.wind[1];
//  ptr[4] = (ptr[4] & 0xfe) | (payloads.wind[2] & 0x1);
  ptr[4] = (ptr[4] & 0xfc) | (payloads.wind[2] & 0x3);

  // clear the flag if wind data valid
  if (ptr[2] != 0) {
    bitClear(ptr[4], 2);
  }

  // Send packet
  radio.send(ptr, channel);

  // Get current time
  now = micros();

  // re-enable interrrupts
  interrupts();

  // dump it to the fifo queue for printing
  radio.fifo.queue(now, (byte*)radio.DATA, channel, 0, 0, now - lastTx);

  // record the last txmt time
  lastTx = now;

  // move things on for the next transmission...
  channel = radio.nextChannel(channel);
  seqIndex = nextPktType(seqIndex);

  // count how many packets we have sent - just for interest
  txPackets++;
}


// Calculate the next packet type to transmit
byte nextPktType(byte packetIndex) {
  return ++packetIndex % sizeof(txseq);
}


// Dump data to serial port
void printIt(uint32_t tim, byte *packet, char rxTx, byte channel, uint32_t packets, uint32_t lostPackets, byte rssi, int16_t fei, uint32_t delta, uint32_t resyncs) {
  int val;
  byte *ptr;
  byte i;
  byte id = packet[0] & 7;

  // want to print this?
  if (storage.output == 0 || storage.filter[id] == '0') {
    return;
  }

  if (storage.output == 3) {
    if (rxTx == 'R' && (packet[2] | (packet[4] & 2) != 0)) {
      // wind data only
      int stIx = rxTx == 'R' ? radio.findStation(id) : TX_ID;
      if (stations[stIx].type == STYPE_VUE) {
        val = (packet[2] << 1) | (packet[4] & 2) >> 1;
        val = round(val * 360 / 512);
      } else {
        val = 9 + round((packet[2] - 1) * 342.0 / 255.0);
      }
      Serial.print(packet[1]);
      Serial.print(',');
      Serial.println(val);
    }
    return;
  }

  if (storage.output == 1) {
    Serial.print(tim);
    Serial.print('\t');
    Serial.print(rxTx);
    Serial.print('\t');
    printHex(packet, 10);
    Serial.print('\t');

    Serial.print(id + 1);
    Serial.print('\t');

    if (rxTx == 'R') {
      digits0 = Serial.print(resyncs);
      Serial.print('/');
      digits1 = Serial.print(packets);
      Serial.print('/');
      digits2 = Serial.print(lostPackets);
      Serial.print('/');
      Serial.print((float)(packets * 100.0 / (packets + lostPackets)), 1);
    } else {
      for (i=0; i < digits0; i++) {
        Serial.print('-');
      }
      Serial.print('/');
      for (i=0; i < digits1; i++) {
        Serial.print('-');
      }
      Serial.print('/');
      for (i=0; i < digits2; i++) {
        Serial.print('-');
      }
      Serial.print("/----");
    }
    Serial.print('\t');

    Serial.print(channel);
    Serial.print('\t');
    if (rxTx == 'R') {
      Serial.print(-rssi);
    } else {
      Serial.print("-");
    }
    Serial.print('\t');
    if (rxTx == 'R') {
      Serial.print(round(fei * RF69_FSTEP / 1000));
    } else {
      Serial.print("-");
    }
    Serial.print('\t');
    Serial.print(delta);
    Serial.print('\t');

    Serial.print((char*)(packet[0] & 0x8 ? "bad" : "ok"));
    Serial.print('\t');

    // All packet payload values are printed unconditionally, either properly
    // calculated or flagged with a special "missing sensor" value '-'.

    int stIx = rxTx == 'R' ? radio.findStation(id) : TX_ID;

    // wind data is present in every packet, windd == 0
    // (packet[2] == 0) means there's no anemometer??? No, get packet[2]=0 when direction from the North!
    if ((packet[2] | (packet[4] & 2) != 0))  {
      if (stations[stIx].type == STYPE_VUE) {
        val = (packet[2] << 1) | (packet[4] & 2) >> 1;
        val = round(val * 360 / 512);
      } else {
        val = 9 + round((packet[2] - 1) * 342.0 / 255.0);
      }
      Serial.print(packet[1]);
      Serial.print('\t');
      Serial.print(val);
      Serial.print('\t');
    } else {
      Serial.print("-\t-\t");
    }
  }

  switch (packet[0] >> 4) {

    case VP2P_UV:
      Serial.print("uv\t");
      if (packet[3] == 0xff) {
        Serial.print('-');
      } else {
        val = (packet[3] << 2) + (packet[4] >> 6);
        Serial.print((float)(val / 50.0), 1);
      }
      break;

    case VP2P_SOLAR:
      Serial.print("solar\t");
      if (packet[3] == 0xff) {
        Serial.print('-');
      } else {
        val = (packet[3] << 2) + (packet[4] >> 6);
        Serial.print(round(val * 1.758));
      }
      break;

    case VP2P_RAIN:
      Serial.print("rain\t");
      if (packet[3] == 0x80) {
        Serial.print('-');
      } else {
        Serial.print(packet[3]);
      }
      break;

    case VP2P_RAINSECS:
      Serial.print("r_secs\t");
      // light rain:  byte4[5:4] as value[9:8] and byte3[7:0] as value[7:0] - 10 bits total
      // strong rain: byte4[5:4] as value[5:4] and byte3[7:4] as value[3:0] - 6 bits total
      val = (packet[4] & 0x30) << 4 | packet[3];
      if (val == 0x3ff) {
        Serial.print('-');
      } else {
        // packet[4] bit 6: strong == 0, light == 1
        if ((packet[4] & 0x40) == 0) {
          val >>= 4;
        }
        Serial.print(val);
      }
      break;

    case VP2P_TEMP:
      Serial.print("temp\t");
      if (packet[3] == 0xff) {
        Serial.print('-');
      } else {
        val = (int)packet[3] << 4 | packet[4] >> 4;
        Serial.print((float)(val / 10.0), 1);
        Serial.print(" (");
        Serial.print((float)(((val / 10.0) - 32) * (5.0 / 9.0)), 1);
        Serial.print(')');
      }
      break;

    case VP2P_HUMIDITY:
      Serial.print("hum\t");
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
      if (val == 0) {
        Serial.print('-');
      } else {
        Serial.print(val + 1);
      }
      break;

    case VP2P_WINDGUST:
      Serial.print("gust\t");
      if (packet[2] == 0) {
        Serial.print('-');
      } else {
        Serial.print(packet[3]);
        if (packet[3] != 0) {
          Serial.print("\tgustref:");
          Serial.print(packet[5] & 0xf0 >> 4);
        }
      }
      break;

    case VP2P_SOIL_LEAF:
      // Not implemented, see...
      // https://github.com/cmatteri/CC1101-Weather-Receiver/wiki/Soil-Moisture-Station-Protocol
      /*
      byte subType;
      byte port;
      subType = packet[1] & 0x03;
      port = (packet[1] >> 5) & 0x07;
      Serial.print("soilleaf\t");
      if (subType == 1) {
	      val = (packet[2] << 2) + (packet[4] >> 6);    // Soil Moisture
	      Serial.print((String)(32+port) + ": " + val + " ");
	      val = (packet[3] << 2) + (packet[5] >> 6);    // Soil Temp
	      Serial.print((String)(36+port) + ": " + val);
      } else {
	      Serial.print((String)(40+port) + ": -");
      }
      */
      break;

    case VUEP_VCAP:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      Serial.print("vcap\t");
      Serial.print((float)(val / 300.0));
      break;

    case VUEP_VSOLAR:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      Serial.print("vsolar\t");
      Serial.print((float)(val / 300.0));
      break;

    default:
      Serial.print("unknown");
  }

  Serial.println();

}

void ledOnOff(bool on) {
  pinMode(LED, OUTPUT);
  if (on) {
    if (!ledOn) {
      ledOn = true;
      digitalWrite(LED, HIGH);
    }
    ledTimer = micros();
  } else if (ledOn) {
    digitalWrite(LED, LOW);
    ledOn = false;
  }
}


void printStatus(bool st, char *info = NULL) {
  Serial.print(st ? F("# OK") : F("# ERR"));
  if (info != NULL) {
    Serial.print(' ');
    Serial.print(info);
  }
  Serial.println();
}

void printStatusF(bool st, const __FlashStringHelper *info = NULL) {
  Serial.print(st ? F("# OK") : F("# ERR"));
  if (info != NULL) {
    Serial.print(' ');
    Serial.print(info);
  }
  Serial.println();
}


void processSerial() {
  int c;
  static int i;
  static char s[SERBUF_LEN];

  if (Serial.available()) {
    c = Serial.read();

    if (c == '\r' || c == '\n') {
      // ack empty command with OK
      if (i == 0) {
        printStatus(S_OK);
        return;
      }

      s[i] = 0;
      i = 0;

      switch(s[0]) {
        case 't':
          cmdTimer(s + 1);
          break;
        case 'o':
          cmdOutput(s + 1);
          break;
        case 'f':
          cmdFilter(s + 1);
          break;
        case 'q':
          cmdFreq(s + 1);
          break;
        case '?':
          cmdShowValues();
          break;
        case 'r':
          cmdShowRadio();
          break;
        default:
          printStatusF(S_ERR, F("bad command"));
      }
    } else {
      if (i < SERBUF_LEN - 1) {
        s[i++] = c;
      } else {
        s[0] = 0;
        i = 0;
        Serial.println();
        printStatusF(S_ERR, F("bad command"));
      }
    }
  }
}


void cmdShowValues() {
  Serial.print(F("# OK "));
  Serial.print('t');
  Serial.print(storage.timer, 5);
  Serial.print(F(" o"));
  Serial.print(storage.output);
  Serial.print(F(" f"));
  for (unsigned int i=0; i<8; i++) {
    Serial.print(storage.filter[i]);
  }
  Serial.print(F(" q"));
  Serial.print(storage.frequency);
  Serial.println();
}


void cmdTimer(char *s) {
  float t = atof(s);
  if (t < 0.8 || t > 1.2) {
    printStatusF(S_ERR, F("out of range"));
    return;
  }
  txPeriod = (long)((41 + TX_ID) * 1000.0 / 16.0 * t);
  MsTimer2::setTimeout(txPeriod);
  radio.setTimerCalibation(t);
  storage.timer = t;
  saveConfig();
  Serial.print(F("# OK t"));
  Serial.println(t, 5);
}


void cmdOutput(char *s) {
  if (s[0] == '0') {
    storage.output = 0;
    printStatusF(S_OK, F("output off"));
  } else if (s[0] == '1') {
    storage.output = 1;
    printStatusF(S_OK, F("output on"));
  } else if (s[0] == '2') {
    storage.output = 2;
    printStatusF(S_OK, F("output data only"));
  } else if (s[0] == '3') {
    storage.output = 3;
    printStatusF(S_OK, F("output wind data only"));
  } else {
    printStatusF(S_ERR, F("unimplemented"));
    return;
  }
  saveConfig();
}


void cmdFilter(char *s) {
  if (strlen(s) > 8) {
    printStatusF(S_ERR, F("too long"));
  } else if (s[0] == '\n') {
    printStatusF(S_ERR, F("no filters"));
  } else {
    Serial.print("# OK f");
    for (byte i=0; i < strlen(s); i++) {
      if (s[i] == '1' || s[i] == '0') {
        storage.filter[i] = s[i];
        Serial.print(s[i]);
      } else {
        Serial.println();
        printStatusF(S_ERR, F("not 0 or 1"));
        break;
      }
    }
    Serial.println();
    saveConfig();
  }
}


void cmdFreq(char *s) {
  int q = atoi(s);
  if (q < -1000 || q > 1000) {
    printStatusF(S_ERR, F("out of range"));
    return;
  }
  radio.setFrequencyCalibation(q);
  storage.frequency = q;
  saveConfig();
  Serial.print(F("# OK q"));
  Serial.println(q);
}

void cmdShowRadio() {
  Serial.println("id\tlastRx\tlastSeen\tinterval\tresyncs\tpackets\tmissed\tlost");
  for (int i = 0; i < NUM_RX_STATIONS; i++) {
    Serial.print(stations[i].id);
    Serial.print('\t');
    Serial.print(stations[i].lastRx);
    Serial.print('\t');
    Serial.print(stations[i].lastSeen);
    Serial.print('\t');
    Serial.print(stations[i].interval);
    Serial.print('\t');
    Serial.print(stations[i].numResyncs);
    Serial.print('\t');
    Serial.print(stations[i].packets);
    Serial.print('\t');
    Serial.print(stations[i].missedPackets);
    Serial.print('\t');
    Serial.println(stations[i].lostPackets);
  }
  // and the transmitter
    Serial.print(TX_ID);
    Serial.print('\t');
    Serial.print(lastTx);
    Serial.print('\t');
    Serial.print("---");
    Serial.print('\t');
    Serial.print(txPeriod * 1000); // convert ms to micros for compatibility with rx ids
    Serial.print("\t---\t");
    Serial.print(txPackets);
    Serial.println("\t---\t---");
}

void loadConfig() {
  // To make sure there are settings, and they are OURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t=0; t<sizeof(storage); t++) {
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
    }
  } else {
    storage.timer = radio.getTimerCalibation();
    storage.frequency = radio.getFrequencyCalibation();
  }
}


void saveConfig() {
  for (unsigned int t=0; t<sizeof(storage); t++) {
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
  }
}

